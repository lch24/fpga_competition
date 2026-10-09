#include "internal.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <numeric>
namespace chessboard {
void merge_duplicates(std::vector<Point2f>& points, float radius) {
    std::vector<bool> used(points.size(), false);
    std::vector<Point2f> out;
    for (size_t i = 0; i < points.size(); ++i) {
        if (used[i])
            continue;
        Point2f sum = points[i];
        int n = 1;
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (!used[j] && distance(points[i], points[j]) < radius) {
                used[j] = true;
                sum.x += points[j].x;
                sum.y += points[j].y;
                ++n;
            }
        }
        out.push_back({sum.x / n, sum.y / n});
    }
    points = std::move(out);
}
static void harris(const GrayImage& img, std::vector<Point2f>& points) {
    int w = img.w, h = img.h;
    std::vector<int> gx(w * h), gy(w * h);
    auto at = [&](int x, int y) { return int(img.get(std::clamp(x, 0, w - 1), std::clamp(y, 0, h - 1))); };
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            gx[y * w + x] = -at(x - 1, y - 1) + at(x + 1, y - 1) - 2 * at(x - 1, y) + 2 * at(x + 1, y) -
                            at(x - 1, y + 1) + at(x + 1, y + 1);
            gy[y * w + x] = -at(x - 1, y - 1) - 2 * at(x, y - 1) - at(x + 1, y - 1) + at(x - 1, y + 1) +
                            2 * at(x, y + 1) + at(x + 1, y + 1);
        }
    std::vector<int64_t> score(w * h);
    int64_t maximum = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            int64_t a = 0, b = 0, c = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= w || yy >= h)
                        continue;
                    int64_t ix = gx[yy * w + xx], iy = gy[yy * w + xx];
                    a += ix * ix;
                    b += ix * iy;
                    c += iy * iy;
                }
            // k=1/25. Exact integer score, bounded well inside signed 64 bits.
            int64_t s = 25 * (a * c - b * b) - (a + c) * (a + c);
            score[y * w + x] = s;
            maximum = std::max(maximum, s);
        }
    if (maximum <= 0)
        return;
    for (int y = 2; y < h - 2; ++y)
        for (int x = 2; x < w - 2; ++x) {
            int64_t s = score[y * w + x];
            // Shi response scales with contrast^2; Harris with contrast^4.
            // Fixed relative threshold: 0.08 squared = 4/625.
            if (s <= 0 || 625 * s < 4 * maximum)
                continue;
            bool peak = true;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    if (score[(y + dy) * w + x + dx] > s)
                        peak = false;
            if (peak)
                points.push_back({float(x), float(y)});
        }
}
static bool ring(const GrayImage& img, Point2f p, int radius) {
    int cx = int(std::lround(p.x)), cy = int(std::lround(p.y));
    if (cx < radius + 1 || cy < radius + 1 || cx >= img.w - radius - 1 || cy >= img.h - radius - 1)
        return false;
    // Constant-offset ROMs, initialized once, not coordinate trig per candidate.
    static const auto rom = []() {
        std::array<std::array<std::array<int, 2>, 32>, 3> table{};
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 32; ++k) {
                double a = 2 * 3.14159265358979323846 * k / 32;
                table[r][k] = {int(std::lround((4 + 2 * r) * std::cos(a))),
                               int(std::lround((4 + 2 * r) * std::sin(a)))};
            }
        return table;
    }();
    int values[32], smooth[32], lo = 1020, hi = 0;
    for (int k = 0; k < 32; ++k) {
        auto o = rom[(radius - 4) / 2][k];
        values[k] = img.get(cx + o[0], cy + o[1]);
    }
    for (int k = 0; k < 32; ++k) {
        smooth[k] = values[(k + 31) % 32] + 2 * values[k] + values[(k + 1) % 32];
        lo = std::min(lo, smooth[k]);
        hi = std::max(hi, smooth[k]);
    }
    if (hi - lo < 80)
        return false;
    std::vector<int> changes;
    int opposite = 0;
    for (int k = 0; k < 32; ++k) {
        if ((2 * smooth[k] > hi + lo) != (2 * smooth[(k + 31) % 32] > hi + lo))
            changes.push_back(k);
        opposite += std::abs(smooth[k] - smooth[(k + 16) % 32]);
    }
    if (changes.size() != 4 || 25 * opposite > 32 * (hi - lo) * 7)
        return false;
    for (int k = 0; k < 4; ++k) {
        int n = (changes[(k + 1) % 4] - changes[k] + 32) % 32;
        if (n < 3 || n > 13)
            return false;
    }
    return true;
}

ChessboardInfo detect_native(const GrayImage& gray, int rows, int cols) {
    ChessboardInfo info{};
    info.rows = rows;
    info.cols = cols;
    if (!gray.data || gray.w < 16 || gray.h < 16 || rows < 2 || cols < 2 || rows > 100 || cols > 100)
        return info;
    std::vector<Point2f> candidates;
    harris(gray, candidates);
    // Bound quadratic candidate work on unrelated, heavily textured inputs.
    if (candidates.size() > 12000 || candidates.size() < size_t(rows * cols))
        return info;
    merge_duplicates(candidates, 5.0f);
    refine_subpixel(gray, candidates, 7);
    merge_duplicates(candidates, 3.0f);
    std::vector<Point2f> inner;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (ring(gray, candidates[i], 6) && (ring(gray, candidates[i], 4) || ring(gray, candidates[i], 8)))
            inner.push_back(candidates[i]);
    }
    if (inner.size() < size_t(rows * cols))
        return info;
    if (!organize_grid(inner, rows, cols, info.corners))
        return info;
    refine_grid(gray, info);
    return info;
}

} // namespace chessboard
