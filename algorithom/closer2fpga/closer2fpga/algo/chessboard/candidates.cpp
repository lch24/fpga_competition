#include "internal.h"
#include <algorithm>
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
float nearest_distance(const std::vector<Point2f>& points, size_t i) {
    float result = std::numeric_limits<float>::max();
    for (size_t j = 0; j < points.size(); ++j)
        if (i != j)
            result = std::min(result, distance(points[i], points[j]));
    return result;
}

// A ring around a four-cell junction has four brightness transitions.
// Unlike image-axis quadrants, this test does not assume horizontal edges.
// Smoothing and minimum sector lengths reject isolated texture/noise responses.
bool alternating_ring(const GrayImage& img, Point2f p, float radius) {
    if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 1 || p.y >= img.h - radius - 1)
        return false;
    float values[32], smooth[32];
    for (int k = 0; k < 32; ++k) {
        float a = 2 * pi * k / 32;
        values[k] = float(img.get(int(std::lround(p.x + radius * std::cos(a))),
                                  int(std::lround(p.y + radius * std::sin(a)))));
    }
    float lo = 255, hi = 0;
    for (int k = 0; k < 32; ++k) {
        smooth[k] = (values[(k + 31) % 32] + 2 * values[k] + values[(k + 1) % 32]) / 4;
        lo = std::min(lo, smooth[k]);
        hi = std::max(hi, smooth[k]);
    }
    if (hi - lo < 20)
        return false;
    float threshold = (hi + lo) * 0.5f;
    std::vector<int> transitions;
    float opposite_error = 0;
    for (int k = 0; k < 32; ++k) {
        if ((smooth[k] > threshold) != (smooth[(k + 31) % 32] > threshold))
            transitions.push_back(k);
        opposite_error += std::fabs(smooth[k] - smooth[(k + 16) % 32]);
    }
    if (transitions.size() != 4 || opposite_error > 32 * (hi - lo) * 0.28f)
        return false;
    for (int k = 0; k < 4; ++k) {
        int length = (transitions[(k + 1) % 4] - transitions[k] + 32) % 32;
        if (length < 3 || length > 13)
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
    shi_tomasi_detect(gray, candidates, 0.08f, 3);
    // Bound quadratic candidate work on unrelated, heavily textured inputs.
    if (candidates.size() > 12000 || candidates.size() < size_t(rows * cols))
        return info;
    merge_duplicates(candidates, 5.0f);
    refine_subpixel(gray, candidates, 7);
    merge_duplicates(candidates, 3.0f);
    std::vector<Point2f> inner;
    for (size_t i = 0; i < candidates.size(); ++i) {
        float spacing = nearest_distance(candidates, i);
        float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
        if (alternating_ring(gray, candidates[i], radius) &&
            (alternating_ring(gray, candidates[i], radius * 0.75f) ||
             alternating_ring(gray, candidates[i], radius * 1.25f)))
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
