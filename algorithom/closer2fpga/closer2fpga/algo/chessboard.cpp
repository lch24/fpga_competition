#include "chessboard.h"
#include "shi_tomasi.h"
#include "subpixel.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace {
constexpr float pi = 3.14159265358979323846f;
float distance(Point2f a, Point2f b) { return std::hypot(a.x - b.x, a.y - b.y); }
float median(std::vector<float> v) {
    if (v.empty()) return 0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}
void merge_duplicates(std::vector<Point2f>& points, float radius) {
    std::vector<bool> used(points.size(), false);
    std::vector<Point2f> out;
    for (size_t i = 0; i < points.size(); ++i) {
        if (used[i]) continue;
        Point2f sum = points[i];
        int n = 1;
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (!used[j] && distance(points[i], points[j]) < radius) {
                used[j] = true; sum.x += points[j].x; sum.y += points[j].y; ++n;
            }
        }
        out.push_back({sum.x / n, sum.y / n});
    }
    points = std::move(out);
}
float nearest_distance(const std::vector<Point2f>& points, size_t i) {
    float result = std::numeric_limits<float>::max();
    for (size_t j = 0; j < points.size(); ++j)
        if (i != j) result = std::min(result, distance(points[i], points[j]));
    return result;
}

// A ring around a four-cell junction has four brightness transitions.
// Unlike image-axis quadrants, this test does not assume horizontal edges.
// Smoothing and minimum sector lengths reject isolated texture/noise responses.
bool alternating_ring(const GrayImage& img, Point2f p, float radius) {
    if (p.x < radius + 1 || p.y < radius + 1 ||
        p.x >= img.w - radius - 1 || p.y >= img.h - radius - 1) return false;
    float values[32], smooth[32];
    for (int k = 0; k < 32; ++k) {
        float a = 2 * pi * k / 32;
        values[k] = float(img.get(int(std::lround(p.x + radius * std::cos(a))),
                                   int(std::lround(p.y + radius * std::sin(a)))));
    }
    float lo = 255, hi = 0;
    for (int k = 0; k < 32; ++k) {
        smooth[k] = (values[(k + 31) % 32] + 2 * values[k] + values[(k + 1) % 32]) / 4;
        lo = std::min(lo, smooth[k]); hi = std::max(hi, smooth[k]);
    }
    if (hi - lo < 20) return false;
    float threshold = (hi + lo) * 0.5f;
    std::vector<int> transitions;
    float opposite_error = 0;
    for (int k = 0; k < 32; ++k) {
        if ((smooth[k] > threshold) != (smooth[(k + 31) % 32] > threshold))
            transitions.push_back(k);
        opposite_error += std::fabs(smooth[k] - smooth[(k + 16) % 32]);
    }
    if (transitions.size() != 4 || opposite_error > 32 * (hi - lo) * 0.28f) return false;
    for (int k = 0; k < 4; ++k) {
        int length = (transitions[(k + 1) % 4] - transitions[k] + 32) % 32;
        if (length < 3 || length > 13) return false;
    }
    return true;
}

// Validate both lattice directions, convex cells, and smoothly varying spacing.
// All tolerances are relative to observed spacing, not fixed pixel distances.
float grid_cost(const std::vector<Point2f>& g, int rows, int cols) {
    float cost = 0, sign = 0;
    for (int r = 0; r < rows; ++r) for (int c = 0; c < cols; ++c) {
        Point2f p = g[r * cols + c];
        for (int axis = 0; axis < 2; ++axis) {
            int step = axis ? cols : 1;
            int pos = axis ? r : c, count = axis ? rows : cols;
            if (pos + 2 >= count) continue;
            Point2f a = g[r * cols + c + step], b = g[r * cols + c + 2 * step];
            float dx1 = a.x - p.x, dy1 = a.y - p.y;
            float dx2 = b.x - a.x, dy2 = b.y - a.y;
            float l1 = std::hypot(dx1, dy1), l2 = std::hypot(dx2, dy2);
            if (l1 < 4 || l2 < 4 || l2 / l1 < 0.55f || l2 / l1 > 1.8f) return 1e30f;
            float cosine = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
            if (cosine < 0.90f) return 1e30f;
            float change = std::log(l2 / l1);
            cost += (1 - cosine) + change * change;
        }
        if (r + 1 < rows && c + 1 < cols) {
            Point2f q[4] = {p, g[r * cols + c + 1], g[(r + 1) * cols + c + 1], g[(r + 1) * cols + c]};
            for (int k = 0; k < 4; ++k) {
                Point2f a = q[k], b = q[(k + 1) % 4], d = q[(k + 2) % 4];
                float cross = (b.x - a.x) * (d.y - b.y) - (b.y - a.y) * (d.x - b.x);
                float lengths = distance(a, b) * distance(b, d);
                if (lengths < 16 || std::fabs(cross) < lengths * 0.2f) return 1e30f;
                if (sign == 0) sign = cross;
                if (cross * sign <= 0) return 1e30f;
            }
        }
    }
    return cost;
}

bool organize_grid(const std::vector<Point2f>& points, int rows, int cols, std::vector<Point2f>& best) {
    float best_cost = 1e30f;
    // Search a board-aligned coordinate system; only project for ordering.
    // The output always retains the original measured image coordinates.
    for (int degree = -90; degree < 90; degree += 2) {
        float a = degree * pi / 180, co = std::cos(a), si = std::sin(a);
        auto u = [&](int i) { return points[i].x * co + points[i].y * si; };
        auto v = [&](int i) { return -points[i].x * si + points[i].y * co; };
        std::vector<int> order(points.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int i, int j) { return v(i) < v(j); });
        std::vector<int> gaps(order.size() - 1);
        std::iota(gaps.begin(), gaps.end(), 0);
        std::sort(gaps.begin(), gaps.end(), [&](int i, int j) {
            return v(order[i + 1]) - v(order[i]) > v(order[j + 1]) - v(order[j]);
        });
        gaps.resize(rows - 1);
        std::sort(gaps.begin(), gaps.end());
        gaps.push_back(int(order.size()) - 1);
        int begin = 0;
        std::vector<Point2f> grid;
        for (int r = 0; r < rows; ++r) {
            int end = gaps[r] + 1;
            if (end - begin < cols) break;
            std::sort(order.begin() + begin, order.begin() + end, [&](int i, int j) { return u(i) < u(j); });
            // Select a contiguous, regularly spaced row; extra edge candidates
            // may be discarded, but missing interior corners are never invented.
            float row_best = 1e30f;
            int start_best = -1;
            for (int start = begin; start + cols <= end; ++start) {
                std::vector<float> steps;
                for (int c = 1; c < cols; ++c) steps.push_back(distance(points[order[start + c - 1]], points[order[start + c]]));
                float spacing = median(steps), score = 0;
                if (spacing < 4) continue;
                for (float step : steps) {
                    float change = (step - spacing) / spacing;
                    score += change * change;
                }
                if (score < row_best) { row_best = score; start_best = start; }
            }
            if (start_best < 0) break;
            for (int c = 0; c < cols; ++c) grid.push_back(points[order[start_best + c]]);
            begin = end;
        }
        if ((int)grid.size() != rows * cols) continue;
        float cost = grid_cost(grid, rows, cols);
        if (cost < best_cost) { best_cost = cost; best = std::move(grid); }
    }
    if (best.empty()) return false;
    // Deterministic image-relative origin among the four equivalent board corners.
    // An unmarked chessboard cannot encode a unique physical origin.
    int corner_ids[4] = {0, cols - 1, (rows - 1) * cols, rows * cols - 1};
    int origin = 0;
    for (int k = 1; k < 4; ++k)
        if (best[corner_ids[k]].x + best[corner_ids[k]].y < best[corner_ids[origin]].x + best[corner_ids[origin]].y) origin = k;
    auto copy = best;
    for (int r = 0; r < rows; ++r) for (int c = 0; c < cols; ++c)
        best[r * cols + c] = copy[(origin >= 2 ? rows - 1 - r : r) * cols + (origin % 2 ? cols - 1 - c : c)];
    return true;
}

bool refine_grid(const GrayImage& gray, ChessboardInfo& board) {
    // Keep the window within the shortest projected cell edge at this scale.
    float min_step = std::numeric_limits<float>::max();
    for (int r = 0; r < board.rows; ++r) for (int c = 0; c < board.cols; ++c) {
        int i = r * board.cols + c;
        if (c + 1 < board.cols)
            min_step = std::min(min_step, distance(board.corners[i], board.corners[i + 1]));
        if (r + 1 < board.rows)
            min_step = std::min(min_step, distance(board.corners[i], board.corners[i + board.cols]));
    }
    refine_subpixel(gray, board.corners, std::clamp(int(min_step * 0.15f), 2, 10));
    board.valid = grid_cost(board.corners, board.rows, board.cols) < 1e30f;
    if (!board.valid) board.corners.clear();
    return board.valid;
}

ChessboardInfo detect_native(const GrayImage& gray, int rows, int cols) {
    ChessboardInfo info{};
    info.rows = rows; info.cols = cols;
    if (!gray.data || gray.w < 16 || gray.h < 16 || rows < 2 || cols < 2 ||
        rows > 100 || cols > 100) return info;
    std::vector<Point2f> candidates;
    shi_tomasi_detect(gray, candidates, 0.08f, 3);
    // Bound quadratic candidate work on unrelated, heavily textured inputs.
    if (candidates.size() > 12000 || candidates.size() < size_t(rows * cols)) return info;
    merge_duplicates(candidates, 5.0f);
    refine_subpixel(gray, candidates, 7);
    merge_duplicates(candidates, 3.0f);
    std::vector<Point2f> inner;
    for (size_t i = 0; i < candidates.size(); ++i) {
        float spacing = nearest_distance(candidates, i);
        float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
        if (alternating_ring(gray, candidates[i], radius) &&
            (alternating_ring(gray, candidates[i], radius * 0.75f) ||
             alternating_ring(gray, candidates[i], radius * 1.25f))) inner.push_back(candidates[i]);
    }
    if (inner.size() < size_t(rows * cols)) return info;
    if (!organize_grid(inner, rows, cols, info.corners)) return info;
    refine_grid(gray, info);
    return info;
}
}

ChessboardInfo detect_chessboard(const GrayImage& gray, int rows, int cols) {
    if (gray.data && gray.w >= 32 && gray.h >= 32 && std::max(gray.w, gray.h) > 960) {
        // Coarse-to-fine detection suppresses multiple responses around broad
        // printed edges. Only the grid search is downsampled; final localization
        // always uses the original image. Fall back to full resolution for small
        // or distant boards whose corners would disappear in the pyramid.
        GrayImage half(gray.w / 2, gray.h / 2);
        for (int y = 0; y < half.h; ++y) for (int x = 0; x < half.w; ++x)
            half.set(x, y, uint8_t((int(gray.get(2*x, 2*y)) + gray.get(2*x+1, 2*y)
                + gray.get(2*x, 2*y+1) + gray.get(2*x+1, 2*y+1) + 2) / 4));
        auto coarse = detect_chessboard(half, rows, cols);
        if (coarse.valid) {
            for (auto& p : coarse.corners) { p.x = 2*p.x + 0.5f; p.y = 2*p.y + 0.5f; }
            if (refine_grid(gray, coarse)) return coarse;
        }
    }
    return detect_native(gray, rows, cols);
}
