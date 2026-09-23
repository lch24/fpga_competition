#include "internal.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
namespace chessboard {
float median(std::vector<float> v) {
    if (v.empty())
        return 0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
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
            if (end - begin < cols)
                break;
            std::sort(order.begin() + begin, order.begin() + end, [&](int i, int j) { return u(i) < u(j); });
            // Select a contiguous, regularly spaced row; extra edge candidates
            // may be discarded, but missing interior corners are never invented.
            float row_best = 1e30f;
            int start_best = -1;
            for (int start = begin; start + cols <= end; ++start) {
                std::vector<float> steps;
                for (int c = 1; c < cols; ++c)
                    steps.push_back(distance(points[order[start + c - 1]], points[order[start + c]]));
                float spacing = median(steps), score = 0;
                if (spacing < 4)
                    continue;
                for (float step : steps) {
                    float change = (step - spacing) / spacing;
                    score += change * change;
                }
                if (score < row_best) {
                    row_best = score;
                    start_best = start;
                }
            }
            if (start_best < 0)
                break;
            for (int c = 0; c < cols; ++c)
                grid.push_back(points[order[start_best + c]]);
            begin = end;
        }
        if ((int)grid.size() != rows * cols)
            continue;
        float cost = grid_cost(grid, rows, cols);
        if (cost < best_cost) {
            best_cost = cost;
            best = std::move(grid);
        }
    }
    if (best.empty())
        return false;
    // Deterministic image-relative origin among the four equivalent board corners.
    // An unmarked chessboard cannot encode a unique physical origin.
    int corner_ids[4] = {0, cols - 1, (rows - 1) * cols, rows * cols - 1};
    int origin = 0;
    for (int k = 1; k < 4; ++k)
        if (best[corner_ids[k]].x + best[corner_ids[k]].y <
            best[corner_ids[origin]].x + best[corner_ids[origin]].y)
            origin = k;
    auto copy = best;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            best[r * cols + c] =
                copy[(origin >= 2 ? rows - 1 - r : r) * cols + (origin % 2 ? cols - 1 - c : c)];
    return true;
}

} // namespace chessboard
