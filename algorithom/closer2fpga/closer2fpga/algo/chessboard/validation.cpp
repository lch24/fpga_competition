#include "internal.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
namespace chessboard {
// Validate both lattice directions, convex cells, and smoothly varying spacing.
// All tolerances are relative to observed spacing, not fixed pixel distances.
float grid_cost(const std::vector<Point2f>& g, int rows, int cols) {
    float cost = 0, sign = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            Point2f p = g[r * cols + c];
            for (int axis = 0; axis < 2; ++axis) {
                int step = axis ? cols : 1;
                int pos = axis ? r : c, count = axis ? rows : cols;
                if (pos + 2 >= count)
                    continue;
                Point2f a = g[r * cols + c + step], b = g[r * cols + c + 2 * step];
                float dx1 = a.x - p.x, dy1 = a.y - p.y;
                float dx2 = b.x - a.x, dy2 = b.y - a.y;
                float l1 = std::hypot(dx1, dy1), l2 = std::hypot(dx2, dy2);
                if (l1 < 4 || l2 < 4 || l2 / l1 < 0.55f || l2 / l1 > 1.8f)
                    return 1e30f;
                float cosine = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
                if (cosine < 0.90f)
                    return 1e30f;
                float change = std::log(l2 / l1);
                cost += (1 - cosine) + change * change;
            }
            if (r + 1 < rows && c + 1 < cols) {
                Point2f q[4] = {p, g[r * cols + c + 1], g[(r + 1) * cols + c + 1], g[(r + 1) * cols + c]};
                for (int k = 0; k < 4; ++k) {
                    Point2f a = q[k], b = q[(k + 1) % 4], d = q[(k + 2) % 4];
                    float cross = (b.x - a.x) * (d.y - b.y) - (b.y - a.y) * (d.x - b.x);
                    float lengths = distance(a, b) * distance(b, d);
                    if (lengths < 16 || std::fabs(cross) < lengths * 0.2f)
                        return 1e30f;
                    if (sign == 0)
                        sign = cross;
                    if (cross * sign <= 0)
                        return 1e30f;
                }
            }
        }
    return cost;
}

bool refine_grid(const GrayImage& gray, ChessboardInfo& board) {
    // Keep the window within the shortest projected cell edge at this scale.
    float min_step = std::numeric_limits<float>::max();
    for (int r = 0; r < board.rows; ++r)
        for (int c = 0; c < board.cols; ++c) {
            int i = r * board.cols + c;
            if (c + 1 < board.cols)
                min_step = std::min(min_step, distance(board.corners[i], board.corners[i + 1]));
            if (r + 1 < board.rows)
                min_step = std::min(min_step, distance(board.corners[i], board.corners[i + board.cols]));
        }
    refine_subpixel(gray, board.corners, std::clamp(int(min_step * 0.15f), 2, 10));
    board.valid = grid_cost(board.corners, board.rows, board.cols) < 1e30f;
    if (!board.valid)
        board.corners.clear();
    return board.valid;
}

} // namespace chessboard
