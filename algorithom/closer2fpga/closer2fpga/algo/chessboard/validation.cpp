#include "internal.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
namespace chessboard {
// Squared geometry gates and rational spacing score avoid sqrt/log here.
float grid_cost(const std::vector<Point2f>& g, int rows, int cols) {
    double cost = 0, sign = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            Point2f p = g[r * cols + c];
            for (int axis = 0; axis < 2; ++axis) {
                int step = axis ? cols : 1, pos = axis ? r : c, count = axis ? rows : cols;
                if (pos + 2 >= count)
                    continue;
                auto a = g[r * cols + c + step], b = g[r * cols + c + 2 * step];
                double x = a.x - p.x, y = a.y - p.y, u = b.x - a.x, v = b.y - a.y;
                double d1 = x * x + y * y, d2 = u * u + v * v, dot = x * u + y * v, prod = d1 * d2;
                if (d1 < 16 || d2 < 16 || d2 < .3025 * d1 || d2 > 3.24 * d1 || dot < 0 ||
                    dot * dot < .81 * prod)
                    return 1e30f;
                // No sqrt/log. Approximate smooth-spacing ranking, same squared gates.
                cost += 1 - dot * dot / prod + 4 * (d2 - d1) * (d2 - d1) / ((d1 + d2) * (d1 + d2));
            }
            if (r + 1 < rows && c + 1 < cols) {
                Point2f q[4] = {p, g[r * cols + c + 1], g[(r + 1) * cols + c + 1], g[(r + 1) * cols + c]};
                for (int k = 0; k < 4; ++k) {
                    auto a = q[k], b = q[(k + 1) % 4], d = q[(k + 2) % 4];
                    double x = b.x - a.x, y = b.y - a.y, u = d.x - b.x, v = d.y - b.y;
                    double cross = x * v - y * u, prod = (x * x + y * y) * (u * u + v * v);
                    if (prod < 256 || cross * cross < .04 * prod)
                        return 1e30f;
                    if (sign == 0)
                        sign = cross;
                    if (cross * sign <= 0)
                        return 1e30f;
                }
            }
        }
    return float(cost);
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
