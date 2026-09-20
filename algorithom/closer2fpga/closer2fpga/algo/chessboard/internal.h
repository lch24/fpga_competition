#pragma once
#include "../chessboard.h"
#include "../shi_tomasi.h"
#include "../subpixel.h"
#include <cmath>

namespace chessboard {
constexpr float pi = 3.14159265358979323846f;
inline float distance(Point2f a, Point2f b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
float grid_cost(const std::vector<Point2f>& grid, int rows, int cols);
bool organize_grid(const std::vector<Point2f>& points, int rows, int cols, std::vector<Point2f>& best);
bool refine_grid(const GrayImage& gray, ChessboardInfo& board);
ChessboardInfo detect_native(const GrayImage& gray, int rows, int cols);
} // namespace chessboard
