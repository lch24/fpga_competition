#pragma once
#include "../common/types.h"
#include "../common/image.h"
#include <vector>

struct ChessboardInfo {
    int rows;
    int cols;
    std::vector<Point2f> corners;
    bool valid;
};

ChessboardInfo detect_chessboard(const GrayImage& gray, int rows, int cols);