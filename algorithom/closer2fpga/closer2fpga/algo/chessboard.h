#pragma once
#include "../common/types.h"
#include "../common/image.h"
#include <vector>

struct ChessboardInfo {
    // rows / cols are the number of INTERNAL chessboard corners.
    int rows;
    int cols;
    std::vector<Point2f> corners; // row-major order, internal corners only
    bool valid;
};

// Detect a chessboard with inner_rows x inner_cols INTERNAL corners.
// For example, a 6x8-square board has 5x7 internal corners.
ChessboardInfo detect_chessboard(
    const GrayImage& gray,
    int inner_rows,
    int inner_cols
);
