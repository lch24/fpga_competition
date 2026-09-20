#pragma once
#include "../common/types.h"
#include "../common/image.h"
#include <vector>

struct ChessboardInfo {
    int rows;
    int cols;
    // Refined original-image coordinates, row-major. Empty on failure.
    // Origin is image-relative; an unmarked board has flip ambiguity.
    std::vector<Point2f> corners;
    std::vector<Point2f> all_candidates;
    bool valid;
};

ChessboardInfo detect_chessboard(
    const GrayImage& gray,
    int inner_rows,
    int inner_cols
);
