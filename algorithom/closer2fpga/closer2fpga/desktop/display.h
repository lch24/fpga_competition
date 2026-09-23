#pragma once
#include "../algo/chessboard.h"
#include <opencv2/core.hpp>
namespace desktop {
cv::Mat draw_corners(const cv::Mat& image, const ChessboardInfo& board);
cv::Mat comparison(const cv::Mat& original, const GrayImage& corrected);
} // namespace desktop
