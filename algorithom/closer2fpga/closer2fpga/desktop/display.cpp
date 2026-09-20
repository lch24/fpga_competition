#include "display.h"
#include <cmath>
#include <opencv2/imgproc.hpp>
namespace desktop {
cv::Point fixed_point(Point2f p) {
    return {int(std::lround(p.x * 256)), int(std::lround(p.y * 256))};
}

cv::Mat draw_corners(const cv::Mat& image, const ChessboardInfo& board) {
    cv::Mat view = image.clone();
    if (!board.valid) {
        cv::putText(view, "Chessboard detection failed", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255},
                    2, cv::LINE_AA);
        return view;
    }
    for (int r = 0; r < board.rows; ++r) {
        for (int c = 0; c < board.cols; ++c) {
            int i = r * board.cols + c;
            Point2f p = board.corners[i];
            if (c + 1 < board.cols)
                cv::line(view, fixed_point(p), fixed_point(board.corners[i + 1]), {180, 100, 0}, 1,
                         cv::LINE_AA, 8);
            if (r + 1 < board.rows)
                cv::line(view, fixed_point(p), fixed_point(board.corners[i + board.cols]), {180, 100, 0}, 1,
                         cv::LINE_AA, 8);
            // Preserve subpixel coordinates when drawing the small green cross.
            cv::line(view, fixed_point({p.x - 4, p.y}), fixed_point({p.x + 4, p.y}), {0, 255, 0}, 1,
                     cv::LINE_AA, 8);
            cv::line(view, fixed_point({p.x, p.y - 4}), fixed_point({p.x, p.y + 4}), {0, 255, 0}, 1,
                     cv::LINE_AA, 8);
        }
    }
    return view;
}

// OpenCV is used here only as the display container. Remapping/interpolation
// above this boundary runs on our own interleaved Image<uint8_t> buffer.
cv::Mat comparison(const cv::Mat& original, const GrayImage& corrected) {
    cv::Mat view(original.rows, original.cols * 2, CV_8UC3);
    for (int y = 0; y < original.rows; ++y) {
        for (int x = 0; x < original.cols; ++x) {
            view.at<cv::Vec3b>(y, x) = original.at<cv::Vec3b>(y, x);
            size_t offset = (size_t(y) * corrected.w + x) * 3;
            view.at<cv::Vec3b>(y, x + original.cols) =
                cv::Vec3b(corrected.data[offset], corrected.data[offset + 1], corrected.data[offset + 2]);
        }
    }
    cv::putText(view, "Original", {20, 35}, cv::FONT_HERSHEY_SIMPLEX, .8, {0, 255, 0}, 2, cv::LINE_AA);
    cv::putText(view, "Undistorted", {original.cols + 20, 35}, cv::FONT_HERSHEY_SIMPLEX, .8, {0, 255, 0}, 2,
                cv::LINE_AA);
    return view;
}
} // namespace desktop
