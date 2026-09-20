#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "common/image.h"
#include "algo/chessboard.h"

namespace {
cv::Point fixed_point(Point2f p) {
    return {int(std::lround(p.x * 256)), int(std::lround(p.y * 256))};
}

cv::Mat draw_corners(const cv::Mat& image, const ChessboardInfo& board) {
    cv::Mat view = image.clone();
    if (!board.valid) {
        cv::putText(view, "Chessboard detection failed", {20, 40},
            cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2, cv::LINE_AA);
        return view;
    }
    for (int r = 0; r < board.rows; ++r) {
        for (int c = 0; c < board.cols; ++c) {
            int i = r * board.cols + c;
            Point2f p = board.corners[i];
            if (c + 1 < board.cols)
                cv::line(view, fixed_point(p), fixed_point(board.corners[i + 1]),
                    {180, 100, 0}, 1, cv::LINE_AA, 8);
            if (r + 1 < board.rows)
                cv::line(view, fixed_point(p), fixed_point(board.corners[i + board.cols]),
                    {180, 100, 0}, 1, cv::LINE_AA, 8);
            // Preserve subpixel coordinates when drawing the small green cross.
            cv::line(view, fixed_point({p.x - 4, p.y}), fixed_point({p.x + 4, p.y}),
                {0, 255, 0}, 1, cv::LINE_AA, 8);
            cv::line(view, fixed_point({p.x, p.y - 4}), fixed_point({p.x, p.y + 4}),
                {0, 255, 0}, 1, cv::LINE_AA, 8);
        }
    }
    return view;
}
}

int main() {
    const char* paths[] = {
        "E:/fpga/algorithom/test0.jpg",
        "E:/fpga/algorithom/test1.jpg",
        "E:/fpga/algorithom/test2.jpg",
    };
    constexpr int rows = 5, cols = 8;
    int failures = 0;
    cv::Mat views[3];
    const char* windows[] = {"test0.jpg", "test1.jpg", "test2.jpg"};
    bool open[3] = {};
    for (int i = 0; i < 3; ++i) {
        cv::Mat color = cv::imread(paths[i]);
        if (color.empty()) {
            std::printf("Cannot load: %s\n", paths[i]);
            ++failures;
            continue;
        }
        GrayImage gray(color.cols, color.rows);
        for (int y = 0; y < gray.h; ++y) {
            for (int x = 0; x < gray.w; ++x) {
                auto bgr = color.at<cv::Vec3b>(y, x);
                gray.set(x, y, uint8_t((299 * bgr[2] + 587 * bgr[1] + 114 * bgr[0] + 500) / 1000));
            }
        }
        ChessboardInfo board = detect_chessboard(gray, rows, cols);
        std::printf("test%d.jpg: valid=%s corners=%zu\n", i,
            board.valid ? "YES" : "NO", board.corners.size());
        if (!board.valid) ++failures;
        views[i] = draw_corners(color, board);
    }
    // Show all results before entering the shared window event loop.
    for (int i = 0; i < 3; ++i) {
        if (views[i].empty()) continue;
        cv::namedWindow(windows[i], cv::WINDOW_NORMAL);
        cv::imshow(windows[i], views[i]);
        cv::resizeWindow(windows[i], 640, 360);
        cv::moveWindow(windows[i], 20 + (i % 2) * 660, 20 + (i / 2) * 420);
        open[i] = true;
    }
    while (open[0] || open[1] || open[2]) {
        cv::waitKey(30);
        for (int i = 0; i < 3; ++i)
            if (open[i] && cv::getWindowProperty(windows[i], cv::WND_PROP_VISIBLE) < 1)
                open[i] = false;
    }
    cv::destroyAllWindows();
    return failures ? 1 : 0;
}
