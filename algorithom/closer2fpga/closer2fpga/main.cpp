#include <cstdio>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "desktop/display.h"
#include "common/image.h"
#include "algo/grayscale.h"
#include "algo/chessboard.h"
#include "algo/calibrate.h"
#include "algo/undistort.h"

using namespace desktop;

int main() {
    const char* paths[] = {
        "E:/fpga/algorithom/test0.jpg",
        "E:/fpga/algorithom/test1.jpg",
        "E:/fpga/algorithom/test2.jpg",
    };
    constexpr int rows = 5, cols = 8;
    int failures = 0;
    bool correction_applied = false;
    cv::Mat views[3];
    cv::Mat colors[3];
    GrayImage sources[3];
    std::vector<std::vector<Point2f>> image_points(3);
    int width = 0, height = 0;
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
        if (width == 0) {
            width = color.cols;
            height = color.rows;
        }
        if (color.cols != width || color.rows != height) {
            std::printf("All calibration images must have the same resolution.\n");
            ++failures;
        }
        sources[i] = GrayImage(color.cols, color.rows, 3);
        for (int y = 0; y < gray.h; ++y) {
            for (int x = 0; x < gray.w; ++x) {
                auto bgr = color.at<cv::Vec3b>(y, x);

                for (int c = 0; c < 3; ++c)
                    sources[i].data[(size_t(y) * gray.w + x) * 3 + c] = bgr[c];
            }
        }
        convert_grayscale(image_view(static_cast<const GrayImage&>(sources[i])), image_view(gray));
        ChessboardInfo board = detect_chessboard(gray, rows, cols);
        std::printf("test%d.jpg: valid=%s corners=%zu\n", i, board.valid ? "YES" : "NO",
                    board.corners.size());
        if (!board.valid)
            ++failures;
        image_points[i] = board.corners;
        colors[i] = color;
        views[i] = draw_corners(color, board);
    }
    if (failures == 0) {
        // The square's physical length is unnecessary for intrinsics/distortion.
        // With size=1, reported translations are in board-square units.
        auto calibration = calibrate_camera(image_points, width, height, rows, cols, 1.0);
        const auto& k = calibration.camera;
        std::printf("\nCamera calibration (zero skew, k3 fixed to zero for three views):\n");
        std::printf("fx=%.8f fy=%.8f cx=%.8f cy=%.8f\n", k.fx, k.fy, k.cx, k.cy);
        std::printf("k1=%.9f k2=%.9f p1=%.9f p2=%.9f k3=%.9f\n", k.k1, k.k2, k.p1, k.p2, k.k3);
        std::printf("RMS=%.6f px, maximum error=%.6f px, converged=%s\n", calibration.rms,
                    calibration.max_error, calibration.converged ? "YES" : "NO");
        for (size_t i = 0; i < calibration.per_view_rms.size(); ++i)
            std::printf("test%zu.jpg reprojection RMS=%.6f px\n", i, calibration.per_view_rms[i]);
        std::printf("%s\n", calibration.message.c_str());
        if (k.valid) {
            // Same camera and resolution: build once, then reuse for all frames.
            auto table = build_remap_table(width, height, k);
            for (int i = 0; i < 3; ++i) {
                GrayImage corrected;
                remap_bilinear(sources[i], table, corrected, RemapBorder::ConstantBlack);
                views[i] = comparison(colors[i], corrected);
            }
            correction_applied = true;
        } else
            ++failures;
    } else
        std::printf("Calibration skipped: all three images must contain the complete board at the same "
                    "resolution.\n");
    // Show all results before entering the shared window event loop.
    for (int i = 0; i < 3; ++i) {
        if (views[i].empty())
            continue;
        cv::namedWindow(windows[i], cv::WINDOW_NORMAL);
        cv::imshow(windows[i], views[i]);
        // Three simultaneous windows; each shows before/after side by side.
        cv::resizeWindow(windows[i], correction_applied ? 960 : 640, correction_applied ? 270 : 360);
        cv::moveWindow(windows[i], 20 + (i % 2) * 80, 20 + i * 300);
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
