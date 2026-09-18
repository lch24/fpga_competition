#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/types.h"
#include "common/image.h"

#include "algo/chessboard.h"
#include "algo/subpixel.h"


static GrayImage cv_to_gray(const cv::Mat& mat)
{
    GrayImage img(mat.cols, mat.rows);

    for (int y = 0; y < mat.rows; ++y) {
        for (int x = 0; x < mat.cols; ++x) {
            img.set(x, y, mat.at<unsigned char>(y, x));
        }
    }

    return img;
}


static void draw_point(cv::Mat& img, const Point2f& p,
                       cv::Scalar color, int radius)
{
    cv::circle(img,
        cv::Point((int)p.x, (int)p.y),
        radius, color, -1);
}


int main()
{
    const char* paths[] = {
        "E:/fpga/algorithom/test0.jpg",
        "E:/fpga/algorithom/test1.jpg",
        "E:/fpga/algorithom/test2.jpg",
    };

    const int inner_rows = 5;
    const int inner_cols = 8;

    printf("Internal grid: %d x %d\n\n", inner_rows, inner_cols);

    cv::namedWindow("Detections", cv::WINDOW_NORMAL);

    for (int i = 0; i < 3; ++i) {
        printf("Loading: %s\n", paths[i]);

        cv::Mat color = cv::imread(paths[i]);
        if (color.empty()) {
            printf("  ERROR: cannot open image\n\n");
            continue;
        }

        cv::Mat gray_cv;
        cv::cvtColor(color, gray_cv, cv::COLOR_BGR2GRAY);

        GrayImage gray = cv_to_gray(gray_cv);

        ChessboardInfo board =
            detect_chessboard(gray, inner_rows, inner_cols);

        printf("  all_candidates: %zu\n", board.all_candidates.size());
        printf("  valid: %s, corners: %zu\n",
            board.valid ? "YES" : "NO",
            board.corners.size());

        cv::Mat view = color.clone();

        for (const auto& p : board.all_candidates)
            draw_point(view, p, cv::Scalar(0, 0, 255), 5);

        if (board.valid) {
            for (auto& p : board.corners)
                draw_point(view, p, cv::Scalar(255, 0, 0), 6);

            for (int r = 0; r < inner_rows; ++r) {
                for (int c = 0; c < inner_cols - 1; ++c) {
                    Point2f a = board.corners[r * inner_cols + c];
                    Point2f b = board.corners[r * inner_cols + c + 1];
                    cv::line(view,
                        cv::Point((int)a.x, (int)a.y),
                        cv::Point((int)b.x, (int)b.y),
                        cv::Scalar(255, 100, 0), 1);
                }
            }

            for (int c = 0; c < inner_cols; ++c) {
                for (int r = 0; r < inner_rows - 1; ++r) {
                    Point2f a = board.corners[r * inner_cols + c];
                    Point2f b = board.corners[(r + 1) * inner_cols + c];
                    cv::line(view,
                        cv::Point((int)a.x, (int)a.y),
                        cv::Point((int)b.x, (int)b.y),
                        cv::Scalar(255, 100, 0), 1);
                }
            }
        }

        char title[256];
        if (board.valid)
            snprintf(title, sizeof(title),
                "test%d.jpg  RED=all_candidates  BLUE=inner_corners", i);
        else
            snprintf(title, sizeof(title),
                "test%d.jpg  RED=all_candidates  FAILED (press any key)", i);

        cv::setWindowTitle("Detections", title);
        cv::imshow("Detections", view);

        printf("  Press key to continue (q to quit)...\n\n");
        int key = cv::waitKey(0);
        if (key == 'q' || key == 'Q')
            break;
    }

    cv::destroyAllWindows();

    return 0;
}