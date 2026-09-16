#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "common/types.h"
#include "common/image.h"
#include "algo/threshold.h"
#include "algo/chessboard.h"
#include "algo/shi_tomasi.h"
#include "algo/subpixel.h"
#include "algo/undistort.h"
#include "algo/calibrate.h"

static GrayImage make_checkerboard(
    int cell,
    int square_rows,
    int square_cols,
    int pad = 10)
{
    int h = cell * square_rows + pad * 2;
    int w = cell * square_cols + pad * 2;

    GrayImage img(w, h);

    for (int i = 0; i < w * h; ++i)
        img.data[i] = 0;

    for (int y = pad; y < pad + cell * square_rows; ++y)
    {
        int ry = (y - pad) / cell;

        for (int x = pad; x < pad + cell * square_cols; ++x)
        {
            int rx = (x - pad) / cell;

            img.set(
                x,
                y,
                ((rx + ry) % 2 == 0) ? 255 : 0
            );
        }
    }

    return img;
}

static void print_points(
    const char* tag,
    const std::vector<Point2f>& pts)
{
    if (pts.empty())
    {
        printf("  [%s] EMPTY\n", tag);
        return;
    }

    f32 x_min = 1e9f;
    f32 x_max = -1e9f;
    f32 y_min = 1e9f;
    f32 y_max = -1e9f;

    f32 sx = 0.0f;
    f32 sy = 0.0f;

    for (const auto& p : pts)
    {
        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);
        y_min = std::min(y_min, p.y);
        y_max = std::max(y_max, p.y);

        sx += p.x;
        sy += p.y;
    }

    printf(
        "  [%s] n=%zu  x=[%.0f..%.0f] mean_x=%.1f  "
        "y=[%.0f..%.0f] mean_y=%.1f\n",
        tag,
        pts.size(),
        x_min,
        x_max,
        sx / (f32)pts.size(),
        y_min,
        y_max,
        sy / (f32)pts.size()
    );
}

static std::vector<Point2f> make_inner_ground_truth(
    int pad,
    int cell,
    int square_rows,
    int square_cols)
{
    std::vector<Point2f> pts;

    const int inner_rows = square_rows - 1;
    const int inner_cols = square_cols - 1;

    pts.reserve(inner_rows * inner_cols);

    // Only INTERNAL corners:
    // r = 1 ... square_rows - 1
    // c = 1 ... square_cols - 1
    for (int r = 1; r < square_rows; ++r)
    {
        for (int c = 1; c < square_cols; ++c)
        {
            pts.push_back({
                (f32)(pad + c * cell),
                (f32)(pad + r * cell)
            });
        }
    }

    return pts;
}

static std::vector<Point2f> distort_points(
    const std::vector<Point2f>& ideal_pts,
    const CameraParams& cam)
{
    std::vector<Point2f> result;
    result.reserve(ideal_pts.size());

    for (const auto& ip : ideal_pts)
    {
        f32 nx = (ip.x - cam.cx) / cam.fx;
        f32 ny = (ip.y - cam.cy) / cam.fy;

        f32 xd, yd;

        forward_distort_norm(
            nx,
            ny,
            cam,
            xd,
            yd
        );

        result.push_back({
            cam.fx * xd + cam.cx,
            cam.fy * yd + cam.cy
        });
    }

    return result;
}

static void print_accuracy(
    const char* tag,
    const std::vector<Point2f>& detected,
    const std::vector<Point2f>& truth)
{
    if (detected.size() != truth.size() || detected.empty())
    {
        printf(
            "  [%s] cannot calculate accuracy: "
            "detected=%zu truth=%zu\n",
            tag,
            detected.size(),
            truth.size()
        );
        return;
    }

    f32 mean_err = 0.0f;
    f32 max_err = 0.0f;

    for (size_t i = 0; i < detected.size(); ++i)
    {
        const f32 dx = detected[i].x - truth[i].x;
        const f32 dy = detected[i].y - truth[i].y;

        const f32 e = std::sqrt(dx * dx + dy * dy);

        mean_err += e;
        max_err = std::max(max_err, e);
    }

    mean_err /= (f32)detected.size();

    printf(
        "  [%s] mean=%.3f px  max=%.3f px\n",
        tag,
        mean_err,
        max_err
    );
}

int main()
{
    // ------------------------------------------------------------
    // Board definition
    //
    // This test board has 6 x 8 SQUARES.
    // Therefore:
    //   complete vertices = 7 x 9
    //   internal corners  = 5 x 7 = 35
    // ------------------------------------------------------------
    const int pad = 60;
    const int cell = 50;

    const int square_rows = 6;
    const int square_cols = 8;

    const int inner_rows = square_rows - 1;
    const int inner_cols = square_cols - 1;

    const int inner_need = inner_rows * inner_cols;

    printf("=== Diagnostics ===\n\n");

    printf(
        "Board: %d x %d squares, "
        "%d x %d internal corners (%d points)\n\n",
        square_rows,
        square_cols,
        inner_rows,
        inner_cols,
        inner_need
    );

    GrayImage ideal_img = make_checkerboard(
        cell,
        square_rows,
        square_cols,
        pad
    );

    printf(
        "Ideal image: %dx%d\n",
        ideal_img.w,
        ideal_img.h
    );

    // ------------------------------------------------------------
    // Synthetic camera distortion
    // ------------------------------------------------------------
    CameraParams cam = {};

    cam.fx = ideal_img.w * 0.85f;
    cam.fy = ideal_img.h * 0.85f;
    cam.cx = ideal_img.w * 0.5f;
    cam.cy = ideal_img.h * 0.5f;

    cam.k1 = -0.3f;
    cam.k2 =  0.15f;
    cam.p1 =  0.02f;
    cam.p2 = -0.01f;
    cam.k3 =  0.0f;

    cam.valid = true;

    GrayImage dist_img;

    make_distorted_image(
        ideal_img,
        cam,
        dist_img
    );

    printf(
        "Distorted: %dx%d\n\n",
        dist_img.w,
        dist_img.h
    );

    // ============================================================
    // Ground truth: INTERNAL corners only
    // ============================================================
    std::vector<Point2f> ideal_truth =
        make_inner_ground_truth(
            pad,
            cell,
            square_rows,
            square_cols
        );

    std::vector<Point2f> distorted_truth =
        distort_points(
            ideal_truth,
            cam
        );

    printf(
        "Ground truth: ideal=%zu, distorted=%zu\n\n",
        ideal_truth.size(),
        distorted_truth.size()
    );

    // ============================================================
    // Step 1: IDEAL image
    // ============================================================
    printf(
        "--- Step 1: Detect INTERNAL corners on IDEAL ---\n"
    );

    {
        std::vector<Point2f> raw;

        shi_tomasi_detect(
            ideal_img,
            raw,
            0.15f,
            3
        );

        print_points(
            "ideal shi_tomasi raw",
            raw
        );

        ChessboardInfo info =
            detect_chessboard(
                ideal_img,
                inner_rows,
                inner_cols
            );

        if (!info.valid)
        {
            printf(
                "  ideal detect FAILED\n\n"
            );
        }
        else
        {
            print_points(
                "ideal internal detected",
                info.corners
            );

            // NOTE:
            // This is your current subpixel implementation.
            // We leave it unchanged for now.
            refine_subpixel(
                ideal_img,
                info.corners,
                7
            );

            print_points(
                "ideal internal + subpixel",
                info.corners
            );

            print_accuracy(
                "IDEAL accuracy",
                info.corners,
                ideal_truth
            );

            printf("\n");
        }
    }

    // ============================================================
    // Step 2: DISTORTED image
    // ============================================================
    printf(
        "--- Step 2: Detect INTERNAL corners on DISTORTED ---\n"
    );

    {
        std::vector<Point2f> raw;

        shi_tomasi_detect(
            dist_img,
            raw,
            0.15f,
            3
        );

        print_points(
            "dist shi_tomasi raw",
            raw
        );

        ChessboardInfo info =
            detect_chessboard(
                dist_img,
                inner_rows,
                inner_cols
            );

        if (!info.valid)
        {
            printf(
                "  FAIL: detect_chessboard valid=false\n"
            );

            cv::Mat m(
                dist_img.h,
                dist_img.w,
                CV_8UC1,
                dist_img.data
            );

            cv::imshow(
                "Distorted Gray",
                m
            );

            cv::waitKey(0);
            return 1;
        }

        print_points(
            "dist internal detected",
            info.corners
        );

        refine_subpixel(
            dist_img,
            info.corners,
            7
        );

        print_points(
            "dist internal + subpixel",
            info.corners
        );

        print_accuracy(
            "DISTORTED accuracy",
            info.corners,
            distorted_truth
        );

        // --------------------------------------------------------
        // Visualization
        //
        // Red   = detected INTERNAL corners
        // Green = INTERNAL ground truth
        //
        // The outer boundary corners are intentionally not drawn.
        // --------------------------------------------------------
        cv::Mat dist_mat(
            dist_img.h,
            dist_img.w,
            CV_8UC1,
            dist_img.data
        );

        cv::Mat disp;

        cv::cvtColor(
            dist_mat,
            disp,
            cv::COLOR_GRAY2BGR
        );

        for (const auto& c : info.corners)
        {
            cv::circle(
                disp,
                cv::Point(
                    (int)std::lround(c.x),
                    (int)std::lround(c.y)
                ),
                5,
                cv::Scalar(0, 0, 255),
                -1
            );
        }

        for (const auto& t : distorted_truth)
        {
            cv::circle(
                disp,
                cv::Point(
                    (int)std::lround(t.x),
                    (int)std::lround(t.y)
                ),
                4,
                cv::Scalar(0, 255, 0),
                1
            );
        }

        const int scale = 2;

        cv::Mat big;

        cv::resize(
            disp,
            big,
            cv::Size(
                disp.cols * scale,
                disp.rows * scale
            ),
            0,
            0,
            cv::INTER_NEAREST
        );

        cv::putText(
            big,
            "Red=detected INTERNAL  Green=ground truth INTERNAL",
            cv::Point(
                10 * scale,
                30 * scale
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6 * scale,
            cv::Scalar(0, 255, 255),
            2 * scale
        );

        cv::imshow(
            "Distorted + Internal Corners",
            big
        );

        printf(
            "\nPress any key...\n"
        );

        cv::waitKey(0);
    }

    return 0;
}
