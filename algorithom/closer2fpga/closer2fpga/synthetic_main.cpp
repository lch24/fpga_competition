#define _CRT_SECURE_NO_WARNINGS

#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "common/types.h"
#include "common/image.h"

#include "algo/chessboard.h"
#include "algo/shi_tomasi.h"
#include "algo/subpixel.h"
#include "algo/undistort.h"
#include "algo/calibrate.h"


// ============================================================
// Synthetic chessboard
// ============================================================

static GrayImage make_ideal_chessboard(
    int width,
    int height,
    int squares_y,
    int squares_x,
    int cell,
    int offset_x,
    int offset_y)
{
    GrayImage img(width, height);

    // background
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            img.set(x, y, 127);
        }
    }

    for (int sy = 0; sy < squares_y; ++sy) {
        for (int sx = 0; sx < squares_x; ++sx) {

            int x0 = offset_x + sx * cell;
            int y0 = offset_y + sy * cell;

            unsigned char value =
                ((sx + sy) & 1) ? 255 : 0;

            for (int y = y0; y < y0 + cell; ++y) {
                if (y < 0 || y >= height)
                    continue;

                for (int x = x0; x < x0 + cell; ++x) {
                    if (x < 0 || x >= width)
                        continue;

                    img.set(x, y, value);
                }
            }
        }
    }

    return img;
}


// ============================================================
// Ideal internal corner coordinates
// ============================================================

static std::vector<Point2f> make_ideal_corners(
    int inner_rows,
    int inner_cols,
    int cell,
    int offset_x,
    int offset_y)
{
    std::vector<Point2f> pts;

    pts.reserve(inner_rows * inner_cols);

    for (int r = 0; r < inner_rows; ++r) {
        for (int c = 0; c < inner_cols; ++c) {

            Point2f p;

            p.x = (f32)(offset_x + (c + 1) * cell);
            p.y = (f32)(offset_y + (r + 1) * cell);

            pts.push_back(p);
        }
    }

    return pts;
}


// ============================================================
// Bilinear sampling
// ============================================================

static f32 bilinear_sample(
    const GrayImage& img,
    f32 x,
    f32 y)
{
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;

    if (x > (f32)(img.w - 1))
        x = (f32)(img.w - 1);

    if (y > (f32)(img.h - 1))
        y = (f32)(img.h - 1);

    int x0 = (int)std::floor(x);
    int y0 = (int)std::floor(y);

    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x1 >= img.w) x1 = img.w - 1;
    if (y1 >= img.h) y1 = img.h - 1;

    f32 dx = x - (f32)x0;
    f32 dy = y - (f32)y0;

    f32 p00 = (f32)img.get(x0, y0);
    f32 p10 = (f32)img.get(x1, y0);
    f32 p01 = (f32)img.get(x0, y1);
    f32 p11 = (f32)img.get(x1, y1);

    f32 a = p00 * (1.0f - dx) + p10 * dx;
    f32 b = p01 * (1.0f - dx) + p11 * dx;

    return a * (1.0f - dy) + b * dy;
}


// ============================================================
// Generate distorted image
// ============================================================

static GrayImage make_distorted_image(
    const GrayImage& ideal,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& d)
{
    GrayImage out(ideal.w, ideal.h);

    for (int y = 0; y < ideal.h; ++y) {
        for (int x = 0; x < ideal.w; ++x) {

            f64 xd =
                ((f64)x - cx) / fx;

            f64 yd =
                ((f64)y - cy) / fy;

            f64 r2 = xd * xd + yd * yd;

            f64 radial =
                1.0
                + d.k1 * r2
                + d.k2 * r2 * r2
                + d.k3 * r2 * r2 * r2;

            f64 xt =
                2.0 * d.p1 * xd * yd
                + d.p2 * (r2 + 2.0 * xd * xd);

            f64 yt =
                d.p1 * (r2 + 2.0 * yd * yd)
                + 2.0 * d.p2 * xd * yd;

            f64 xu = xd * radial + xt;
            f64 yu = yd * radial + yt;

            f32 sx =
                (f32)(xu * fx + cx);

            f32 sy =
                (f32)(yu * fy + cy);

            f32 v =
                bilinear_sample(ideal, sx, sy);

            int iv = (int)(v + 0.5f);

            if (iv < 0) iv = 0;
            if (iv > 255) iv = 255;

            out.set(x, y, (unsigned char)iv);
        }
    }

    return out;
}


// ============================================================
// GrayImage -> OpenCV image
// ============================================================

static cv::Mat to_cv_mat(const GrayImage& img)
{
    cv::Mat mat(img.h, img.w, CV_8UC1);

    for (int y = 0; y < img.h; ++y) {
        for (int x = 0; x < img.w; ++x) {
            mat.at<unsigned char>(y, x) =
                img.get(x, y);
        }
    }

    return mat;
}


// ============================================================
// Image error
// ============================================================

static void image_error(
    const GrayImage& a,
    const GrayImage& b,
    f64& mean_abs,
    int& max_err)
{
    mean_abs = 0.0;
    max_err = 0;

    if (a.w != b.w || a.h != b.h)
        return;

    const int n = a.w * a.h;

    f64 sum = 0.0;

    for (int y = 0; y < a.h; ++y) {
        for (int x = 0; x < a.w; ++x) {

            int av = (int)a.get(x, y);
            int bv = (int)b.get(x, y);

            int e = std::abs(av - bv);

            sum += (f64)e;

            if (e > max_err)
                max_err = e;
        }
    }

    mean_abs = sum / (f64)n;
}


// ============================================================
// Main
// ============================================================

#if 0
int main()
{
    std::printf(
        "============================================================\n"
        " Synthetic Camera Calibration Test\n"
        "============================================================\n\n"
    );

    // --------------------------------------------------------
    // Board
    // --------------------------------------------------------

    const int squares_y = 6;
    const int squares_x = 8;

    const int inner_rows = squares_y - 1;
    const int inner_cols = squares_x - 1;

    const int cell = 50;

    const int width = 520;
    const int height = 420;

    const int offset_x = 60;
    const int offset_y = 60;

    std::printf(
        "Board:\n"
        "  squares          = %d x %d\n"
        "  internal corners = %d x %d\n"
        "  total points     = %d\n\n",
        squares_y,
        squares_x,
        inner_rows,
        inner_cols,
        inner_rows * inner_cols
    );

    // --------------------------------------------------------
    // Camera
    // --------------------------------------------------------

    CameraParams cam{};

    cam.fx = width * 0.85f;
    cam.fy = height * 0.85f;
    cam.cx = width * 0.5f;
    cam.cy = height * 0.5f;

    // Ground-truth distortion.
    CalibDistort truth;

    truth.k1 = -0.30;
    truth.k2 = 0.15;
    truth.p1 = 0.02;
    truth.p2 = -0.01;
    truth.k3 = 0.0;

    std::printf(
        "Camera:\n"
        "  fx = %.4f\n"
        "  fy = %.4f\n"
        "  cx = %.4f\n"
        "  cy = %.4f\n\n",
        cam.fx,
        cam.fy,
        cam.cx,
        cam.cy
    );

    std::printf(
        "Ground truth distortion:\n"
        "  k1 = % .8f\n"
        "  k2 = % .8f\n"
        "  p1 = % .8f\n"
        "  p2 = % .8f\n"
        "  k3 = % .8f\n\n",
        truth.k1,
        truth.k2,
        truth.p1,
        truth.p2,
        truth.k3
    );

    // --------------------------------------------------------
    // Generate ideal image
    // --------------------------------------------------------

    GrayImage ideal =
        make_ideal_chessboard(
            width,
            height,
            squares_y,
            squares_x,
            cell,
            offset_x,
            offset_y
        );

    // --------------------------------------------------------
    // Generate distorted image
    // --------------------------------------------------------

    GrayImage distorted =
        make_distorted_image(
            ideal,
            cam.fx,
            cam.fy,
            cam.cx,
            cam.cy,
            truth
        );

    std::printf(
        "Generated synthetic images.\n\n"
    );

    // --------------------------------------------------------
    // Save input images
    // --------------------------------------------------------

    cv::imwrite(
        "ideal.png",
        to_cv_mat(ideal)
    );

    cv::imwrite(
        "distorted.png",
        to_cv_mat(distorted)
    );

    std::printf(
        "Saved:\n"
        "  ideal.png\n"
        "  distorted.png\n\n"
    );

    // --------------------------------------------------------
    // Detect INTERNAL corners
    //
    // IMPORTANT:
    // detect_chessboard() expects INTERNAL dimensions.
    // --------------------------------------------------------

    ChessboardInfo board =
        detect_chessboard(
            distorted,
            inner_rows,
            inner_cols
        );

    if (!board.valid) {

        std::printf(
            "ERROR: chessboard detection failed.\n"
        );

        return 1;
    }

    if ((int)board.corners.size()
        != inner_rows * inner_cols)
    {
        std::printf(
            "ERROR: wrong number of detected corners: %zu\n",
            board.corners.size()
        );

        return 1;
    }

    std::printf(
        "Detected INTERNAL corners:\n"
        "  n = %zu\n",
        board.corners.size()
    );

    // --------------------------------------------------------
    // Subpixel refinement
    // --------------------------------------------------------

    refine_subpixel(
        distorted,
        board.corners,
        7
    );

    std::printf(
        "Subpixel refinement finished.\n\n"
    );

    // --------------------------------------------------------
    // Calibration
    //
    // IMPORTANT:
    //
    // We DO NOT use distorted_truth here.
    //
    // The actual detected corners are used.
    //
    // Ideal board coordinates are known from the synthetic
    // board geometry, so they are used as the calibration
    // reference points.
    // --------------------------------------------------------

    std::vector<Point2f> ideal_points =
        make_ideal_corners(
            inner_rows,
            inner_cols,
            cell,
            offset_x,
            offset_y
        );

    std::vector<Point2f> detected_points =
        board.corners;

    CalibDistort init;

    init.k1 = 0.0;
    init.k2 = 0.0;
    init.p1 = 0.0;
    init.p2 = 0.0;
    init.k3 = 0.0;

    std::printf(
        "------------------------------------------------------------\n"
        "Calibration\n"
        "------------------------------------------------------------\n"
    );

    std::printf(
        "Using %zu detected corners for calibration.\n",
        detected_points.size()
    );

    LMResult calib =
        lm_calibrate_distort(
            detected_points,
            ideal_points,
            cam.fx,
            cam.fy,
            cam.cx,
            cam.cy,
            init,
            200,
            1e-8
        );

    std::printf(
        "\nEstimated distortion:\n"
        "  k1 = % .8f\n"
        "  k2 = % .8f\n"
        "  p1 = % .8f\n"
        "  p2 = % .8f\n"
        "  k3 = % .8f\n"
        "\n"
        "Calibration:\n"
        "  RMS        = %.8f px\n"
        "  iterations = %d\n"
        "  converged  = %s\n\n",
        calib.d.k1,
        calib.d.k2,
        calib.d.p1,
        calib.d.p2,
        calib.d.k3,
        calib.final_error,
        calib.iterations,
        calib.converged ? "YES" : "NO"
    );

    // --------------------------------------------------------
    // Compare estimated parameters with ground truth
    //
    // This is only for displaying the diagnostic result.
    // It is NOT fed back into calibration.
    // --------------------------------------------------------

    std::printf(
        "Parameter error:\n"
        "  dk1 = % .8f\n"
        "  dk2 = % .8f\n"
        "  dp1 = % .8f\n"
        "  dp2 = % .8f\n"
        "  dk3 = % .8f\n\n",
        calib.d.k1 - truth.k1,
        calib.d.k2 - truth.k2,
        calib.d.p1 - truth.p1,
        calib.d.p2 - truth.p2,
        calib.d.k3 - truth.k3
    );

    // --------------------------------------------------------
    // Undistort
    //
    // Use ONLY estimated calibration parameters.
    // --------------------------------------------------------

    CameraParams estimated_cam = cam;

    estimated_cam.k1 = (f32)calib.d.k1;
    estimated_cam.k2 = (f32)calib.d.k2;
    estimated_cam.p1 = (f32)calib.d.p1;
    estimated_cam.p2 = (f32)calib.d.p2;
    estimated_cam.k3 = (f32)calib.d.k3;

    GrayImage undistorted(
        width,
        height
    );

    RemapTable remap =
        build_remap_table(
            width,
            height,
            estimated_cam
        );

    remap_bilinear(
        distorted,
        remap,
        undistorted
    );

    // --------------------------------------------------------
    // Compare undistorted image with original ideal image
    // --------------------------------------------------------

    f64 mean_abs = 0.0;
    int max_err = 0;

    image_error(
        ideal,
        undistorted,
        mean_abs,
        max_err
    );

    std::printf(
        "------------------------------------------------------------\n"
        "Undistortion Result\n"
        "------------------------------------------------------------\n"
        "Mean absolute image error = %.6f\n"
        "Max image error           = %d\n\n",
        mean_abs,
        max_err
    );

    // --------------------------------------------------------
    // Save result
    // --------------------------------------------------------

    cv::imwrite(
        "undistorted.png",
        to_cv_mat(undistorted)
    );

    // --------------------------------------------------------
    // Also save an absolute difference image
    // --------------------------------------------------------

    cv::Mat ideal_cv =
        to_cv_mat(ideal);

    cv::Mat undistorted_cv =
        to_cv_mat(undistorted);

    cv::Mat diff;

    cv::absdiff(
        ideal_cv,
        undistorted_cv,
        diff
    );

    cv::imwrite(
        "undistort_diff.png",
        diff
    );

    std::printf(
        "Saved:\n"
        "  undistorted.png\n"
        "  undistort_diff.png\n\n"
    );

    // --------------------------------------------------------
    // Visualization
    // --------------------------------------------------------

    cv::Mat distorted_cv =
        to_cv_mat(distorted);

    cv::Mat detected_view =
        distorted_cv.clone();

    for (const auto& p : detected_points) {

        cv::circle(
            detected_view,
            cv::Point(
                (int)std::lround(p.x),
                (int)std::lround(p.y)
            ),
            3,
            cv::Scalar(255),
            -1
        );
    }

    cv::imshow(
        "Ideal",
        ideal_cv
    );

    cv::imshow(
        "Distorted",
        distorted_cv
    );

    cv::imshow(
        "Detected Corners",
        detected_view
    );

    cv::imshow(
        "Undistorted",
        undistorted_cv
    );

    cv::imshow(
        "Undistortion Error",
        diff
    );

    std::printf(
        "Press any key in an OpenCV window to exit.\n"
    );

    cv::waitKey(0);

    return 0;
}
#endif