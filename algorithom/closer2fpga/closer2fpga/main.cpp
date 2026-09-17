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
// 生成理想棋盘
// ============================================================

static GrayImage make_checkerboard(
    int cell,
    int square_rows,
    int square_cols,
    int pad)
{
    int h = cell * square_rows + pad * 2;
    int w = cell * square_cols + pad * 2;

    GrayImage img(w, h);

    for (int i = 0; i < w * h; ++i)
        img.data[i] = 0;

    for (int y = pad; y < pad + cell * square_rows; ++y) {

        int ry = (y - pad) / cell;

        for (int x = pad; x < pad + cell * square_cols; ++x) {

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


// ============================================================
// 生成 INTERNAL corners 的 Ground Truth
//
// 6 x 8 squares
// => 5 x 7 internal corners
//
// 顺序：row-major
//
// (1,1) (1,2) ... (1,7)
// (2,1) (2,2) ... (2,7)
// ...
// (5,1) ...      (5,7)
// ============================================================

static std::vector<Point2f> make_inner_ground_truth(
    int pad,
    int cell,
    int square_rows,
    int square_cols)
{
    std::vector<Point2f> pts;

    int inner_rows = square_rows - 1;
    int inner_cols = square_cols - 1;

    pts.reserve(inner_rows * inner_cols);

    for (int r = 1; r < square_rows; ++r) {

        for (int c = 1; c < square_cols; ++c) {

            pts.push_back({
                (f32)(pad + c * cell),
                (f32)(pad + r * cell)
                });
        }
    }

    return pts;
}


// ============================================================
// 使用真实 CameraParams 将理想角点变成畸变角点
// ============================================================

static std::vector<Point2f> distort_points(
    const std::vector<Point2f>& ideal_pts,
    const CameraParams& cam)
{
    std::vector<Point2f> result;

    result.reserve(ideal_pts.size());

    for (const auto& ip : ideal_pts) {

        f32 nx =
            (ip.x - cam.cx) / cam.fx;

        f32 ny =
            (ip.y - cam.cy) / cam.fy;

        f32 xd;
        f32 yd;

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


// ============================================================
// 打印点统计
// ============================================================

static void print_points(
    const char* tag,
    const std::vector<Point2f>& pts)
{
    if (pts.empty()) {

        printf(
            "  [%s] EMPTY\n",
            tag
        );

        return;
    }

    f32 x_min = 1e9f;
    f32 x_max = -1e9f;
    f32 y_min = 1e9f;
    f32 y_max = -1e9f;

    f32 sx = 0;
    f32 sy = 0;

    for (const auto& p : pts) {

        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);

        y_min = std::min(y_min, p.y);
        y_max = std::max(y_max, p.y);

        sx += p.x;
        sy += p.y;
    }

    printf(
        "  [%s] n=%zu  "
        "x=[%.2f..%.2f] mean_x=%.2f  "
        "y=[%.2f..%.2f] mean_y=%.2f\n",
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


// ============================================================
// 计算点误差
// ============================================================

static bool calculate_accuracy(
    const std::vector<Point2f>& a,
    const std::vector<Point2f>& b,
    f32& mean_err,
    f32& max_err)
{
    mean_err = 0;
    max_err = 0;

    if (a.size() != b.size() || a.empty())
        return false;

    for (size_t i = 0; i < a.size(); ++i) {

        f32 dx = a[i].x - b[i].x;
        f32 dy = a[i].y - b[i].y;

        f32 e =
            std::sqrt(dx * dx + dy * dy);

        mean_err += e;

        max_err =
            std::max(max_err, e);
    }

    mean_err /= (f32)a.size();

    return true;
}


static void print_accuracy(
    const char* tag,
    const std::vector<Point2f>& detected,
    const std::vector<Point2f>& truth)
{
    f32 mean_err;
    f32 max_err;

    if (!calculate_accuracy(
        detected,
        truth,
        mean_err,
        max_err))
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

    printf(
        "  [%s] mean=%.6f px  max=%.6f px\n",
        tag,
        mean_err,
        max_err
    );
}


// ============================================================
// 打印点对应关系
//
// 用于检查：
// detected[i] 是否真的对应 truth[i]
// ============================================================

static void print_point_correspondence(
    const std::vector<Point2f>& detected,
    const std::vector<Point2f>& truth,
    int count)
{
    printf(
        "\n"
        "  --------------------------------------------------\n"
        "  Point correspondence check\n"
        "  --------------------------------------------------\n"
    );

    int n =
        (int)std::min(
            std::min(
                detected.size(),
                truth.size()
            ),
            (size_t)count
        );

    for (int i = 0; i < n; ++i) {

        f32 dx =
            detected[i].x - truth[i].x;

        f32 dy =
            detected[i].y - truth[i].y;

        f32 e =
            std::sqrt(dx * dx + dy * dy);

        printf(
            "  [%2d] detected=(%9.3f,%9.3f) "
            "truth=(%9.3f,%9.3f) "
            "error=%9.3f\n",
            i,
            detected[i].x,
            detected[i].y,
            truth[i].x,
            truth[i].y,
            e
        );
    }
}


// ============================================================
// 打印畸变参数
// ============================================================

static void print_distortion(
    const char* tag,
    const CalibDistort& d)
{
    printf(
        "  [%s]\n"
        "    k1 = % .8f\n"
        "    k2 = % .8f\n"
        "    p1 = % .8f\n"
        "    p2 = % .8f\n"
        "    k3 = % .8f\n",
        tag,
        d.k1,
        d.k2,
        d.p1,
        d.p2,
        d.k3
    );
}


// ============================================================
// 打印畸变参数误差
// ============================================================

static void print_distortion_error(
    const CalibDistort& estimated,
    const CalibDistort& truth)
{
    printf(
        "  [parameter error]\n"
        "    dk1 = % .8f\n"
        "    dk2 = % .8f\n"
        "    dp1 = % .8f\n"
        "    dp2 = % .8f\n"
        "    dk3 = % .8f\n",
        estimated.k1 - truth.k1,
        estimated.k2 - truth.k2,
        estimated.p1 - truth.p1,
        estimated.p2 - truth.p2,
        estimated.k3 - truth.k3
    );
}


// ============================================================
// GrayImage -> OpenCV Mat
// ============================================================

static cv::Mat gray_to_mat(
    const GrayImage& img)
{
    return cv::Mat(
        img.h,
        img.w,
        CV_8UC1,
        img.data
    );
}


// ============================================================
// 绘制 INTERNAL corners
// ============================================================

static cv::Mat draw_internal_corners(
    const GrayImage& gray,
    const std::vector<Point2f>& corners,
    const char* title)
{
    cv::Mat gray_mat =
        gray_to_mat(gray);

    cv::Mat disp;

    cv::cvtColor(
        gray_mat,
        disp,
        cv::COLOR_GRAY2BGR
    );

    for (size_t i = 0; i < corners.size(); ++i) {

        const auto& p =
            corners[i];

        cv::Point q(
            (int)std::lround(p.x),
            (int)std::lround(p.y)
        );

        cv::circle(
            disp,
            q,
            4,
            cv::Scalar(0, 0, 255),
            -1
        );

        cv::putText(
            disp,
            std::to_string(i),
            q + cv::Point(5, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.35,
            cv::Scalar(255, 255, 255),
            1
        );
    }

    cv::putText(
        disp,
        title,
        cv::Point(10, 25),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(0, 255, 255),
        2
    );

    return disp;
}


// ============================================================
// 绘制 detection vs truth
//
// 绿色 = truth
// 红色 = detected
// ============================================================

static cv::Mat draw_detection_comparison(
    const GrayImage& gray,
    const std::vector<Point2f>& detected,
    const std::vector<Point2f>& truth,
    const char* title)
{
    cv::Mat gray_mat =
        gray_to_mat(gray);

    cv::Mat disp;

    cv::cvtColor(
        gray_mat,
        disp,
        cv::COLOR_GRAY2BGR
    );

    for (const auto& p : truth) {

        cv::circle(
            disp,
            cv::Point(
                (int)std::lround(p.x),
                (int)std::lround(p.y)
            ),
            5,
            cv::Scalar(0, 255, 0),
            1
        );
    }

    for (const auto& p : detected) {

        cv::circle(
            disp,
            cv::Point(
                (int)std::lround(p.x),
                (int)std::lround(p.y)
            ),
            3,
            cv::Scalar(0, 0, 255),
            -1
        );
    }

    cv::putText(
        disp,
        title,
        cv::Point(10, 25),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        2
    );

    return disp;
}


// ============================================================
// 图像差异
// ============================================================

static cv::Mat make_difference_image(
    const GrayImage& a,
    const GrayImage& b)
{
    cv::Mat ma =
        gray_to_mat(a);

    cv::Mat mb =
        gray_to_mat(b);

    cv::Mat diff;

    cv::absdiff(
        ma,
        mb,
        diff
    );

    return diff;
}


// ============================================================
// CameraParams -> CalibDistort
// ============================================================

static CalibDistort camera_to_distortion(
    const CameraParams& cam)
{
    CalibDistort d;

    d.k1 = cam.k1;
    d.k2 = cam.k2;
    d.p1 = cam.p1;
    d.p2 = cam.p2;
    d.k3 = cam.k3;

    return d;
}


// ============================================================
// 创建带有指定畸变参数的 CameraParams
// ============================================================

static CameraParams make_camera_with_distortion(
    const CameraParams& base,
    const CalibDistort& d)
{
    CameraParams cam = base;

    cam.k1 = (f32)d.k1;
    cam.k2 = (f32)d.k2;
    cam.p1 = (f32)d.p1;
    cam.p2 = (f32)d.p2;
    cam.k3 = (f32)d.k3;

    cam.valid = true;

    return cam;
}


// ============================================================
// 使用 CameraParams 对图像去畸变
// ============================================================

static GrayImage undistort_with_camera(
    const GrayImage& distorted,
    const CameraParams& cam)
{
    RemapTable remap =
        build_remap_table(
            distorted.w,
            distorted.h,
            cam
        );

    GrayImage result;

    remap_bilinear(
        distorted,
        remap,
        result
    );

    return result;
}


// ============================================================
// 图像差异统计
// ============================================================

static void print_image_difference(
    const char* tag,
    const GrayImage& a,
    const GrayImage& b)
{
    cv::Mat diff =
        make_difference_image(a, b);

    double mean_diff =
        cv::mean(diff)[0];

    double max_diff = 0;

    cv::minMaxLoc(
        diff,
        nullptr,
        &max_diff
    );

    printf(
        "  [%s]\n"
        "    mean absolute difference = %.6f\n"
        "    max absolute difference  = %.6f\n",
        tag,
        mean_diff,
        max_diff
    );
}


// ============================================================
// main
// ============================================================

int main()
{
    // ========================================================
    // 1. 基本参数
    // ========================================================

    const int pad = 60;
    const int cell = 50;

    const int square_rows = 6;
    const int square_cols = 8;

    const int inner_rows =
        square_rows - 1;

    const int inner_cols =
        square_cols - 1;

    const int inner_need =
        inner_rows * inner_cols;


    printf(
        "============================================================\n"
        " Synthetic Camera Calibration Diagnostic Test\n"
        "============================================================\n\n"
    );


    printf(
        "Board:\n"
        "  squares          = %d x %d\n"
        "  internal corners = %d x %d\n"
        "  total points     = %d\n\n",
        square_rows,
        square_cols,
        inner_rows,
        inner_cols,
        inner_need
    );


    // ========================================================
    // 2. 生成理想棋盘
    // ========================================================

    GrayImage ideal_img =
        make_checkerboard(
            cell,
            square_rows,
            square_cols,
            pad
        );


    printf(
        "Image:\n"
        "  size = %d x %d\n\n",
        ideal_img.w,
        ideal_img.h
    );


    // ========================================================
    // 3. Ground Truth Camera
    // ========================================================

    CameraParams cam = {};

    cam.fx =
        ideal_img.w * 0.85f;

    cam.fy =
        ideal_img.h * 0.85f;

    cam.cx =
        ideal_img.w * 0.5f;

    cam.cy =
        ideal_img.h * 0.5f;

    // 径向畸变
    cam.k1 = -0.30f;
    cam.k2 = 0.15f;
    cam.k3 = 0.00f;

    // 切向畸变
    cam.p1 = 0.02f;
    cam.p2 = -0.01f;

    cam.valid = true;


    printf(
        "------------------------------------------------------------\n"
        "Ground Truth Camera\n"
        "------------------------------------------------------------\n"
        "  fx = %.4f\n"
        "  fy = %.4f\n"
        "  cx = %.4f\n"
        "  cy = %.4f\n\n",
        cam.fx,
        cam.fy,
        cam.cx,
        cam.cy
    );


    CalibDistort truth_dist =
        camera_to_distortion(cam);


    print_distortion(
        "Ground truth distortion",
        truth_dist
    );


    // ========================================================
    // 4. 生成畸变图像
    // ========================================================

    GrayImage distorted_img;

    make_distorted_image(
        ideal_img,
        cam,
        distorted_img
    );


    printf(
        "\n------------------------------------------------------------\n"
        "Generated distorted image\n"
        "------------------------------------------------------------\n"
        "  size = %d x %d\n\n",
        distorted_img.w,
        distorted_img.h
    );


    // ========================================================
    // 5. Ground Truth corners
    // ========================================================

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
        "Ground truth corners:\n"
        "  ideal     = %zu\n"
        "  distorted = %zu\n\n",
        ideal_truth.size(),
        distorted_truth.size()
    );


    print_points(
        "Ideal truth",
        ideal_truth
    );

    print_points(
        "Distorted truth",
        distorted_truth
    );


    // ========================================================
    // TEST 1
    //
    // 使用 Ground Truth 参数直接去畸变
    //
    // 目的：
    // 单独检查 undistort.cpp
    // ========================================================

    printf(
        "\n============================================================\n"
        "TEST 1: Ground Truth Parameters -> Undistortion\n"
        "============================================================\n"
    );


    GrayImage gt_undistorted =
        undistort_with_camera(
            distorted_img,
            cam
        );


    printf(
        "\nGround truth parameter undistortion finished.\n"
    );


    print_image_difference(
        "Ideal vs GT-undistorted",
        ideal_img,
        gt_undistorted
    );


    // --------------------------------------------------------
    // GT 去畸变后再次检测棋盘
    // --------------------------------------------------------

    ChessboardInfo gt_info =
        detect_chessboard(
            gt_undistorted,

            // 注意：
            // detect_chessboard 的参数是
            // INTERNAL corner 数量
            //
            // 这里必须是：
            // 5 x 7
            inner_rows,
            inner_cols
        );


    std::vector<Point2f> gt_undist_detected;


    if (gt_info.valid) {

        gt_undist_detected =
            gt_info.corners;

        refine_subpixel(
            gt_undistorted,
            gt_undist_detected,
            7
        );

        print_points(
            "GT-undistorted detected",
            gt_undist_detected
        );

        print_accuracy(
            "GT-undistorted vs ideal",
            gt_undist_detected,
            ideal_truth
        );

    }
    else {

        printf(
            "  WARNING: corner detection failed "
            "after GT undistortion.\n"
        );
    }


    // ========================================================
    // TEST 2
    //
    // 从畸变图像检测 INTERNAL corners
    //
    // 目的：
    // 检查：
    //   1. Shi-Tomasi
    //   2. clustering
    //   3. chessboard grouping
    //   4. corner ordering
    // ========================================================

    printf(
        "\n============================================================\n"
        "TEST 2: Detect INTERNAL corners from DISTORTED image\n"
        "============================================================\n"
    );


    std::vector<Point2f> raw;


    shi_tomasi_detect(
        distorted_img,
        raw,
        0.15f,
        3
    );


    print_points(
        "Shi-Tomasi raw",
        raw
    );


    // --------------------------------------------------------
    // 这里必须传 INTERNAL corner 数量
    //
    // 5 x 7
    // --------------------------------------------------------

    ChessboardInfo info =
        detect_chessboard(
            distorted_img,
            inner_rows,
            inner_cols
        );


    if (!info.valid) {

        printf(
            "\nERROR:\n"
            "  detect_chessboard() failed.\n"
        );

        cv::imshow(
            "Detection Failed",
            gray_to_mat(distorted_img)
        );

        cv::waitKey(0);

        return 1;
    }


    std::vector<Point2f> detected =
        info.corners;


    print_points(
        "Detected INTERNAL corners",
        detected
    );


    printf(
        "\n"
        "Expected internal corners = %d\n"
        "Detected internal corners = %zu\n",
        inner_need,
        detected.size()
    );


    // ========================================================
    // Subpixel refinement
    // ========================================================

    printf(
        "\n------------------------------------------------------------\n"
        "Subpixel refinement\n"
        "------------------------------------------------------------\n"
    );


    refine_subpixel(
        distorted_img,
        detected,
        7
    );


    print_points(
        "Detected + subpixel",
        detected
    );


    // ========================================================
    // 检查检测精度
    // ========================================================

    print_accuracy(
        "Detection accuracy",
        detected,
        distorted_truth
    );


    // ========================================================
    // 检查点对应关系
    // ========================================================

    print_point_correspondence(
        detected,
        distorted_truth,
        10
    );


    if ((int)detected.size() != inner_need) {

        printf(
            "\nERROR:\n"
            "  Expected %d corners, but detected %zu.\n",
            inner_need,
            detected.size()
        );

        cv::waitKey(0);

        return 1;
    }


    // ========================================================
    // TEST 3
    //
    // LM calibration
    //
    // image_pts = 畸变图像中检测到的点
    // ideal_pts = 理想棋盘点
    //
    // 固定：
    //   fx fy cx cy
    //
    // 优化：
    //   k1 k2 p1 p2 k3
    // ========================================================

    printf(
        "\n============================================================\n"
        "TEST 3: LM Distortion Calibration\n"
        "============================================================\n"
    );


    CalibDistort init = {};

    init.k1 = 0;
    init.k2 = 0;
    init.p1 = 0;
    init.p2 = 0;
    init.k3 = 0;


    printf(
        "\nInitial parameters:\n"
    );


    print_distortion(
        "Initial",
        init
    );


    LMResult calib =
        lm_calibrate_distort(
            detected,
            //distorted_truth,
            ideal_truth,
            cam.fx,
            cam.fy,
            cam.cx,
            cam.cy,
            init,
            200,
            1e-8
        );


    printf(
        "\n------------------------------------------------------------\n"
        "Calibration Result\n"
        "------------------------------------------------------------\n"
    );


    print_distortion(
        "Estimated",
        calib.d
    );


    printf("\n");


    print_distortion(
        "Ground truth",
        truth_dist
    );


    printf("\n");


    print_distortion_error(
        calib.d,
        truth_dist
    );


    double rms =
        std::sqrt(
            calib.final_error /
            (2.0 * (f64)detected.size())
        );


    printf(
        "\n"
        "  final SSE  = %.10f\n"
        "  final RMS  = %.10f px\n"
        "  iterations = %d\n"
        "  converged  = %s\n",
        calib.final_error,
        rms,
        calib.iterations,
        calib.converged ? "YES" : "NO"
    );


    // ========================================================
    // TEST 4
    //
    // 使用 LM 估计参数去畸变
    // ========================================================

    printf(
        "\n============================================================\n"
        "TEST 4: Estimated Parameters -> Undistortion\n"
        "============================================================\n"
    );


    CameraParams estimated_cam =
        make_camera_with_distortion(
            cam,
            calib.d
        );


    printf(
        "\nEstimated CameraParams:\n"
        "  fx = %.6f\n"
        "  fy = %.6f\n"
        "  cx = %.6f\n"
        "  cy = %.6f\n",
        estimated_cam.fx,
        estimated_cam.fy,
        estimated_cam.cx,
        estimated_cam.cy
    );


    print_distortion(
        "Estimated distortion used for remap",
        calib.d
    );


    printf(
        "\nBuilding remap table...\n"
    );


    GrayImage estimated_undistorted =
        undistort_with_camera(
            distorted_img,
            estimated_cam
        );


    printf(
        "Undistortion finished.\n"
    );


    print_image_difference(
        "Ideal vs estimated-undistorted",
        ideal_img,
        estimated_undistorted
    );


    // --------------------------------------------------------
    // 去畸变之后再次检测角点
    // --------------------------------------------------------

    printf(
        "\n------------------------------------------------------------\n"
        "Detect corners after estimated undistortion\n"
        "------------------------------------------------------------\n"
    );


    std::vector<Point2f> estimated_undist_detected;


    ChessboardInfo estimated_info =
        detect_chessboard(
            estimated_undistorted,
            inner_rows,
            inner_cols
        );


    if (estimated_info.valid) {

        estimated_undist_detected =
            estimated_info.corners;

        refine_subpixel(
            estimated_undistorted,
            estimated_undist_detected,
            7
        );

        print_points(
            "Estimated-undistorted detected",
            estimated_undist_detected
        );

        print_accuracy(
            "Estimated-undistorted vs ideal",
            estimated_undist_detected,
            ideal_truth
        );

    }
    else {

        printf(
            "  WARNING: failed to detect corners "
            "after estimated undistortion.\n"
        );
    }


    // ========================================================
    // FINAL SUMMARY
    // ========================================================

    printf(
        "\n============================================================\n"
        "FINAL DIAGNOSTIC SUMMARY\n"
        "============================================================\n"
    );


    // --------------------------------------------------------
    // Test 1
    // --------------------------------------------------------

    f32 gt_corner_mean = 0;
    f32 gt_corner_max = 0;

    bool gt_corner_ok =
        calculate_accuracy(
            gt_undist_detected,
            ideal_truth,
            gt_corner_mean,
            gt_corner_max
        );


    printf(
        "\n[1] Ground-truth undistortion\n"
        "    Checks undistort.cpp independently.\n"
    );


    if (gt_corner_ok) {

        printf(
            "    corner mean error = %.6f px\n"
            "    corner max error  = %.6f px\n",
            gt_corner_mean,
            gt_corner_max
        );

    }
    else {

        printf(
            "    corner detection FAILED\n"
        );
    }


    // --------------------------------------------------------
    // Test 2
    // --------------------------------------------------------

    f32 detect_mean = 0;
    f32 detect_max = 0;

    bool detect_ok =
        calculate_accuracy(
            detected,
            distorted_truth,
            detect_mean,
            detect_max
        );


    printf(
        "\n[2] Distorted corner detection\n"
        "    Checks detector + ordering.\n"
    );


    if (detect_ok) {

        printf(
            "    mean error = %.6f px\n"
            "    max error  = %.6f px\n",
            detect_mean,
            detect_max
        );

    }
    else {

        printf(
            "    detection FAILED\n"
        );
    }


    // --------------------------------------------------------
    // Test 3
    // --------------------------------------------------------

    printf(
        "\n[3] LM calibration\n"
        "    Checks distortion parameter estimation.\n"
    );


    printf(
        "    converged = %s\n"
        "    RMS       = %.8f px\n",
        calib.converged ? "YES" : "NO",
        rms
    );


    // --------------------------------------------------------
    // Test 4
    // --------------------------------------------------------

    f32 est_corner_mean = 0;
    f32 est_corner_max = 0;

    bool est_corner_ok =
        calculate_accuracy(
            estimated_undist_detected,
            ideal_truth,
            est_corner_mean,
            est_corner_max
        );


    printf(
        "\n[4] Estimated-parameter undistortion\n"
        "    Checks the complete calibration pipeline.\n"
    );


    if (est_corner_ok) {

        printf(
            "    corner mean error = %.6f px\n"
            "    corner max error  = %.6f px\n",
            est_corner_mean,
            est_corner_max
        );

    }
    else {

        printf(
            "    corner detection FAILED\n"
        );
    }


    // ========================================================
    // Visualization
    // ========================================================

    printf(
        "\n============================================================\n"
        "Visualization\n"
        "============================================================\n"
        "\n"
        "1. Ideal\n"
        "2. Distorted\n"
        "3. Distorted + detected corners\n"
        "4. GT parameter undistortion\n"
        "5. Estimated parameter undistortion\n"
        "6. GT undistortion + corners\n"
        "7. Estimated undistortion + corners\n"
        "8. Ideal vs estimated difference\n"
        "\n"
        "Press any key to exit.\n"
    );


    // --------------------------------------------------------
    // Window 1: Ideal
    // --------------------------------------------------------

    {
        cv::Mat mat =
            gray_to_mat(ideal_img);

        cv::Mat disp;

        cv::resize(
            mat,
            disp,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::putText(
            disp,
            "Ideal chessboard",
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(255),
            2
        );

        cv::imshow(
            "1 - Ideal",
            disp
        );
    }


    // --------------------------------------------------------
    // Window 2: Distorted
    // --------------------------------------------------------

    {
        cv::Mat mat =
            gray_to_mat(distorted_img);

        cv::Mat disp;

        cv::resize(
            mat,
            disp,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::putText(
            disp,
            "Distorted image",
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(255),
            2
        );

        cv::imshow(
            "2 - Distorted",
            disp
        );
    }


    // --------------------------------------------------------
    // Window 3: Distorted + detected
    // --------------------------------------------------------

    {
        cv::Mat comparison =
            draw_detection_comparison(
                distorted_img,
                detected,
                distorted_truth,
                "Distorted: RED=detected GREEN=truth"
            );

        cv::Mat big;

        cv::resize(
            comparison,
            big,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::imshow(
            "3 - Distorted + Corners",
            big
        );
    }


    // --------------------------------------------------------
    // Window 4: GT undistortion
    // --------------------------------------------------------

    {
        cv::Mat mat =
            gray_to_mat(gt_undistorted);

        cv::Mat disp;

        cv::resize(
            mat,
            disp,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::putText(
            disp,
            "Undistorted using GROUND TRUTH parameters",
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(255),
            2
        );

        cv::imshow(
            "4 - GT Undistorted",
            disp
        );
    }


    // --------------------------------------------------------
    // Window 5: Estimated undistortion
    // --------------------------------------------------------

    {
        cv::Mat mat =
            gray_to_mat(estimated_undistorted);

        cv::Mat disp;

        cv::resize(
            mat,
            disp,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::putText(
            disp,
            "Undistorted using ESTIMATED parameters",
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(255),
            2
        );

        cv::imshow(
            "5 - Estimated Undistorted",
            disp
        );
    }


    // --------------------------------------------------------
    // Window 6: GT undistortion + corners
    // --------------------------------------------------------

    if (!gt_undist_detected.empty()) {

        cv::Mat disp =
            draw_internal_corners(
                gt_undistorted,
                gt_undist_detected,
                "GT undistorted + INTERNAL corners"
            );

        cv::Mat big;

        cv::resize(
            disp,
            big,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::imshow(
            "6 - GT Undistorted + Corners",
            big
        );
    }


    // --------------------------------------------------------
    // Window 7: Estimated undistortion + corners
    // --------------------------------------------------------

    if (!estimated_undist_detected.empty()) {

        cv::Mat disp =
            draw_internal_corners(
                estimated_undistorted,
                estimated_undist_detected,
                "Estimated undistorted + INTERNAL corners"
            );

        cv::Mat big;

        cv::resize(
            disp,
            big,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::imshow(
            "7 - Estimated Undistorted + Corners",
            big
        );
    }


    // --------------------------------------------------------
    // Window 8: Difference
    // --------------------------------------------------------

    {
        cv::Mat diff =
            make_difference_image(
                ideal_img,
                estimated_undistorted
            );

        cv::Mat diff_big;

        cv::resize(
            diff,
            diff_big,
            cv::Size(),
            2.0,
            2.0,
            cv::INTER_NEAREST
        );

        cv::putText(
            diff_big,
            "Absolute difference: ideal vs estimated undistorted",
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255),
            2
        );

        cv::imshow(
            "8 - Difference",
            diff_big
        );
    }


    cv::waitKey(0);

    return 0;
}