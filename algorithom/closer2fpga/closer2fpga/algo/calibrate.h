#pragma once

#include "../common/types.h"
#include <vector>
#include <array>
#include <string>

// Forward declaration.
struct CameraParams;

// ------------------------------------------------------------
// Distortion parameters
//
// OpenCV-style Brown-Conrady model:
//
// radial:
//   1 + k1*r2 + k2*r4 + k3*r6
//
// tangential:
//   xt = 2*p1*x*y + p2*(r2 + 2*x*x)
//   yt = p1*(r2 + 2*y*y) + 2*p2*x*y
// ------------------------------------------------------------

struct CalibDistort {
    f64 k1 = 0.0;
    f64 k2 = 0.0;
    f64 p1 = 0.0;
    f64 p2 = 0.0;
    f64 k3 = 0.0;
};

// ------------------------------------------------------------
// Radial distortion factor
// ------------------------------------------------------------

f64 radial_distort(
    f64 x_n,
    f64 y_n,
    const CalibDistort& d
);

// ------------------------------------------------------------
// Project an ideal image point to a distorted image point.
//
// x_ideal, y_ideal:
//     ideal pixel coordinates
//
// fx, fy, cx, cy:
//     known camera intrinsics
//
// d:
//     distortion parameters
//
// x_d, y_d:
//     distorted pixel coordinates
// ------------------------------------------------------------

void project_distorted(
    f64 x_ideal,
    f64 y_ideal,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& d,
    f64& x_d,
    f64& y_d
);

// ------------------------------------------------------------
// LM calibration result
// ------------------------------------------------------------

struct LMResult {
    CalibDistort d;

    // Sum of squared reprojection error.
    f64 final_error = 0.0;

    // Number of accepted optimization iterations.
    int iterations = 0;

    bool converged = false;
};

// ------------------------------------------------------------
// Optimize distortion parameters.
//
// IMPORTANT:
//
// This function currently assumes:
//   fx, fy, cx, cy are already known.
//
// It optimizes only:
//
//   k1
//   k2
//   p1
//   p2
//   k3
//
// image_pts:
//     observed/distorted image points
//
// ideal_pts:
//     corresponding ideal image points
//
// This is therefore a "known intrinsics + distortion fitting"
// test, NOT a complete camera calibration.
// ------------------------------------------------------------

LMResult lm_calibrate_distort(
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& init,
    int max_iter = 200,
    f64 tol = 1e-6
);

// Full planar-board calibration. Every view contains rows*cols measured pixel
// coordinates in row-major order. Square size scales translations only; use 1
// if its physical length is unknown. No OpenCV dependency.
struct CameraPose {
    std::array<f64, 9> rotation{}; // row-major, board -> camera
    std::array<f64, 3> translation{};
};

struct CameraCalibrationOptions {
    int max_iterations = 150; // per optimization stage
    bool estimate_k3 = false; // keep the highest radial term fixed for sparse data
};

struct CameraCalibrationResult {
    CameraParams camera{};
    std::vector<CameraPose> poses;
    std::vector<f64> per_view_rms;
    f64 rms = 0; // sqrt(sum(dx*dx + dy*dy) / total_corner_count), pixels
    f64 max_error = 0;
    int iterations = 0;
    bool converged = false;
    bool weak_geometry = false;
    bool k3_estimated = false;
    std::string message;
};

CameraCalibrationResult calibrate_camera(
    const std::vector<std::vector<Point2f>>& image_points,
    int width, int height, int rows, int cols, f64 square_size = 1.0,
    const CameraCalibrationOptions& options = {});
