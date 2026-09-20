#pragma once
#include "../common/types.h"
#include <array>
#include <string>
#include <vector>

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
