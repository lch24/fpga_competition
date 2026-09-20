#pragma once
#include "../calibrate.h"
#include "../../common/math3.h"
#include <limits>

// Internal calibration stages, called by the public calibrate_camera controller.
namespace calibration {
using math3::cross;
using math3::dot;
using math3::M3;
using math3::multiply;
using math3::rodrigues;
using math3::rotation_vector;
using math3::scaled;
using math3::V3;
using State = std::vector<double>;
using Points = std::vector<std::vector<Point2f>>;
constexpr double infinity = std::numeric_limits<double>::infinity();

// State layout: log(fx), log(fy), cx/width, cy/height, k1,k2,p1,p2,k3,
// followed by [rotation-vector(3), tx,ty,log(tz)] for each view.
bool homography(const std::vector<Point2f>& points, int rows, int cols, M3& h);
bool zhang_intrinsics(const std::vector<M3>& homographies, int width, int height,
                      std::array<double, 4>& intrinsics);
bool initialize(const std::vector<M3>& homographies, const std::array<double, 4>& intrinsics, int width,
                int height, State& state);
double residuals(const State& state, const Points& points, int width, int height, int rows, int cols,
                 std::vector<double>& residual);
bool optimize(State& state, const Points& points, int width, int height, int rows, int cols,
              const std::vector<int>& active, int limit, int& iterations);
void finish_result(const State& best, const Points& points, int width, int height, int rows, int cols,
                   double square_size, double best_cost, bool converged, int iterations,
                   CameraCalibrationResult& result);
} // namespace calibration
