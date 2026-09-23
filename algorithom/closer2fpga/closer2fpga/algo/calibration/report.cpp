#include "internal.h"
#include <algorithm>
#include <cmath>

namespace calibration {
bool mapping_is_regular(const CameraParams& k, int w, int h) {
    // Require positive Jacobian determinant throughout the output field to
    // reject folding maps caused by extrapolated high-order radial coefficients.
    for (int iy = 0; iy <= 24; ++iy)
        for (int ix = 0; ix <= 32; ++ix) {
            double x = ((w - 1) * ix / 32. - k.cx) / k.fx, y = ((h - 1) * iy / 24. - k.cy) / k.fy;
            double r2 = x * x + y * y, radial = 1 + k.k1 * r2 + k.k2 * r2 * r2 + k.k3 * r2 * r2 * r2;
            double dr = k.k1 + 2 * k.k2 * r2 + 3 * k.k3 * r2 * r2;
            double a = radial + 2 * x * x * dr + 2 * k.p1 * y + 6 * k.p2 * x;
            double b = 2 * x * y * dr + 2 * k.p1 * x + 2 * k.p2 * y;
            double d = radial + 2 * y * y * dr + 6 * k.p1 * y + 2 * k.p2 * x;
            if (!std::isfinite(a * d - b * b) || a <= 0 || d <= 0 || a * d - b * b <= 1e-4)
                return false;
        }
    return true;
}

void finish_result(const State& best, const Points& points, int width, int height, int rows, int cols,
                   double square_size, double best_cost, bool best_converged, int best_iterations,
                   CameraCalibrationResult& result) {
    auto& k = result.camera;
    k.fx = float(std::exp(best[0]));
    k.fy = float(std::exp(best[1]));
    k.cx = float(best[2] * width);
    k.cy = float(best[3] * height);
    k.k1 = float(best[4]);
    k.k2 = float(best[5]);
    k.p1 = float(best[6]);
    k.p2 = float(best[7]);
    k.k3 = float(best[8]);
    result.converged = best_converged;
    result.iterations = best_iterations;
    std::vector<double> residual;
    residuals(best, points, width, height, rows, cols, residual);
    result.rms = std::sqrt(best_cost / (points.size() * rows * cols));
    for (size_t i = 0; i < points.size(); ++i) {
        double cost = 0;
        for (int j = 0; j < rows * cols; ++j) {
            size_t id = 2 * (i * rows * cols + j);
            double error = std::hypot(residual[id], residual[id + 1]);
            cost += error * error;
            result.max_error = std::max(result.max_error, error);
        }
        result.per_view_rms.push_back(std::sqrt(cost / (rows * cols)));
        size_t offset = 9 + 6 * i;
        CameraPose pose;
        pose.rotation = rodrigues({best[offset], best[offset + 1], best[offset + 2]});
        // Report translation relative to the first inner corner (0,0), not
        // the centered coordinates used internally for conditioning.
        for (int j = 0; j < 3; ++j) {
            double t = j == 2 ? std::exp(best[offset + 5]) : best[offset + 3 + j];
            pose.translation[j] =
                (t - pose.rotation[j * 3] * (cols - 1) * .5 - pose.rotation[j * 3 + 1] * (rows - 1) * .5) *
                square_size;
        }
        result.poses.push_back(pose);
    }
    double max_angle = 0;
    for (size_t i = 0; i < result.poses.size(); ++i)
        for (size_t j = 0; j < i; ++j) {
            auto a = result.poses[i].rotation, b = result.poses[j].rotation;
            double cosine = std::fabs(a[2] * b[2] + a[5] * b[5] + a[8] * b[8]);
            max_angle = std::max(max_angle, std::acos(std::clamp(cosine, 0., 1.)));
        }
    result.weak_geometry = max_angle < .17 || points.size() < 5;
    bool intrinsics_ok = k.fx > .05 * width && k.fy > .05 * width && k.fx < 20 * width && k.fy < 20 * width &&
                         k.cx >= 0 && k.cx < width && k.cy >= 0 && k.cy < height;
    k.valid = best_converged && intrinsics_ok && result.rms < 3 && mapping_is_regular(k, width, height) &&
              max_angle > .01;
    if (!k.valid)
        result.message = "Unreliable calibration: check convergence, intrinsics, reprojection error and map "
                         "folding. No correction should be applied.";
    else if (result.weak_geometry)
        result.message = "Only a few views or small pose variation: provisional parameters; low training RMS "
                         "does not prove accuracy outside the board.";
    else
        result.message = "Calibration converged.";
}
} // namespace calibration
