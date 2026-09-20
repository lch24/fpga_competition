#include "calibrate.h"
#include "calibration/internal.h"
#include <cmath>

using namespace calibration;

CameraCalibrationResult calibrate_camera(const std::vector<std::vector<Point2f>>& points, int width,
                                         int height, int rows, int cols, double square_size,
                                         const CameraCalibrationOptions& options) {
    CameraCalibrationResult result;
    result.k3_estimated = options.estimate_k3;
    if (points.size() < 3 || points.size() > 100 || width < 2 || height < 2 || rows < 3 || cols < 3 ||
        rows > 100 || cols > 100 || !std::isfinite(square_size) || square_size <= 0 ||
        options.max_iterations < 1) {
        result.message = "Need at least three views, valid image/board dimensions and positive square size.";
        return result;
    }
    std::vector<M3> hom(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].size() != size_t(rows * cols)) {
            result.message = "Corner count mismatch.";
            return result;
        }
        for (auto p : points[i])
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= width ||
                p.y >= height) {
                result.message = "Non-finite or out-of-image corner.";
                return result;
            }
        if (!homography(points[i], rows, cols, hom[i])) {
            result.message = "Degenerate board geometry.";
            return result;
        }
    }
    // Reject repeated homographies; three copies of one observation are not
    // three independent calibration views, even though LM could fit them.
    double diversity = 0;
    for (size_t i = 1; i < hom.size(); ++i)
        for (int j = 0; j < 8; ++j)
            diversity += std::fabs(hom[i][j] - hom[0][j]);
    if (diversity < 1e-6) {
        result.message = "Repeated views do not constrain camera intrinsics.";
        return result;
    }
    std::vector<std::array<double, 4>> seeds;
    std::array<double, 4> zhang;
    if (zhang_intrinsics(hom, width, height, zhang))
        seeds.push_back(zhang);
    for (double factor : {.6, 1., 1.8, 3.})
        seeds.push_back({width * factor, width * factor, (width - 1) * .5, (height - 1) * .5});
    State best;
    double best_cost = infinity;
    bool best_converged = false;
    int best_iterations = 0;
    for (const auto& seed : seeds) {
        State state;
        if (!initialize(hom, seed, width, height, state))
            continue;
        std::vector<int> active{0, 1, 2, 3};
        for (int i = 9; i < int(state.size()); ++i)
            active.push_back(i);
        int iterations = 0;
        optimize(state, points, width, height, rows, cols, active, options.max_iterations, iterations);
        active.push_back(4);
        optimize(state, points, width, height, rows, cols, active, options.max_iterations, iterations);
        active.insert(active.end(), {5, 6, 7});
        bool converged =
            optimize(state, points, width, height, rows, cols, active, options.max_iterations, iterations);
        if (options.estimate_k3) {
            active.push_back(8);
            converged = optimize(state, points, width, height, rows, cols, active, options.max_iterations,
                                 iterations);
        }
        std::vector<double> residual;
        double cost = residuals(state, points, width, height, rows, cols, residual);
        if (cost < best_cost) {
            best_cost = cost;
            best = std::move(state);
            best_converged = converged;
            best_iterations = iterations;
        }
    }
    if (best.empty()) {
        result.message = "Calibration optimization failed.";
        return result;
    }
    finish_result(best, points, width, height, rows, cols, square_size, best_cost, best_converged,
                  best_iterations, result);
    return result;
}
