#include "calibrate.h"
#include "calibration/internal.h"
#include <cmath>

using namespace calibration;

CameraCalibrationResult calibrate_camera(const std::vector<std::vector<Point2f>>& points, int width,
                                         int height, int rows, int cols, double square_size) {
    CameraCalibrationResult result;
    if (points.size() < 3 || points.size() > 100 || width < 2 || height < 2 || rows < 3 || cols < 3 ||
        rows > 100 || cols > 100 || !std::isfinite(square_size) || square_size <= 0) {
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
    // One rough intrinsic seed. Pose initialization remains essential for LM.
    const std::array<double, 4> intrinsics{
        double(width), double(width), (width - 1) * .5, (height - 1) * .5};
    State state;
    if (!initialize(hom, intrinsics, width, height, state)) {
        result.message = "Pose initialization failed.";
        return result;
    }
    // k3 stays zero; all views share the other eight camera parameters.
    int iterations = 0;
    bool converged = optimize(state, points, width, height, rows, cols,
                              iterations, result.work);
    std::vector<double> residual;
    double cost = residuals(state, points, width, height, rows, cols, residual);
    ++result.work.residual_passes;
    if (!std::isfinite(cost)) {
        result.message = "Calibration optimization failed.";
        return result;
    }
    finish_result(state, points, width, height, rows, cols, square_size, cost,
                  converged, iterations, result, residual);
    return result;
}
