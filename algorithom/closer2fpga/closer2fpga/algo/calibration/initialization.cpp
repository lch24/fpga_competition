#include "internal.h"
#include "../../common/symmetric_eigen.h"
#include <cmath>

namespace calibration {
using linalg::accumulate_outer;
using linalg::eigen_symmetric;
bool homography(const std::vector<Point2f>& points, int rows, int cols, M3& h) {
    // Object coordinates are centered and expressed in board-square units.
    double mx = 0, my = 0;
    for (auto p : points) {
        mx += p.x;
        my += p.y;
    }
    mx /= points.size();
    my /= points.size();
    double di = 0, dw = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            auto p = points[r * cols + c];
            di += std::hypot(p.x - mx, p.y - my);
            dw += std::hypot(c - (cols - 1) * .5, r - (rows - 1) * .5);
        }
    if (di < 1e-6 || dw < 1e-6)
        return false;
    double si = std::sqrt(2.) * points.size() / di, sw = std::sqrt(2.) * points.size() / dw;
    std::vector<double> ata(81, 0), eval, evec;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            auto p = points[r * cols + c];
            double x = (c - (cols - 1) * .5) * sw, y = (r - (rows - 1) * .5) * sw;
            double u = (p.x - mx) * si, v = (p.y - my) * si;
            accumulate_outer(ata, {-x, -y, -1, 0, 0, 0, u * x, u * y, u});
            accumulate_outer(ata, {0, 0, 0, -x, -y, -1, v * x, v * y, v});
        }
    if (!eigen_symmetric(ata, 9, eval, evec) || eval[1] < eval[8] * 1e-10)
        return false;
    M3 normalized{};
    for (int k = 0; k < 9; ++k)
        normalized[k] = evec[k * 9];
    h = multiply(multiply({1 / si, 0, mx, 0, 1 / si, my, 0, 0, 1}, normalized),
                 {sw, 0, 0, 0, sw, 0, 0, 0, 1});
    if (std::fabs(h[8]) < 1e-12)
        return false;
    double scale = h[8];
    for (auto& v : h)
        v /= scale;
    return true;
}

static std::vector<double> vij(const M3& h, int i, int j) {
    return {h[i] * h[j],
            h[i] * h[3 + j] + h[3 + i] * h[j],
            h[3 + i] * h[3 + j],
            h[6 + i] * h[j] + h[i] * h[6 + j],
            h[6 + i] * h[3 + j] + h[3 + i] * h[6 + j],
            h[6 + i] * h[6 + j]};
}

bool zhang_intrinsics(const std::vector<M3>& homographies, int w, int h, std::array<double, 4>& k) {
    // Normalize image coordinates before forming the conic constraints to avoid
    // mixing pixel^4 terms with order-one entries in the eigensystem.
    std::vector<double> ata(36, 0), eval, evec;
    M3 t{1. / w, 0, -.5, 0, 1. / w, -double(h) / (2 * w), 0, 0, 1};
    for (const auto& hom : homographies) {
        M3 a = multiply(t, hom);
        auto v12 = vij(a, 0, 1), v11 = vij(a, 0, 0), v22 = vij(a, 1, 1);
        for (int i = 0; i < 6; ++i)
            v11[i] -= v22[i];
        accumulate_outer(ata, v12);
        accumulate_outer(ata, v11);
    }
    if (!eigen_symmetric(ata, 6, eval, evec))
        return false;
    double b[6];
    for (int i = 0; i < 6; ++i)
        b[i] = evec[i * 6];
    if (b[0] < 0)
        for (auto& v : b)
            v = -v;
    double denominator = b[0] * b[2] - b[1] * b[1];
    if (b[0] <= 0 || denominator <= 1e-14)
        return false;
    double cy = (b[1] * b[3] - b[0] * b[4]) / denominator;
    double lambda = b[5] - (b[3] * b[3] + cy * (b[1] * b[3] - b[0] * b[4])) / b[0];
    if (lambda <= 0)
        return false;
    double fx = std::sqrt(lambda / b[0]), fy = std::sqrt(lambda * b[0] / denominator);
    double skew = -b[1] * fx * fx * fy / lambda;
    double cx = skew * cy / fy - b[3] * fx * fx / lambda;
    k = {fx * w, fy * w, cx * w + w * .5, cy * w + h * .5};
    return std::isfinite(k[0]) && std::isfinite(k[1]) && k[0] > .05 * w && k[1] > .05 * w && k[0] < 20 * w &&
           k[1] < 20 * w && std::fabs(k[2] - w * .5) < w && std::fabs(k[3] - h * .5) < h;
}
} // namespace calibration
