#include "internal.h"
#include "../../common/matrix.h"
#include <cmath>

namespace calibration {
// Normalized 8-variable least squares; no eigensolver is required.
static void accumulate_outer(std::vector<double>& a, const std::array<double, 9>& row) {
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) a[i * 9 + j] += row[i] * row[j];
}
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
    std::vector<double> ata(81, 0);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            auto p = points[r * cols + c];
            double x = (c - (cols - 1) * .5) * sw, y = (r - (rows - 1) * .5) * sw;
            double u = (p.x - mx) * si, v = (p.y - my) * si;
            accumulate_outer(ata, {-x, -y, -1, 0, 0, 0, u * x, u * y, u});
            accumulate_outer(ata, {0, 0, 0, -x, -y, -1, v * x, v * y, v});
        }
    M3 normalized{};
    {
        LMtx a(8, 8), b(8, 1);
        for (int i = 0; i < 8; ++i) {
            b.at(i, 0) = -ata[i * 9 + 8];
            for (int j = 0; j < 8; ++j) a.at(i, j) = ata[i * 9 + j];
        }
        auto x = solve_gauss(std::move(a), std::move(b));
        if (x.rows != 8) return false;
        for (int i = 0; i < 8; ++i) normalized[i] = x.at(i, 0);
        normalized[8] = 1;
    }
    h = multiply(multiply({1 / si, 0, mx, 0, 1 / si, my, 0, 0, 1}, normalized),
                 {sw, 0, 0, 0, sw, 0, 0, 0, 1});
    if (std::fabs(h[8]) < 1e-12)
        return false;
    double scale = h[8];
    for (auto& v : h)
        v /= scale;
    return true;
}

} // namespace calibration
