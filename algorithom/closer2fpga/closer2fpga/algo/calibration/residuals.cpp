#include "internal.h"
#include "../../kernels/distortion.h"
#include <cmath>

namespace calibration {
double residuals(const State& p, const std::vector<std::vector<Point2f>>& points, int w, int h, int rows,
                 int cols, std::vector<double>& residual) {
    for (double v : p)
        if (!std::isfinite(v))
            return infinity;
    double fx = std::exp(p[0]), fy = std::exp(p[1]), cx = p[2] * w, cy = p[3] * h;
    if (fx < 1e-3 || fy < 1e-3 || fx > 1e7 || fy > 1e7)
        return infinity;
    residual.resize(points.size() * rows * cols * 2);
    double cost = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        size_t offset = 9 + 6 * i;
        M3 r = rodrigues({p[offset], p[offset + 1], p[offset + 2]});
        double tz = std::exp(p[offset + 5]);
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < cols; ++x) {
                double X = x - (cols - 1) * .5, Y = y - (rows - 1) * .5;
                double z = r[6] * X + r[7] * Y + tz;
                if (z <= 1e-5)
                    return infinity;
                double nx = (r[0] * X + r[1] * Y + p[offset + 3]) / z;
                double ny = (r[3] * X + r[4] * Y + p[offset + 4]) / z;
                auto distorted =
                    kernels::distort(nx, ny, kernels::Distortion<double>{p[4], p[5], p[8], p[6], p[7]});
                double xd = distorted.x, yd = distorted.y;
                size_t id = 2 * (i * rows * cols + y * cols + x);
                double dx = fx * xd + cx - points[i][y * cols + x].x;
                double dy = fy * yd + cy - points[i][y * cols + x].y;
                residual[id] = dx;
                residual[id + 1] = dy;
                cost += dx * dx + dy * dy;
            }
    }
    return std::isfinite(cost) ? cost : infinity;
}
} // namespace calibration
