#include "subpixel.h"
#include "../kernels/interpolation.h"
#include "../kernels/gradient.h"
#include <algorithm>
#include <cmath>

namespace {
float sample(const GrayImage& img, float x, float y) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y);
    float dx = x - ix, dy = y - iy;
    return kernels::bilinear(img.get(ix, iy), img.get(ix + 1, iy), img.get(ix, iy + 1),
                             img.get(ix + 1, iy + 1), dx, dy);
}
} // namespace

// Iterative, Gaussian-weighted gradient intersection, in local coordinates.
// Gradient magnitude is retained so weak/noisy edges are not overweighted.
void refine_subpixel(const GrayImage& img, std::vector<Point2f>& corners, int half_win) {
    const int radius = std::clamp(half_win, 2, 15);
    if (!img.data || img.w < 2 * radius + 5 || img.h < 2 * radius + 5)
        return;
    for (auto& corner : corners) {
        const Point2f original = corner;
        if (!std::isfinite(original.x) || !std::isfinite(original.y))
            continue;
        Point2f p = original;
        bool reliable = false;
        for (int iter = 0; iter < 40; ++iter) {
            if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 2 ||
                p.y >= img.h - radius - 2) {
                reliable = false;
                break;
            }
            double a = 0, b = 0, c = 0, bx = 0, by = 0;
            for (int y = -radius; y <= radius; ++y) {
                for (int x = -radius; x <= radius; ++x) {
                    float sx = p.x + x, sy = p.y + y;
                    double gx = sample(img, sx + 1, sy) - sample(img, sx - 1, sy);
                    double gy = sample(img, sx, sy + 1) - sample(img, sx, sy - 1);
                    double w = std::exp(-double(x * x + y * y) / (radius * radius));
                    auto tensor = kernels::outer_product(gx, gy, w);
                    double xx = tensor.xx, xy = tensor.xy, yy = tensor.yy;
                    a += xx;
                    b += xy;
                    c += yy;
                    bx += xx * x + xy * y;
                    by += xy * x + yy * y;
                }
            }
            double dx, dy;
            if (!kernels::solve_tensor(a, b, c, bx, by, dx, dy)) {
                reliable = false;
                break;
            }
            Point2f next{float(p.x + dx), float(p.y + dy)};
            if (!std::isfinite(next.x) || !std::isfinite(next.y) ||
                std::hypot(next.x - original.x, next.y - original.y) > radius) {
                reliable = false;
                break;
            }
            p = next;
            if (dx * dx + dy * dy < 1e-6) {
                reliable = true;
                break;
            }
        }
        // Failed/ill-conditioned fits retain the initial point. No centroid bias.
        if (reliable)
            corner = p;
    }
}
