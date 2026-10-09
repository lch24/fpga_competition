#include "internal.h"
#include "../../kernels/distortion.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace calibration {
namespace {
// Shared 8x8 camera block; each view adds 8x6 cross and 6x6 pose blocks.
struct Blocks {
    std::array<double, 64> a{};
    std::array<double, 8> g{};
    struct View {
        std::array<double, 48> b{};
        std::array<double, 36> c{};
        std::array<double, 6> g{};
    };
    std::vector<View> views;
    explicit Blocks(int n) : views(n) {}
};
struct Prepared {
    State p;
    M3 r;
    int w, h, off;
    double fx, fy, tz;
    Prepared(const State& s, int width, int height, int view) : p(s), w(width), h(height), off(9 + 6 * view) {
        r = rodrigues({p[off], p[off + 1], p[off + 2]});
        fx = std::exp(p[0]);
        fy = std::exp(p[1]);
        tz = std::exp(p[off + 5]);
    }
    std::array<double, 2> project(double X, double Y) const {
        double z = r[6] * X + r[7] * Y + tz;
        double x = (r[0] * X + r[1] * Y + p[off + 3]) / z, y = (r[3] * X + r[4] * Y + p[off + 4]) / z;
        auto d = kernels::distort(x, y, kernels::Distortion<double>{p[4], p[5], p[8], p[6], p[7]});
        return {fx * d.x + p[2] * w, fy * d.y + p[3] * h};
    }
    // Analytic intrinsics/distortion/translation; rotation still finite difference.
    void analytic(double X, double Y, double j[2][14]) const {
        double z = r[6] * X + r[7] * Y + tz, x = (r[0] * X + r[1] * Y + p[off + 3]) / z,
               y = (r[3] * X + r[4] * Y + p[off + 4]) / z;
        double rr = x * x + y * y, rad = 1 + p[4] * rr + p[5] * rr * rr, dr = p[4] + 2 * p[5] * rr;
        auto d = kernels::distort(x, y, kernels::Distortion<double>{p[4], p[5], 0, p[6], p[7]});
        j[0][0] = fx * d.x;
        j[1][1] = fy * d.y;
        j[0][2] = w;
        j[1][3] = h;
        j[0][4] = fx * x * rr;
        j[1][4] = fy * y * rr;
        j[0][5] = fx * x * rr * rr;
        j[1][5] = fy * y * rr * rr;
        j[0][6] = fx * 2 * x * y;
        j[1][6] = fy * (rr + 2 * y * y);
        j[0][7] = fx * (rr + 2 * x * x);
        j[1][7] = fy * 2 * x * y;
        double a = rad + 2 * x * x * dr + 2 * p[6] * y + 6 * p[7] * x,
               b = 2 * x * y * dr + 2 * p[6] * x + 2 * p[7] * y;
        double c = rad + 2 * y * y * dr + 6 * p[6] * y + 2 * p[7] * x;
        j[0][11] = fx * a / z;
        j[1][11] = fy * b / z;
        j[0][12] = fx * b / z;
        j[1][12] = fy * c / z;
        j[0][13] = -fx * (a * x + b * y) * tz / z;
        j[1][13] = -fy * (b * x + c * y) * tz / z;
    }
};
bool solve_many(std::vector<double> a, std::vector<double>& b, int n, int nrhs) {
    for (int k = 0; k < n; ++k) {
        int pivot = k;
        for (int i = k + 1; i < n; ++i)
            if (std::fabs(a[i * n + k]) > std::fabs(a[pivot * n + k]))
                pivot = i;
        if (!std::isfinite(a[pivot * n + k]) || std::fabs(a[pivot * n + k]) < 1e-25)
            return false;
        for (int j = k; j < n; ++j)
            std::swap(a[k * n + j], a[pivot * n + j]);
        for (int j = 0; j < nrhs; ++j)
            std::swap(b[k * nrhs + j], b[pivot * nrhs + j]);
        for (int i = k + 1; i < n; ++i) {
            double f = a[i * n + k] / a[k * n + k];
            for (int j = k; j < n; ++j)
                a[i * n + j] -= f * a[k * n + j];
            for (int j = 0; j < nrhs; ++j)
                b[i * nrhs + j] -= f * b[k * nrhs + j];
        }
    }
    for (int i = n - 1; i >= 0; --i)
        for (int j = 0; j < nrhs; ++j) {
            double x = b[i * nrhs + j];
            for (int k = i + 1; k < n; ++k)
                x -= a[i * n + k] * b[k * nrhs + j];
            b[i * nrhs + j] = x / a[i * n + i];
            if (!std::isfinite(b[i * nrhs + j]))
                return false;
        }
    return true;
}
bool step(const Blocks& blocks, double lambda, const std::vector<double>& scales,
          std::vector<double>& delta) {
    int views = int(blocks.views.size()), n = 8 + 6 * views;
    std::vector<double> a(64), g(8);
    std::vector<std::vector<double>> cb(views);
    for (int i = 0; i < 8; ++i) {
        g[i] = -blocks.g[i] * scales[i];
        for (int j = 0; j < 8; ++j)
            a[i * 8 + j] = double(blocks.a[i * 8 + j] * scales[i] * scales[j] + (i == j ? lambda : 0));
    }
    for (int v = 0; v < views; ++v) {
        const auto& b = blocks.views[v];
        std::vector<double> c(36), rhs(54);
        std::array<double, 48> cross{};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j)
                c[i * 6 + j] = double(b.c[i * 6 + j] * scales[8 + 6 * v + i] * scales[8 + 6 * v + j] +
                                      (i == j ? lambda : 0));
            for (int j = 0; j < 8; ++j) {
                rhs[i * 9 + j] = b.b[j * 6 + i] * scales[j] * scales[8 + 6 * v + i];
                cross[j * 6 + i] = rhs[i * 9 + j];
            }
            rhs[i * 9 + 8] = -b.g[i] * scales[8 + 6 * v + i];
        }
        if (!solve_many(c, rhs, 6, 9))
            return false;
        cb[v] = rhs;
        for (int i = 0; i < 8; ++i)
            for (int k = 0; k < 6; ++k) {
                g[i] -= cross[i * 6 + k] * rhs[k * 9 + 8];
                for (int j = 0; j < 8; ++j)
                    a[i * 8 + j] -= cross[i * 6 + k] * rhs[k * 9 + j];
            }
    }
    if (!solve_many(a, g, 8, 1))
        return false;
    delta.resize(n);
    for (int i = 0; i < 8; ++i)
        delta[i] = g[i];
    for (int v = 0; v < views; ++v)
        for (int i = 0; i < 6; ++i) {
            double d = cb[v][i * 9 + 8];
            for (int j = 0; j < 8; ++j)
                d -= cb[v][i * 9 + j] * g[j];
            delta[8 + 6 * v + i] = d;
        }
    for (int i = 0; i < n; ++i)
        delta[i] *= scales[i];
    return true;
}
} // namespace

// One point contributes only to shared camera parameters and its own pose.
// Accumulate these blocks directly; never materialize the full Jacobian.
bool optimize(State& p, const Points& points, int w, int h, int rows, int cols, int& accepted,
              CalibrationWork& work) {
    std::vector<int> ids{0, 1, 2, 3, 4, 5, 6, 7};
    for (int i = 9; i < int(p.size()); ++i)
        ids.push_back(i);
    int n = int(ids.size());
    work.max_active = n;
    bool converged = false;
    double lambda = 1e-3;
    std::vector<double> residual;
    double cost = residuals(p, points, w, h, rows, cols, residual);
    ++work.residual_passes;
    for (int iteration = 0; iteration < 60; ++iteration) {
        if (!std::isfinite(cost))
            return false;
        if (cost < 1e-16) {
            converged = true;
            break;
        }
        ++work.jacobians;
        Blocks normal(int(points.size()));
        for (int v = 0; v < int(points.size()); ++v) {
            Prepared base(p, w, h, v);
            std::vector<Prepared> perturbed;
            std::array<double, 14> eps{};
            for (int k = 8; k < 11; ++k) {
                int id = 9 + 6 * v + k - 8;
                State q = p;
                eps[k] = 1e-6 * (1 + std::fabs(p[id]));
                q[id] += eps[k];
                perturbed.emplace_back(q, w, h, v);
            }
            auto& vb = normal.views[v];
            for (int y = 0; y < rows; ++y)
                for (int x = 0; x < cols; ++x) {
                    double X = x - (cols - 1) * .5, Y = y - (rows - 1) * .5;
                    auto pred = base.project(X, Y);
                    double jac[2][14]{};
                    base.analytic(X, Y, jac);
                    for (int k = 8; k < 11; ++k) {
                        auto q = perturbed[k - 8].project(X, Y);
                        for (int axis = 0; axis < 2; ++axis)
                            jac[axis][k] = (q[axis] - pred[axis]) / eps[k];
                    }
                    for (int axis = 0; axis < 2; ++axis) {
                        double r = double(residual[2 * (v * rows * cols + y * cols + x) + axis]);
                        std::array<double, 14> j{};
                        for (int k = 0; k < 14; ++k)
                            j[k] = jac[axis][k];
                        for (int i = 0; i < 8; ++i) {
                            normal.g[i] += j[i] * r;
                            for (int k = 0; k < 8; ++k)
                                normal.a[i * 8 + k] += j[i] * j[k];
                            for (int k = 0; k < 6; ++k)
                                vb.b[i * 6 + k] += j[i] * j[8 + k];
                        }
                        for (int i = 0; i < 6; ++i) {
                            vb.g[i] += j[8 + i] * r;
                            for (int k = 0; k < 6; ++k)
                                vb.c[i * 6 + k] += j[8 + i] * j[8 + k];
                        }
                    }
                }
        }
        std::vector<double> scales(n);
        double maxg = 0;
        for (int i = 0; i < n; ++i) {
            double diag =
                i < 8 ? normal.a[i * 8 + i] : double(normal.views[(i - 8) / 6].c[((i - 8) % 6) * 7]);
            scales[i] = 1 / std::max(std::sqrt(std::max(0., diag)), 1e-12);
            double g = i < 8 ? normal.g[i] : double(normal.views[(i - 8) / 6].g[(i - 8) % 6]);
            maxg = std::max(maxg, std::fabs(g * scales[i]));
        }
        if (maxg < 1e-8 * (1 + std::sqrt(cost))) {
            converged = true;
            break;
        }
        bool success = false;
        for (int attempt = 0; attempt < 8; ++attempt) {
            std::vector<double> delta;
            ++work.linear_solves;
            if (!step(normal, lambda, scales, delta)) {
                lambda *= 10;
                continue;
            }
            State q = p;
            double norm = 0;
            for (int i = 0; i < n; ++i) {
                q[ids[i]] += delta[i];
                norm = std::max(norm, std::fabs(delta[i]) / (1 + std::fabs(p[ids[i]])));
            }
            std::vector<double> trial;
            double nc = residuals(q, points, w, h, rows, cols, trial);
            ++work.residual_passes;
            if (nc < cost) {
                double reduction = cost - nc;
                cost = nc;
                p = std::move(q);
                residual = std::move(trial);
                ++accepted;
                lambda = std::max(lambda * .3, 1e-12);
                success = true;
                if (norm < 1e-9 || reduction < 1e-7 * (1 + cost))
                    converged = true;
                break;
            }
            lambda *= 10;
        }
        if (!success) {
            converged = maxg < 1e-5 * (1 + std::sqrt(cost));
            break;
        }
        if (converged)
            break;
    }
    return converged;
}
} // namespace calibration
