#include "internal.h"
#include "../../common/matrix.h"
#include <algorithm>
#include <cmath>

namespace calibration {
bool optimize(State& p, const std::vector<std::vector<Point2f>>& points, int w, int h, int rows, int cols,
              const std::vector<int>& active, int limit, int& iterations) {
    std::vector<double> r;
    double cost = residuals(p, points, w, h, rows, cols, r), lambda = 1e-3;
    int n = int(active.size()), m = int(r.size());
    if (!std::isfinite(cost))
        return false;
    for (int it = 0; it < limit; ++it) {
        // JACOBIAN: perturb one parameter, evaluate two full residual vectors.
        // Each column must be complete before its norm/scale is available.
        if (cost < 1e-16)
            return true;
        std::vector<double> j(m * n), scales(n), plus, minus;
        for (int k = 0; k < n; ++k) {
            State q = p;
            double step = 1e-6 * (1 + std::fabs(p[active[k]]));
            q[active[k]] += step;
            if (!std::isfinite(residuals(q, points, w, h, rows, cols, plus)))
                return false;
            q[active[k]] -= 2 * step;
            if (!std::isfinite(residuals(q, points, w, h, rows, cols, minus)))
                return false;
            double norm = 0;
            for (int t = 0; t < m; ++t) {
                double v = (plus[t] - minus[t]) / (2 * step);
                j[t * n + k] = v;
                norm += v * v;
            }
            scales[k] = 1 / std::max(std::sqrt(norm), 1e-12);
            for (int t = 0; t < m; ++t)
                j[t * n + k] *= scales[k];
        }
        std::vector<double> normal(n * n, 0), gradient(n, 0);
        // NORMAL EQUATIONS: RAM accumulators have read/modify/write dependencies.
        for (int t = 0; t < m; ++t)
            for (int a = 0; a < n; ++a) {
                gradient[a] += j[t * n + a] * r[t];
                for (int b = 0; b <= a; ++b)
                    normal[a * n + b] += j[t * n + a] * j[t * n + b];
            }
        double max_gradient = 0;
        for (double g : gradient)
            max_gradient = std::max(max_gradient, std::fabs(g));
        if (max_gradient < 1e-8 * (1 + std::sqrt(cost)))
            return true;
        bool accepted = false;
        // DAMPING SEARCH: reuse the normal equations until a step is accepted.
        // A rejected trial must never overwrite the current parameter state.
        for (int attempt = 0; attempt < 16; ++attempt) {
            LMtx a(n, n), b(n, 1);
            for (int x = 0; x < n; ++x) {
                b.at(x, 0) = -gradient[x];
                for (int y = 0; y < n; ++y)
                    a.at(x, y) = x >= y ? normal[x * n + y] : normal[y * n + x];
                a.at(x, x) += lambda;
            }
            auto delta = solve_gauss(std::move(a), std::move(b));
            if (delta.rows != n) {
                lambda *= 10;
                continue;
            }
            State q = p;
            double step_norm = 0;
            for (int k = 0; k < n; ++k) {
                double d = delta.at(k, 0) * scales[k];
                q[active[k]] += d;
                step_norm = std::max(step_norm, std::fabs(d) / (1 + std::fabs(p[active[k]])));
            }
            std::vector<double> trial;
            double new_cost = residuals(q, points, w, h, rows, cols, trial);
            if (new_cost < cost) {
                double reduction = cost - new_cost;
                p = std::move(q);
                r = std::move(trial);
                cost = new_cost;
                ++iterations;
                lambda = std::max(lambda * .3, 1e-12);
                accepted = true;
                if (step_norm < 1e-9 || reduction < 1e-11 * (1 + cost))
                    return true;
                break;
            }
            lambda *= 10;
        }
        if (!accepted)
            return max_gradient < 1e-5 * (1 + std::sqrt(cost));
    }
    return false;
}
} // namespace calibration
