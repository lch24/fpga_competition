#define _CRT_SECURE_NO_WARNINGS
#include "calibrate.h"
#include "../common/matrix.h"
#include <cmath>
#include <cstdio>
#include <vector>

f64 radial_distort(f64 x_n, f64 y_n, const CalibDistort& d) {
    f64 r2 = x_n * x_n + y_n * y_n;
    f64 r4 = r2 * r2;
    f64 r6 = r4 * r2;
    return 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;
}

void project_distorted(f64 x_ideal, f64 y_ideal,
                       f64 fx, f64 fy, f64 cx, f64 cy,
                       const CalibDistort& d,
                       f64& x_d, f64& y_d) {
    f64 x_n = (x_ideal - cx) / fx;
    f64 y_n = (y_ideal - cy) / fy;
    f64 r2 = x_n * x_n + y_n * y_n;
    f64 r4 = r2 * r2;
    f64 r6 = r4 * r2;
    f64 radial = 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;
    f64 xt = 2.0 * d.p1 * x_n * y_n + d.p2 * (r2 + 2.0 * x_n * x_n);
    f64 yt = d.p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * d.p2 * x_n * y_n;
    f64 x_dn = x_n * radial + xt;
    f64 y_dn = y_n * radial + yt;
    x_d = x_dn * fx + cx;
    y_d = y_dn * fy + cy;
}

static f64 compute_cost_all(const CalibDistort& d,
                            const std::vector<Point2f>& image_pts,
                            const std::vector<Point2f>& ideal_pts,
                            f64 fx, f64 fy, f64 cx, f64 cy) {
    int N = (int)image_pts.size();
    f64 s = 0;
    for (int i = 0; i < N; ++i) {
        f64 xd, yd;
        project_distorted(ideal_pts[i].x, ideal_pts[i].y, fx, fy, cx, cy, d, xd, yd);
        f64 dx = (f64)image_pts[i].x - xd;
        f64 dy = (f64)image_pts[i].y - yd;
        s += dx * dx + dy * dy;
    }
    return s;
}

static f64 param_get(const CalibDistort& d, int idx) {
    switch (idx) {
        case 0: return d.k1;
        case 1: return d.k2;
        case 2: return d.p1;
        case 3: return d.p2;
        case 4: return d.k3;
        default: return 0;
    }
}
static void param_set(CalibDistort& d, int idx, f64 v) {
    switch (idx) {
        case 0: d.k1 = v; break;
        case 1: d.k2 = v; break;
        case 2: d.p1 = v; break;
        case 3: d.p2 = v; break;
        case 4: d.k3 = v; break;
    }
}

static void compute_full_jacobian(int N, int NP,
                                  const std::vector<Point2f>& image_pts,
                                  const std::vector<Point2f>& ideal_pts,
                                  f64 fx, f64 fy, f64 cx, f64 cy,
                                  const CalibDistort& d,
                                  LMtx& J, LMtx& r_vec) {
    for (int i = 0; i < N; ++i) {
        f64 x_n = ((f64)ideal_pts[i].x - cx) / fx;
        f64 y_n = ((f64)ideal_pts[i].y - cy) / fy;
        f64 r2 = x_n * x_n + y_n * y_n;
        f64 r4 = r2 * r2;
        f64 r6 = r4 * r2;

        f64 d_n  = fx * x_n;
        f64 d_nv = fy * y_n;

        f64 dxt_dp1 = 2.0 * x_n * y_n;
        f64 dxt_dp2 = r2 + 2.0 * x_n * x_n;
        f64 dyt_dp1 = r2 + 2.0 * y_n * y_n;
        f64 dyt_dp2 = 2.0 * x_n * y_n;

        f64 xd, yd;
        project_distorted(ideal_pts[i].x, ideal_pts[i].y, fx, fy, cx, cy, d, xd, yd);

        f64 rx = (f64)image_pts[i].x - xd;
        f64 ry = (f64)image_pts[i].y - yd;
        r_vec.at(2 * i, 0)     = rx;
        r_vec.at(2 * i + 1, 0) = ry;

        for (int k = 0; k < NP; ++k) {
            f64 jx = 0, jy = 0;
            switch (k) {
                case 0: jx = -d_n  * r2;    jy = -d_nv * r2;    break;
                case 1: jx = -d_n  * r4;    jy = -d_nv * r4;    break;
                case 2: jx = -fx * dxt_dp1; jy = -fy * dyt_dp1; break;
                case 3: jx = -fx * dxt_dp2; jy = -fy * dyt_dp2; break;
                case 4: jx = -d_n  * r6;    jy = -d_nv * r6;    break;
            }
            J.at(2 * i, k)     = jx;
            J.at(2 * i + 1, k) = jy;
        }
    }
}

static LMResult lm_solve_subset(
    CalibDistort state,
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx, f64 fy, f64 cx, f64 cy,
    const std::vector<int>& active,
    int max_iter, f64 tol, bool verbose
) {
    int N = (int)image_pts.size();
    int NP = (int)active.size();

    LMResult res;
    res.d = state;
    res.converged = false;
    res.iterations = 0;

    if (N == 0 || NP == 0) return res;

    auto compute_cost = [&](const CalibDistort& d) -> f64 {
        return compute_cost_all(d, image_pts, ideal_pts, fx, fy, cx, cy);
    };

    f64 cost = compute_cost(state);
    res.final_error = cost;

    static const f64 full_scale[5] = {0.1, 0.01, 0.01, 0.01, 0.001};
    std::vector<f64> scale(NP);
    for (int k = 0; k < NP; ++k)
        scale[k] = full_scale[active[k]];

    LMtx J_full(2 * N, 5);
    LMtx r_full(2 * N, 1);
    LMtx J(2 * N, NP);
    LMtx Js(2 * N, NP);

    f64 lambda = 1e-3;

    for (int iter = 0; iter < max_iter; ++iter) {
        compute_full_jacobian(N, 5, image_pts, ideal_pts, fx, fy, cx, cy, state, J_full, r_full);

        for (int i = 0; i < 2 * N; ++i)
            for (int k = 0; k < NP; ++k)
                J.at(i, k) = J_full.at(i, active[k]);

        for (int i = 0; i < 2 * N; ++i)
            for (int k = 0; k < NP; ++k)
                Js.at(i, k) = J.at(i, k) * scale[k];

        LMtx JtJ = Js.T() * Js;
        LMtx Jtr = Js.T() * r_full;

        f64 max_jj = 0;
        for (int k = 0; k < NP; ++k)
            if (JtJ.at(k, k) > max_jj) max_jj = JtJ.at(k, k);
        if (max_jj < 1e-12) max_jj = 1e-12;

        f64 lambda_abs = lambda * max_jj;
        for (int k = 0; k < NP; ++k)
            JtJ.at(k, k) *= (1.0 + lambda_abs);

        LMtx delta_s = solve_gauss(std::move(JtJ), Jtr * -1.0);

        if (verbose && iter < 3) {
            std::printf("    [iter %2d] cost=%.4f rms=%.4f lambda=%.2e",
                iter, cost, std::sqrt(cost / (2 * N)), lambda_abs);
        }

        std::vector<f64> delta_p(NP);
        for (int k = 0; k < NP; ++k)
            delta_p[k] = delta_s.at(k, 0) * scale[k];

        if (verbose && iter < 3) {
            std::printf(" dp=(");
            for (int k = 0; k < NP; ++k) {
                if (k) std::printf(",");
                std::printf("%.5f", delta_p[k]);
            }
            std::printf(")\n");
        }

        f64 best_cost = cost;
        f64 best_alpha = 0;
        CalibDistort trial_best = state;

        for (int li = 0; li < 10; ++li) {
            f64 alpha = 1.0 / (1 << li);
            CalibDistort trial = state;
            for (int k = 0; k < NP; ++k)
                param_set(trial, active[k], param_get(state, active[k]) + alpha * delta_p[k]);

            f64 ct = compute_cost(trial);
            if (ct < best_cost) {
                best_cost = ct;
                best_alpha = alpha;
                trial_best = trial;
            }
        }

        if (best_alpha > 0) {
            state = trial_best;
            cost = best_cost;
            lambda *= 0.5;
            res.iterations = iter + 1;

            f64 step_norm = 0;
            for (int k = 0; k < NP; ++k) {
                f64 rel = best_alpha * delta_p[k] / scale[k];
                step_norm += rel * rel;
            }
            step_norm = std::sqrt(step_norm);

            if (verbose && iter < 3) {
                std::printf("        ACCEPT alpha=%.4f new_rms=%.4f\n",
                    best_alpha, std::sqrt(cost / (2 * N)));
            }

            if (step_norm < tol) {
                res.converged = true;
                break;
            }
        } else {
            lambda *= 2.0;
            if (lambda > 1e12) break;
            if (verbose && iter < 3) {
                std::printf("        REJECT lambda->%.2e\n", lambda);
            }
        }
    }

    res.d = state;
    res.final_error = cost;
    return res;
}

LMResult lm_calibrate_distort(
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx, f64 fy, f64 cx, f64 cy,
    const CalibDistort& init,
    int max_iter,
    f64 tol
) {
    (void)max_iter;
    (void)tol;

    LMResult res;
    int N = (int)image_pts.size();
    if (N == 0) return res;

    f64 cost0 = compute_cost_all(init, image_pts, ideal_pts, fx, fy, cx, cy);
    f64 rms0 = std::sqrt(cost0 / (2 * N));
    std::printf("  [stage 0] init rms=%.6f\n", rms0);

    if (rms0 < 1e-6) {
        res.d = init;
        res.final_error = cost0;
        res.converged = true;
        res.iterations = 0;
        std::printf("  [stage 0] already near optimal, skip\n");
        return res;
    }

    CalibDistort state = init;

    auto run_stage = [&](const char* name, std::vector<int> active, int iters) {
        std::printf("  [stage %s] active params:", name);
        for (int k : active) {
            const char* nm[] = {"k1", "k2", "p1", "p2", "k3"};
            std::printf(" %s", nm[k]);
        }
        std::printf("\n");

        LMResult sub = lm_solve_subset(
            state, image_pts, ideal_pts, fx, fy, cx, cy,
            active, iters, 1e-8, true
        );
        state = sub.d;
        f64 rms = std::sqrt(sub.final_error / (2 * N));
        std::printf("    -> rms=%.6f iters=%d\n", rms, sub.iterations);
    };

    run_stage("1", {0},                         200);
    run_stage("2", {0, 1},                      200);
    run_stage("3", {0, 1, 2, 3},                200);
    run_stage("4", {0, 1, 2, 3, 4},             500);

    res.d = state;
    res.final_error = compute_cost_all(state, image_pts, ideal_pts, fx, fy, cx, cy);
    f64 rms_final = std::sqrt(res.final_error / (2 * N));
    std::printf("  [final] rms=%.6f\n", rms_final);

    return res;
}