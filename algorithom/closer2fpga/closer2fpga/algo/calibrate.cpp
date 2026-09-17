#define _CRT_SECURE_NO_WARNINGS

#include "calibrate.h"
#include "../common/matrix.h"

#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>

// ============================================================
// Basic helpers
// ============================================================

static bool finite_f64(f64 v)
{
    return std::isfinite(v);
}

// ============================================================
// Radial distortion
// ============================================================

f64 radial_distort(
    f64 x_n,
    f64 y_n,
    const CalibDistort& d
)
{
    f64 r2 = x_n * x_n + y_n * y_n;
    f64 r4 = r2 * r2;
    f64 r6 = r4 * r2;

    return 1.0
        + d.k1 * r2
        + d.k2 * r4
        + d.k3 * r6;
}

// ============================================================
// Ideal -> distorted projection
// ============================================================

void project_distorted(
    f64 x_ideal,
    f64 y_ideal,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& d,
    f64& x_d,
    f64& y_d
)
{
    // Pixel -> normalized camera coordinates.
    f64 x_n = (x_ideal - cx) / fx;
    f64 y_n = (y_ideal - cy) / fy;

    f64 r2 = x_n * x_n + y_n * y_n;
    f64 r4 = r2 * r2;
    f64 r6 = r4 * r2;

    // Radial distortion.
    f64 radial =
        1.0
        + d.k1 * r2
        + d.k2 * r4
        + d.k3 * r6;

    // Tangential distortion.
    f64 xt =
        2.0 * d.p1 * x_n * y_n
        + d.p2 * (r2 + 2.0 * x_n * x_n);

    f64 yt =
        d.p1 * (r2 + 2.0 * y_n * y_n)
        + 2.0 * d.p2 * x_n * y_n;

    // Distorted normalized coordinates.
    f64 x_dn = x_n * radial + xt;
    f64 y_dn = y_n * radial + yt;

    // Normalized -> pixel.
    x_d = x_dn * fx + cx;
    y_d = y_dn * fy + cy;
}

// ============================================================
// Compute total squared reprojection cost
// ============================================================

static f64 compute_cost_all(
    const CalibDistort& d,
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy
)
{
    int N = (int)image_pts.size();

    if (N == 0 ||
        ideal_pts.size() != image_pts.size())
    {
        return 0.0;
    }

    f64 sum = 0.0;

    for (int i = 0; i < N; ++i)
    {
        f64 xd, yd;

        project_distorted(
            ideal_pts[i].x,
            ideal_pts[i].y,
            fx,
            fy,
            cx,
            cy,
            d,
            xd,
            yd
        );

        f64 dx =
            (f64)image_pts[i].x - xd;

        f64 dy =
            (f64)image_pts[i].y - yd;

        if (!finite_f64(dx) ||
            !finite_f64(dy))
        {
            return HUGE_VAL;
        }

        sum += dx * dx + dy * dy;

        if (!finite_f64(sum))
        {
            return HUGE_VAL;
        }
    }

    return sum;
}

// ============================================================
// Parameter access
// ============================================================

static f64 param_get(
    const CalibDistort& d,
    int idx
)
{
    switch (idx)
    {
    case 0: return d.k1;
    case 1: return d.k2;
    case 2: return d.p1;
    case 3: return d.p2;
    case 4: return d.k3;
    default: return 0.0;
    }
}

static void param_set(
    CalibDistort& d,
    int idx,
    f64 v
)
{
    switch (idx)
    {
    case 0: d.k1 = v; break;
    case 1: d.k2 = v; break;
    case 2: d.p1 = v; break;
    case 3: d.p2 = v; break;
    case 4: d.k3 = v; break;
    default: break;
    }
}

// ============================================================
// Compute full Jacobian
//
// residual:
//
//     r = observed - predicted
//
// Therefore Jacobian is:
//
//     dr / dp
//
// The derivatives below use the negative of the derivative
// of the predicted distorted point.
// ============================================================

static void compute_full_jacobian(
    int N,
    int NP,
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& d,
    LMtx& J,
    LMtx& r_vec
)
{
    for (int i = 0; i < N; ++i)
    {
        // Ideal pixel -> normalized coordinates.
        f64 x_n =
            ((f64)ideal_pts[i].x - cx) / fx;

        f64 y_n =
            ((f64)ideal_pts[i].y - cy) / fy;

        f64 r2 = x_n * x_n + y_n * y_n;
        f64 r4 = r2 * r2;
        f64 r6 = r4 * r2;

        // Pixel scale.
        f64 fx_x = fx * x_n;
        f64 fy_y = fy * y_n;

        // Tangential derivatives.
        f64 dxt_dp1 =
            2.0 * x_n * y_n;

        f64 dxt_dp2 =
            r2 + 2.0 * x_n * x_n;

        f64 dyt_dp1 =
            r2 + 2.0 * y_n * y_n;

        f64 dyt_dp2 =
            2.0 * x_n * y_n;

        // Current prediction.
        f64 xd, yd;

        project_distorted(
            ideal_pts[i].x,
            ideal_pts[i].y,
            fx,
            fy,
            cx,
            cy,
            d,
            xd,
            yd
        );

        // Residual = observed - predicted.
        f64 rx =
            (f64)image_pts[i].x - xd;

        f64 ry =
            (f64)image_pts[i].y - yd;

        r_vec.at(2 * i, 0) =
            rx;

        r_vec.at(2 * i + 1, 0) =
            ry;

        for (int k = 0; k < NP; ++k)
        {
            f64 jx = 0.0;
            f64 jy = 0.0;

            switch (k)
            {
                // k1
            case 0:
                jx = -fx_x * r2;
                jy = -fy_y * r2;
                break;

                // k2
            case 1:
                jx = -fx_x * r4;
                jy = -fy_y * r4;
                break;

                // p1
            case 2:
                jx = -fx * dxt_dp1;
                jy = -fy * dyt_dp1;
                break;

                // p2
            case 3:
                jx = -fx * dxt_dp2;
                jy = -fy * dyt_dp2;
                break;

                // k3
            case 4:
                jx = -fx_x * r6;
                jy = -fy_y * r6;
                break;
            }

            J.at(2 * i, k) =
                jx;

            J.at(2 * i + 1, k) =
                jy;
        }
    }
}

// ============================================================
// LM solver for a subset of parameters
// ============================================================

static LMResult lm_solve_subset(
    CalibDistort state,

    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,

    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,

    const std::vector<int>& active,

    int max_iter,
    f64 tol,

    bool verbose
)
{
    LMResult res;

    res.d = state;
    res.final_error = HUGE_VAL;
    res.iterations = 0;
    res.converged = false;

    int N =
        (int)image_pts.size();

    int NP =
        (int)active.size();

    if (N <= 0)
        return res;

    if ((int)ideal_pts.size() != N)
        return res;

    if (NP <= 0)
        return res;

    if (fx <= 0.0 ||
        fy <= 0.0)
    {
        return res;
    }

    if (max_iter <= 0)
        return res;

    if (tol <= 0.0)
        tol = 1e-8;

    // --------------------------------------------------------
    // Parameter scaling.
    // --------------------------------------------------------

    static const f64 full_scale[5] =
    {
        0.1,     // k1
        0.01,    // k2
        0.01,    // p1
        0.01,    // p2
        0.001    // k3
    };

    std::vector<f64> scale(NP);

    for (int k = 0; k < NP; ++k)
    {
        int p =
            active[k];

        if (p < 0 ||
            p >= 5)
        {
            return res;
        }

        scale[k] =
            full_scale[p];
    }

    // --------------------------------------------------------
    // Initial cost.
    // --------------------------------------------------------

    f64 cost =
        compute_cost_all(
            state,
            image_pts,
            ideal_pts,
            fx,
            fy,
            cx,
            cy
        );

    if (!finite_f64(cost))
        return res;

    res.final_error =
        cost;

    // --------------------------------------------------------
    // Allocate matrices.
    // --------------------------------------------------------

    LMtx J_full(
        2 * N,
        5
    );

    LMtx r_full(
        2 * N,
        1
    );

    LMtx J(
        2 * N,
        NP
    );

    LMtx Js(
        2 * N,
        NP
    );

    // --------------------------------------------------------
    // LM damping.
    // --------------------------------------------------------

    f64 lambda =
        1e-3;

    const f64 lambda_min =
        1e-12;

    const f64 lambda_max =
        1e12;

    // ========================================================
    // Main optimization loop
    // ========================================================

    for (int iter = 0;
        iter < max_iter;
        ++iter)
    {
        // ----------------------------------------------------
        // Calculate Jacobian and residual.
        // ----------------------------------------------------

        compute_full_jacobian(
            N,
            5,
            image_pts,
            ideal_pts,
            fx,
            fy,
            cx,
            cy,
            state,
            J_full,
            r_full
        );

        // ----------------------------------------------------
        // Extract active columns.
        // ----------------------------------------------------

        for (int i = 0;
            i < 2 * N;
            ++i)
        {
            for (int k = 0;
                k < NP;
                ++k)
            {
                J.at(i, k) =
                    J_full.at(
                        i,
                        active[k]
                    );
            }
        }

        // ----------------------------------------------------
        // Apply parameter scaling.
        // ----------------------------------------------------

        for (int i = 0;
            i < 2 * N;
            ++i)
        {
            for (int k = 0;
                k < NP;
                ++k)
            {
                Js.at(i, k) =
                    J.at(i, k)
                    * scale[k];
            }
        }

        // ----------------------------------------------------
        // H = J^T J
        // g = J^T r
        // ----------------------------------------------------

        LMtx Jt =
            Js.T();

        LMtx H =
            Jt * Js;

        LMtx g =
            Jt * r_full;

        // ----------------------------------------------------
        // Find maximum diagonal.
        // ----------------------------------------------------

        f64 max_diag =
            0.0;

        for (int k = 0;
            k < NP;
            ++k)
        {
            f64 diag =
                H.at(k, k);

            if (finite_f64(diag) &&
                diag > max_diag)
            {
                max_diag =
                    diag;
            }
        }

        if (max_diag < 1e-18)
            max_diag = 1e-18;

        // ----------------------------------------------------
        // Try multiple damping values.
        // ----------------------------------------------------

        bool accepted =
            false;

        CalibDistort best_state =
            state;

        f64 best_cost =
            cost;

        f64 best_step_norm =
            HUGE_VAL;

        f64 used_lambda =
            lambda;

        for (int attempt = 0;
            attempt < 12;
            ++attempt)
        {
            // ------------------------------------------------
            // IMPORTANT:
            //
            // LMtx is non-copyable.
            //
            // Therefore we cannot do:
            //
            //     LMtx Hd = H;
            //
            // Instead construct a new matrix and manually
            // copy the elements.
            // ------------------------------------------------

            LMtx Hd(
                NP,
                NP
            );

            for (int r = 0;
                r < NP;
                ++r)
            {
                for (int c = 0;
                    c < NP;
                    ++c)
                {
                    Hd.at(r, c) =
                        H.at(r, c);
                }
            }

            // ------------------------------------------------
            // LM diagonal damping:
            //
            // Hii += lambda * max_diag
            // ------------------------------------------------

            f64 damp =
                used_lambda * max_diag;

            for (int k = 0;
                k < NP;
                ++k)
            {
                Hd.at(k, k) +=
                    damp;
            }

            // ------------------------------------------------
            // Build -g.
            //
            // Matrix is non-copyable, so solve_gauss receives
            // it using std::move().
            // ------------------------------------------------

            LMtx neg_g =
                g * (-1.0);

            LMtx delta_s =
                solve_gauss(
                    std::move(Hd),
                    std::move(neg_g)
                );

            // ------------------------------------------------
            // Check Gaussian solver result.
            // ------------------------------------------------

            if (delta_s.rows != NP ||
                delta_s.cols != 1 ||
                delta_s.data == nullptr)
            {
                used_lambda *= 10.0;

                if (used_lambda >
                    lambda_max)
                {
                    break;
                }

                continue;
            }

            // ------------------------------------------------
            // Check delta for NaN / Inf.
            // ------------------------------------------------

            bool finite_delta =
                true;

            for (int k = 0;
                k < NP;
                ++k)
            {
                if (!finite_f64(
                    delta_s.at(k, 0)))
                {
                    finite_delta =
                        false;

                    break;
                }
            }

            if (!finite_delta)
            {
                used_lambda *= 10.0;

                if (used_lambda >
                    lambda_max)
                {
                    break;
                }

                continue;
            }

            // ------------------------------------------------
            // Convert scaled delta to real parameter delta.
            // ------------------------------------------------

            std::vector<f64> delta_p(
                NP,
                0.0
            );

            for (int k = 0;
                k < NP;
                ++k)
            {
                delta_p[k] =
                    delta_s.at(k, 0)
                    * scale[k];
            }

            // ------------------------------------------------
            // Step norm in scaled parameter space.
            // ------------------------------------------------

            f64 step_norm =
                0.0;

            for (int k = 0;
                k < NP;
                ++k)
            {
                f64 q =
                    delta_s.at(k, 0);

                step_norm +=
                    q * q;
            }

            step_norm =
                std::sqrt(step_norm);

            if (!finite_f64(step_norm))
            {
                used_lambda *= 10.0;

                if (used_lambda >
                    lambda_max)
                {
                    break;
                }

                continue;
            }

            // ------------------------------------------------
            // Backtracking line search.
            // ------------------------------------------------

            for (int li = 0;
                li < 12;
                ++li)
            {
                f64 alpha =
                    1.0 /
                    (f64)(1 << li);

                CalibDistort trial =
                    state;

                bool valid_trial =
                    true;

                for (int k = 0;
                    k < NP;
                    ++k)
                {
                    f64 old_p =
                        param_get(
                            state,
                            active[k]
                        );

                    f64 new_p =
                        old_p
                        + alpha
                        * delta_p[k];

                    // Prevent numerical explosion.
                    if (!finite_f64(new_p) ||
                        std::fabs(new_p) > 100.0)
                    {
                        valid_trial =
                            false;

                        break;
                    }

                    param_set(
                        trial,
                        active[k],
                        new_p
                    );
                }

                if (!valid_trial)
                    continue;

                f64 trial_cost =
                    compute_cost_all(
                        trial,
                        image_pts,
                        ideal_pts,
                        fx,
                        fy,
                        cx,
                        cy
                    );

                if (finite_f64(trial_cost) &&
                    trial_cost < best_cost)
                {
                    best_cost =
                        trial_cost;

                    best_state =
                        trial;

                    best_step_norm =
                        step_norm
                        * alpha;

                    accepted =
                        true;

                    break;
                }
            }

            // ------------------------------------------------
            // A valid step was found.
            // ------------------------------------------------

            if (accepted)
                break;

            // ------------------------------------------------
            // Otherwise increase damping.
            // ------------------------------------------------

            used_lambda *=
                10.0;

            if (used_lambda >
                lambda_max)
            {
                break;
            }
        }

        // ----------------------------------------------------
        // Diagnostics.
        // ----------------------------------------------------

        if (verbose &&
            iter < 8)
        {
            f64 rms =
                std::sqrt(
                    cost /
                    (2.0 * N)
                );

            std::printf(
                "    [iter %3d] "
                "rms=%.8f "
                "lambda=%.3e",
                iter,
                rms,
                used_lambda
            );
        }

        // ----------------------------------------------------
        // No acceptable step.
        // ----------------------------------------------------

        if (!accepted)
        {
            if (verbose &&
                iter < 8)
            {
                std::printf(
                    " REJECT\n"
                );
            }

            lambda =
                used_lambda * 10.0;

            if (lambda >
                lambda_max)
            {
                break;
            }

            continue;
        }

        // ----------------------------------------------------
        // Apply step.
        // ----------------------------------------------------

        state =
            best_state;

        f64 old_cost =
            cost;

        cost =
            best_cost;

        res.iterations =
            iter + 1;

        // ----------------------------------------------------
        // Successful step -> decrease damping.
        // ----------------------------------------------------

        lambda =
            used_lambda * 0.3;

        if (lambda <
            lambda_min)
        {
            lambda =
                lambda_min;
        }

        // ----------------------------------------------------
        // Diagnostics.
        // ----------------------------------------------------

        if (verbose &&
            iter < 8)
        {
            f64 rms =
                std::sqrt(
                    cost /
                    (2.0 * N)
                );

            std::printf(
                " ACCEPT "
                "step=%.3e "
                "rms=%.8f "
                "cost_change=%.3e\n",
                best_step_norm,
                rms,
                old_cost - cost
            );
        }

        // ----------------------------------------------------
        // Convergence.
        // ----------------------------------------------------

        f64 cost_change =
            std::fabs(
                old_cost - cost
            );

        f64 relative_change =
            cost_change /
            std::max(
                1.0,
                std::fabs(old_cost)
            );

        // Parameter change is tiny.
        if (best_step_norm <
            tol)
        {
            res.converged =
                true;

            break;
        }

        // Cost change is tiny.
        if (relative_change <
            tol * 0.1)
        {
            res.converged =
                true;

            break;
        }

        // Practically zero error.
        if (cost <
            1e-16)
        {
            res.converged =
                true;

            break;
        }
    }

    // --------------------------------------------------------
    // Final result.
    // --------------------------------------------------------

    res.d =
        state;

    res.final_error =
        cost;

    return res;
}

// ============================================================
// Public distortion calibration
// ============================================================

LMResult lm_calibrate_distort(
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx,
    f64 fy,
    f64 cx,
    f64 cy,
    const CalibDistort& init,
    int max_iter,
    f64 tol
)
{
    LMResult res;

    res.d =
        init;

    res.final_error =
        HUGE_VAL;

    res.iterations =
        0;

    res.converged =
        false;

    // --------------------------------------------------------
    // Validate input.
    // --------------------------------------------------------

    int N =
        (int)image_pts.size();

    if (N == 0)
    {
        std::printf(
            "  [calibrate] ERROR: no points\n"
        );

        return res;
    }

    if ((int)ideal_pts.size() != N)
    {
        std::printf(
            "  [calibrate] ERROR: "
            "image_pts=%d ideal_pts=%d\n",
            N,
            (int)ideal_pts.size()
        );

        return res;
    }

    if (fx <= 0.0 ||
        fy <= 0.0)
    {
        std::printf(
            "  [calibrate] ERROR: "
            "invalid fx/fy\n"
        );

        return res;
    }

    if (max_iter <= 0)
        max_iter = 200;

    if (tol <= 0.0)
        tol = 1e-8;

    // --------------------------------------------------------
    // Initial RMS.
    // --------------------------------------------------------

    f64 cost0 =
        compute_cost_all(
            init,
            image_pts,
            ideal_pts,
            fx,
            fy,
            cx,
            cy
        );

    if (!finite_f64(cost0))
    {
        std::printf(
            "  [calibrate] ERROR: "
            "initial cost invalid\n"
        );

        return res;
    }

    f64 rms0 =
        std::sqrt(
            cost0 /
            (2.0 * N)
        );

    std::printf(
        "  [stage 0] init rms=%.8f\n",
        rms0
    );

    // --------------------------------------------------------
    // Already solved.
    // --------------------------------------------------------

    if (rms0 < 1e-8)
    {
        res.d =
            init;

        res.final_error =
            cost0;

        res.converged =
            true;

        res.iterations =
            0;

        std::printf(
            "  [stage 0] "
            "already near optimal, skip\n"
        );

        return res;
    }

    // --------------------------------------------------------
    // Current optimization state.
    // --------------------------------------------------------

    CalibDistort state =
        init;

    int total_iterations =
        0;

    // ========================================================
    // Stage helper
    // ========================================================

    auto run_stage =
        [&](const char* name,
            const std::vector<int>& active) -> bool
        {
            const char* names[5] =
            {
                "k1",
                "k2",
                "p1",
                "p2",
                "k3"
            };

            std::printf(
                "  [stage %s] active params:",
                name
            );

            for (int k : active)
            {
                std::printf(
                    " %s",
                    names[k]
                );
            }

            std::printf("\n");

            LMResult sub =
                lm_solve_subset(
                    state,
                    image_pts,
                    ideal_pts,
                    fx,
                    fy,
                    cx,
                    cy,
                    active,
                    max_iter,
                    tol,
                    true
                );

            if (!finite_f64(
                sub.final_error))
            {
                std::printf(
                    "    -> stage failed: "
                    "invalid result\n"
                );

                return false;
            }

            state =
                sub.d;

            total_iterations +=
                sub.iterations;

            f64 rms =
                std::sqrt(
                    sub.final_error /
                    (2.0 * N)
                );

            std::printf(
                "    -> rms=%.8f "
                "iters=%d "
                "converged=%s\n",
                rms,
                sub.iterations,
                sub.converged
                ? "yes"
                : "no"
            );

            return true;
        };

    // ========================================================
    // Stage 1: k1
    // ========================================================

    if (!run_stage(
        "1",
        { 0 }))
    {
        res.d =
            state;

        res.final_error =
            compute_cost_all(
                state,
                image_pts,
                ideal_pts,
                fx,
                fy,
                cx,
                cy
            );

        res.iterations =
            total_iterations;

        return res;
    }

    // ========================================================
    // Stage 2: k1 + k2
    // ========================================================

    if (!run_stage(
        "2",
        { 0, 1 }))
    {
        res.d =
            state;

        res.final_error =
            compute_cost_all(
                state,
                image_pts,
                ideal_pts,
                fx,
                fy,
                cx,
                cy
            );

        res.iterations =
            total_iterations;

        return res;
    }

    // ========================================================
    // Stage 3: k1 + k2 + p1 + p2
    // ========================================================

    if (!run_stage(
        "3",
        { 0, 1, 2, 3 }))
    {
        res.d =
            state;

        res.final_error =
            compute_cost_all(
                state,
                image_pts,
                ideal_pts,
                fx,
                fy,
                cx,
                cy
            );

        res.iterations =
            total_iterations;

        return res;
    }

    // ========================================================
    // Stage 4: k1 + k2 + p1 + p2 + k3
    // ========================================================

    if (!run_stage(
        "4",
        { 0, 1, 2, 3, 4 }))
    {
        res.d =
            state;

        res.final_error =
            compute_cost_all(
                state,
                image_pts,
                ideal_pts,
                fx,
                fy,
                cx,
                cy
            );

        res.iterations =
            total_iterations;

        return res;
    }

    // ========================================================
    // Final result
    // ========================================================

    res.d =
        state;

    res.final_error =
        compute_cost_all(
            state,
            image_pts,
            ideal_pts,
            fx,
            fy,
            cx,
            cy
        );

    res.iterations =
        total_iterations;

    if (finite_f64(
        res.final_error))
    {
        f64 rms_final =
            std::sqrt(
                res.final_error /
                (2.0 * N)
            );

        std::printf(
            "  [final] rms=%.8f\n",
            rms_final
        );

        if (rms_final < 1e-6)
        {
            res.converged =
                true;
        }
    }

    return res;
}