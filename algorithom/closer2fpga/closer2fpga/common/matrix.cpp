#define _CRT_SECURE_NO_WARNINGS

#include "matrix.h"
#include <cmath>
#include <cstdio>

// ------------------------------------------------------------
// Gaussian elimination with partial pivoting
//
// Solves:
//     A * x = b
//
// A must be square.
// b must be n x 1.
//
// Compared with the original version:
//   1. Uses partial pivoting.
//   2. Detects singular / nearly-singular pivots.
//   3. Never divides by an almost-zero pivot.
//   4. Returns a zero vector when the system cannot be solved safely.
//
// This keeps the original function interface so the rest of the
// project does not need to be changed.
// ------------------------------------------------------------

LMtx solve_gauss(LMtx A_in, LMtx b) {
    const int n = A_in.rows;

    // Basic validation.
    if (n <= 0 || A_in.cols != n || b.rows != n || b.cols != 1) {
        return LMtx();
    }

    // Augmented matrix [A | b]
    LMtx A(n, n + 1);

    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            A.at(r, c) = A_in.at(r, c);
        }

        A.at(r, n) = b.at(r, 0);
    }

    // --------------------------------------------------------
    // Forward elimination
    // --------------------------------------------------------

    for (int col = 0; col < n; ++col) {

        // Find largest pivot in this column.
        int max_row = col;
        f64 max_val = std::fabs(A.at(col, col));

        for (int r = col + 1; r < n; ++r) {
            f64 v = std::fabs(A.at(r, col));

            if (v > max_val) {
                max_val = v;
                max_row = r;
            }
        }

        // ----------------------------------------------------
        // Singular / nearly singular detection.
        //
        // Use a scale-dependent tolerance instead of only
        // testing against an absolute 1e-18.
        // ----------------------------------------------------

        f64 row_scale = 0.0;

        for (int c = col; c < n; ++c) {
            f64 v = std::fabs(A.at(max_row, c));
            if (v > row_scale)
                row_scale = v;
        }

        if (row_scale < 1e-30) {
            // Matrix contains effectively zero row.
            return LMtx();
        }

        f64 pivot_tol = row_scale * 1e-14;

        if (max_val < pivot_tol) {
            // Numerically singular.
            return LMtx();
        }

        // Swap pivot row into place.
        if (max_row != col) {
            for (int c = col; c <= n; ++c) {
                f64 tmp = A.at(col, c);
                A.at(col, c) = A.at(max_row, c);
                A.at(max_row, c) = tmp;
            }
        }

        f64 pivot = A.at(col, col);

        // Safety check.
        if (!std::isfinite(pivot) || std::fabs(pivot) < 1e-30) {
            return LMtx();
        }

        // Eliminate rows below.
        for (int r = col + 1; r < n; ++r) {

            f64 v = A.at(r, col);

            if (std::fabs(v) < 1e-30)
                continue;

            f64 factor = v / pivot;

            if (!std::isfinite(factor)) {
                return LMtx();
            }

            // Start at col because entries before col should
            // already be zero.
            for (int c = col; c <= n; ++c) {
                A.at(r, c) -= factor * A.at(col, c);
            }
        }
    }

    // --------------------------------------------------------
    // Back substitution
    // --------------------------------------------------------

    LMtx x(n, 1);

    for (int r = n - 1; r >= 0; --r) {

        f64 pivot = A.at(r, r);

        if (!std::isfinite(pivot) || std::fabs(pivot) < 1e-30) {
            return LMtx();
        }

        f64 s = A.at(r, n);

        for (int c = r + 1; c < n; ++c) {
            s -= A.at(r, c) * x.at(c, 0);
        }

        f64 value = s / pivot;

        if (!std::isfinite(value)) {
            return LMtx();
        }

        x.at(r, 0) = value;
    }

    return x;
}