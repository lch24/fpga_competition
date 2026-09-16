#define _CRT_SECURE_NO_WARNINGS
#include "matrix.h"
#include <cmath>

LMtx solve_gauss(LMtx A_in, LMtx b) {
    int n = A_in.rows;
    LMtx A(n, n + 1);
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c)
            A.at(r, c) = A_in.at(r, c);
        A.at(r, n) = b.at(r, 0);
    }

    for (int col = 0; col < n; ++col) {
        int max_row = col;
        f64 max_val = std::fabs(A.at(col, col));
        for (int r = col + 1; r < n; ++r) {
            if (std::fabs(A.at(r, col)) > max_val) {
                max_val = std::fabs(A.at(r, col));
                max_row = r;
            }
        }
        if (max_row != col) {
            for (int c = 0; c <= n; ++c) {
                f64 tmp = A.at(col, c);
                A.at(col, c) = A.at(max_row, c);
                A.at(max_row, c) = tmp;
            }
        }
        f64 piv = A.at(col, col);
        if (std::fabs(piv) < 1e-18) continue;
        for (int r = col + 1; r < n; ++r) {
            f64 factor = A.at(r, col) / piv;
            for (int c = col; c <= n; ++c)
                A.at(r, c) -= factor * A.at(col, c);
        }
    }

    LMtx x(n, 1);
    for (int r = n - 1; r >= 0; --r) {
        f64 s = A.at(r, n);
        for (int c = r + 1; c < n; ++c)
            s -= A.at(r, c) * x.at(c, 0);
        x.at(r, 0) = s / A.at(r, r);
    }
    return x;
}