#include "matrix.h"
#include <cmath>

LMtx solve_gauss(LMtx input, LMtx rhs) {
    const int n = input.rows;
    if (n <= 0 || input.cols != n || rhs.rows != n || rhs.cols != 1)
        return {};

    LMtx augmented(n, n + 1);
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c)
            augmented.at(r, c) = input.at(r, c);
        augmented.at(r, n) = rhs.at(r, 0);
    }

    for (int col = 0; col < n; ++col) {
        int pivot_row = col;
        double max_value = std::fabs(augmented.at(col, col));
        for (int r = col + 1; r < n; ++r) {
            double value = std::fabs(augmented.at(r, col));
            if (value > max_value) {
                max_value = value;
                pivot_row = r;
            }
        }

        // Relative pivot tolerance detects nearly singular systems at any scale.
        double row_scale = 0;
        for (int c = col; c < n; ++c) {
            double value = std::fabs(augmented.at(pivot_row, c));
            if (value > row_scale)
                row_scale = value;
        }
        if (row_scale < 1e-30 || max_value < row_scale * 1e-14)
            return {};
        if (pivot_row != col) {
            for (int c = col; c <= n; ++c) {
                double value = augmented.at(col, c);
                augmented.at(col, c) = augmented.at(pivot_row, c);
                augmented.at(pivot_row, c) = value;
            }
        }
        double pivot = augmented.at(col, col);
        if (!std::isfinite(pivot) || std::fabs(pivot) < 1e-30)
            return {};

        for (int r = col + 1; r < n; ++r) {
            double value = augmented.at(r, col);
            if (std::fabs(value) < 1e-30)
                continue;
            double factor = value / pivot;
            if (!std::isfinite(factor))
                return {};
            for (int c = col; c <= n; ++c)
                augmented.at(r, c) -= factor * augmented.at(col, c);
        }
    }

    LMtx solution(n, 1);
    for (int r = n - 1; r >= 0; --r) {
        double pivot = augmented.at(r, r);
        if (!std::isfinite(pivot) || std::fabs(pivot) < 1e-30)
            return {};
        double value = augmented.at(r, n);
        for (int c = r + 1; c < n; ++c)
            value -= augmented.at(r, c) * solution.at(c, 0);
        value /= pivot;
        if (!std::isfinite(value))
            return {};
        solution.at(r, 0) = value;
    }
    return solution;
}
