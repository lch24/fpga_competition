#include "symmetric_eigen.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace linalg {
// Jacobi eigensolver for small real symmetric matrices (normalized DLT/Zhang).
// Eigenvectors are columns; sorting leaves the smallest eigenpair first.
bool eigen_symmetric(std::vector<double> a, int n, std::vector<double>& values,
                     std::vector<double>& vectors) {
    vectors.assign(n * n, 0);
    for (int i = 0; i < n; ++i)
        vectors[i * n + i] = 1;
    bool done = false;
    for (int iteration = 0; iteration < 100 * n * n; ++iteration) {
        int p = 0, q = 1;
        double largest = 0, diagonal = 0;
        for (int i = 0; i < n; ++i) {
            diagonal = std::max(diagonal, std::fabs(a[i * n + i]));
            for (int j = i + 1; j < n; ++j)
                if (std::fabs(a[i * n + j]) > largest) {
                    largest = std::fabs(a[i * n + j]);
                    p = i;
                    q = j;
                }
        }
        if (largest <= 1e-14 * std::max(diagonal, 1e-30)) {
            done = true;
            break;
        }
        double phi = 0.5 * std::atan2(2 * a[p * n + q], a[q * n + q] - a[p * n + p]);
        double c = std::cos(phi), s = std::sin(phi);
        double app = a[p * n + p], aqq = a[q * n + q], apq = a[p * n + q];
        for (int k = 0; k < n; ++k)
            if (k != p && k != q) {
                double akp = a[k * n + p], akq = a[k * n + q];
                a[k * n + p] = a[p * n + k] = c * akp - s * akq;
                a[k * n + q] = a[q * n + k] = s * akp + c * akq;
            }
        a[p * n + p] = c * c * app - 2 * s * c * apq + s * s * aqq;
        a[q * n + q] = s * s * app + 2 * s * c * apq + c * c * aqq;
        a[p * n + q] = a[q * n + p] = 0;
        for (int k = 0; k < n; ++k) {
            double vkp = vectors[k * n + p], vkq = vectors[k * n + q];
            vectors[k * n + p] = c * vkp - s * vkq;
            vectors[k * n + q] = s * vkp + c * vkq;
        }
    }
    if (!done)
        return false;
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int i, int j) { return a[i * n + i] < a[j * n + j]; });
    auto original = vectors;
    values.resize(n);
    for (int j = 0; j < n; ++j) {
        values[j] = a[order[j] * n + order[j]];
        for (int i = 0; i < n; ++i)
            vectors[i * n + j] = original[i * n + order[j]];
    }
    return true;
}

void accumulate_outer(std::vector<double>& a, const std::vector<double>& row) {
    const int n = int(row.size());
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            a[i * n + j] += row[i] * row[j];
}
} // namespace linalg
