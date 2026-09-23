#pragma once
#include <algorithm>
#include <cmath>

namespace kernels {
struct Gradient {
    float x, y;
};
// Caller supplies the window and implements line buffers/boundary extension.
inline Gradient sobel(float tl, float tc, float tr, float ml, float mr, float bl, float bc, float br) {
    return {-tl - 2 * ml - bl + tr + 2 * mr + br, -tl - 2 * tc - tr + bl + 2 * bc + br};
}
template <class T> struct Tensor {
    T xx, xy, yy;
};
template <class T> Tensor<T> outer_product(T gx, T gy, T weight = T(1)) {
    return {weight * gx * gx, weight * gx * gy, weight * gy * gy};
}
inline float min_eigenvalue(float a, float b, float c) {
    float trace = a + c, det = a * c - b * b;
    double discriminant = double(trace) * trace - 4.0 * det;
    return float(trace - std::sqrt(std::max(0.0, discriminant))) * 0.5f;
}
// One local 2x2 solve; the caller owns window accumulation and iteration.
inline bool solve_tensor(double a, double b, double c, double bx, double by, double& dx, double& dy) {
    double det = a * c - b * b, trace = a + c;
    if (trace < 1e-8 || det <= 1e-5 * trace * trace)
        return false;
    dx = (c * bx - b * by) / det;
    dy = (a * by - b * bx) / det;
    return true;
}
} // namespace kernels
