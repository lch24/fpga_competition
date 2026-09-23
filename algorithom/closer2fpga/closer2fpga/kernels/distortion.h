#pragma once

namespace kernels {
template <class T> struct Distortion {
    T k1, k2, k3, p1, p2;
};
template <class T> struct XY {
    T x, y;
};

// Normalized ideal coordinate -> normalized distorted coordinate.
// Shared by residual evaluation (double) and remap generation (float).
// Suggested register cuts: radius powers -> radial/tangent -> output.
template <class T> XY<T> distort(T x, T y, const Distortion<T>& k) {
    T r2 = x * x + y * y;
    T radial = T(1) + k.k1 * r2 + k.k2 * r2 * r2 + k.k3 * r2 * r2 * r2;
    return {x * radial + T(2) * k.p1 * x * y + k.p2 * (r2 + T(2) * x * x),
            y * radial + k.p1 * (r2 + T(2) * y * y) + T(2) * k.p2 * x * y};
}
} // namespace kernels
