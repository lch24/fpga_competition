#include "math3.h"
#include <cmath>

namespace math3 {
double dot(V3 a, V3 b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
V3 cross(V3 a, V3 b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
V3 scaled(V3 a, double s) {
    for (auto& v : a)
        v *= s;
    return a;
}
M3 multiply(const M3& a, const M3& b) {
    M3 c{};
    for (int r = 0; r < 3; ++r)
        for (int col = 0; col < 3; ++col)
            for (int k = 0; k < 3; ++k)
                c[r * 3 + col] += a[r * 3 + k] * b[k * 3 + col];
    return c;
}

M3 rodrigues(V3 v) {
    double t2 = dot(v, v), a, b;
    if (t2 < 1e-12) {
        a = 1 - t2 / 6;
        b = .5 - t2 / 24;
    } else {
        double t = std::sqrt(t2);
        a = std::sin(t) / t;
        b = (1 - std::cos(t)) / t2;
    }
    M3 s{0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0}, ss = multiply(s, s), r{};
    for (int i = 0; i < 9; ++i)
        r[i] = (i % 4 == 0 ? 1. : 0.) + a * s[i] + b * ss[i];
    return r;
}

V3 rotation_vector(const M3& r) {
    // Quaternion conversion remains stable for row/column-flipped boards whose
    // rotations can be close to pi (unlike division by sin(theta)).
    double qw, qx, qy, qz, trace = r[0] + r[4] + r[8];
    if (trace > 0) {
        double s = 2 * std::sqrt(trace + 1);
        qw = s / 4;
        qx = (r[7] - r[5]) / s;
        qy = (r[2] - r[6]) / s;
        qz = (r[3] - r[1]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
        double s = 2 * std::sqrt(1 + r[0] - r[4] - r[8]);
        qw = (r[7] - r[5]) / s;
        qx = s / 4;
        qy = (r[1] + r[3]) / s;
        qz = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
        double s = 2 * std::sqrt(1 + r[4] - r[0] - r[8]);
        qw = (r[2] - r[6]) / s;
        qx = (r[1] + r[3]) / s;
        qy = s / 4;
        qz = (r[5] + r[7]) / s;
    } else {
        double s = 2 * std::sqrt(1 + r[8] - r[0] - r[4]);
        qw = (r[3] - r[1]) / s;
        qx = (r[2] + r[6]) / s;
        qy = (r[5] + r[7]) / s;
        qz = s / 4;
    }
    if (qw < 0) {
        qw = -qw;
        qx = -qx;
        qy = -qy;
        qz = -qz;
    }
    double n = std::sqrt(qx * qx + qy * qy + qz * qz);
    double scale = n > 1e-12 ? 2 * std::atan2(n, qw) / n : 2;
    return {qx * scale, qy * scale, qz * scale};
}
} // namespace math3
