#pragma once
#include <array>

// Stateless small-matrix operations; rotation conversion also uses sqrt/trig.
// See docs/RTL_GUIDE.md for arithmetic-resource and scheduling implications.
namespace math3 {
using M3 = std::array<double, 9>;
using V3 = std::array<double, 3>;
double dot(V3 a, V3 b);
V3 cross(V3 a, V3 b);
V3 scaled(V3 a, double scale);
M3 multiply(const M3& a, const M3& b);
M3 rodrigues(V3 rotation_vector);
V3 rotation_vector(const M3& rotation);
} // namespace math3
