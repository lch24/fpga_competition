#include "internal.h"
#include <cmath>

namespace calibration {
bool initialize(const std::vector<M3>& hom, const std::array<double, 4>& k, int w, int h, State& p) {
    p.assign(9 + 6 * hom.size(), 0);
    p[0] = std::log(k[0]);
    p[1] = std::log(k[1]);
    p[2] = k[2] / w;
    p[3] = k[3] / h;
    for (size_t i = 0; i < hom.size(); ++i) {
        const auto& a = hom[i];
        auto kinv = [&](int c) -> V3 {
            return {(a[c] - k[2] * a[6 + c]) / k[0], (a[3 + c] - k[3] * a[6 + c]) / k[1], a[6 + c]};
        };
        V3 v1 = kinv(0), v2 = kinv(1), t = kinv(2);
        double n1 = std::sqrt(dot(v1, v1)), n2 = std::sqrt(dot(v2, v2));
        if (n1 < 1e-12 || n2 < 1e-12)
            return false;
        double scale = 2 / (n1 + n2);
        if (t[2] < 0)
            scale = -scale;
        t = scaled(t, scale);
        V3 r1 = scaled(v1, scale > 0 ? 1 / n1 : -1 / n1);
        double parallel = dot(r1, v2);
        for (int j = 0; j < 3; ++j)
            v2[j] -= parallel * r1[j];
        double norm = std::sqrt(dot(v2, v2));
        if (norm < 1e-12 || t[2] <= 0)
            return false;
        V3 r2 = scaled(v2, scale > 0 ? 1 / norm : -1 / norm), r3 = cross(r1, r2);
        M3 rotation{r1[0], r2[0], r3[0], r1[1], r2[1], r3[1], r1[2], r2[2], r3[2]};
        V3 rv = rotation_vector(rotation);
        size_t offset = 9 + 6 * i;
        for (int j = 0; j < 3; ++j)
            p[offset + j] = rv[j];
        p[offset + 3] = t[0];
        p[offset + 4] = t[1];
        p[offset + 5] = std::log(t[2]);
    }
    return true;
}
} // namespace calibration
