#pragma once

namespace kernels {
// Four already-fetched samples. No memory access, border policy or state.
// Hardware cut points: horizontal pair -> vertical blend.
inline float bilinear(float p00, float p10, float p01, float p11, float dx, float dy) {
    float top = (1 - dx) * p00 + dx * p10;
    float bottom = (1 - dx) * p01 + dx * p11;
    return (1 - dy) * top + dy * bottom;
}
} // namespace kernels
