#pragma once
#include <cstdint>
namespace kernels {
inline uint8_t bgr_to_gray(uint8_t b, uint8_t g, uint8_t r) {
    return uint8_t((299 * r + 587 * g + 114 * b + 500) / 1000);
}
} // namespace kernels
