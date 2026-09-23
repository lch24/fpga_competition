#include "../closer2fpga/algo/grayscale.h"
#include "../closer2fpga/algo/undistort.h"
#include "../closer2fpga/kernels/distortion.h"
#include "../closer2fpga/kernels/gradient.h"
#include "../closer2fpga/kernels/interpolation.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
int failures = 0;
void check(bool condition, const char* name) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", name);
    }
}
template <class F> void rejects(F operation, const char* name) {
    try {
        operation();
        check(false, name);
    } catch (const std::invalid_argument&) {
    }
}
void grayscale_buffers() {
    // B, G, R, white; two padded BGR rows -> padded gray rows.
    const uint8_t source[] = {255, 0, 0, 0, 255, 0, 77, 77, 0, 0, 255, 255, 255, 255, 77, 77};
    uint8_t output[] = {99, 99, 99, 99, 99, 99};
    ImageView<const uint8_t> input{source, 2, 2, 3, 8};
    ImageView<uint8_t> destination{output, 2, 2, 1, 3};
    convert_grayscale(input, destination);
    check(output[0] == 29 && output[1] == 150 && output[3] == 76 && output[4] == 255, "gray primary colors");
    check(output[2] == 99 && output[5] == 99, "gray padding untouched");
    rejects([&] { convert_grayscale(input, {output, 2, 2, 1, 1}); }, "short stride rejected");
    rejects([&] { convert_grayscale({output, 1, 1, 3, 3}, {output, 1, 1, 1, 1}); }, "gray overlap rejected");
}
void remap_buffers() {
    std::array<uint8_t, 16> source{};
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 2; ++x)
            for (int c = 0; c < 3; ++c)
                source[y * 8 + x * 3 + c] = uint8_t(y * 80 + x * 40 + c * 10);
    const auto original = source;
    const float mx[] = {.5f, -.5f, 999, std::numeric_limits<float>::quiet_NaN(), 1, 999};
    const float my[] = {.5f, .5f, 999, 0, 1, 999};
    std::array<uint8_t, 18> output;
    output.fill(199);
    ImageView<const uint8_t> input{source.data(), 2, 2, 3, 8};
    ImageView<const float> map_x{mx, 2, 2, 1, 3}, map_y{my, 2, 2, 1, 3};
    ImageView<uint8_t> destination{output.data(), 2, 2, 3, 9};
    remap_bilinear(input, map_x, map_y, destination, RemapBorder::ConstantBlack);
    for (int c = 0; c < 3; ++c) {
        check(output[c] == 60 + c * 10, "center interpolation");
        check(output[3 + c] == 20 + c * 5, "partial black border");
        check(output[9 + c] == 0, "NaN clears prefilled destination");
        check(output[12 + c] == 120 + c * 10, "bottom right edge");
        check(output[6 + c] == 199 && output[15 + c] == 199, "remap padding untouched");
    }
    remap_bilinear(input, map_x, map_y, destination, RemapBorder::Replicate);
    for (int c = 0; c < 3; ++c)
        check(output[3 + c] == 40 + c * 10, "replicated border");
    check(source == original, "source unchanged");
    rejects([&] { remap_bilinear(input, map_x, map_y, {source.data(), 2, 2, 3, 8}); },
            "source overlap rejected");
    rejects([&] { remap_bilinear(input, {mx, 1, 2, 1, 3}, map_y, destination); }, "map shape rejected");
    float writable_map[6] = {};
    rejects(
        [&] {
            remap_bilinear(input, {writable_map, 2, 2, 1, 3}, map_y,
                           {reinterpret_cast<uint8_t*>(writable_map), 2, 2, 3, 6});
        },
        "map overlap rejected");
}
void arithmetic() {
    auto g = kernels::sobel(0, 1, 2, 2, 4, 4, 5, 6);
    check(g.x == 8 && g.y == 16, "Sobel linear ramp");
    auto t = kernels::outer_product(2., 3., .5);
    check(t.xx == 2 && t.xy == 3 && t.yy == 4.5, "weighted tensor");
    check(kernels::min_eigenvalue(4, 0, 9) == 4, "diagonal tensor eigenvalue");
    double dx = 0, dy = 0;
    check(kernels::solve_tensor(4, 0, 9, 8, 27, dx, dy) && dx == 2 && dy == 3, "tensor solve");
    check(!kernels::solve_tensor(1, 1, 1, 1, 1, dx, dy), "singular tensor rejected");
    check(kernels::bilinear(0, 40, 80, 120, .25f, .75f) == 70, "bilinear plane");
    auto p = kernels::distort(1., 2., kernels::Distortion<double>{.1, .01, .001, .02, -.03});
    check(std::fabs(p.x - 1.745) < 1e-12 && std::fabs(p.y - 3.89) < 1e-12,
          "Brown radial and tangential terms");
}
} // namespace
int main() {
    grayscale_buffers();
    remap_buffers();
    arithmetic();
    std::printf("Buffer/kernel regression: failures=%d (no OpenCV linked)\n", failures);
    return failures ? 1 : 0;
}
