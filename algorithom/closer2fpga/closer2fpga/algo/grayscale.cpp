#include "grayscale.h"
#include "../kernels/color.h"

void convert_grayscale(ImageView<const uint8_t> source, ImageView<uint8_t> destination) {
    if (!source.valid() || !destination.valid() || source.c != 3 || destination.c != 1 ||
        source.w != destination.w || source.h != destination.h || overlaps(source, destination))
        throw std::invalid_argument("Invalid or overlapping grayscale buffers");
    for (int y = 0; y < source.h; ++y) {
        const auto* input = source.row(y);
        auto* output = destination.row(y);
        for (int x = 0; x < source.w; ++x)
            output[x] = kernels::bgr_to_gray(input[3 * x], input[3 * x + 1], input[3 * x + 2]);
    }
}
