#pragma once
#include "../common/image_view.h"
// BGR888 -> gray8, raster order. Padding untouched; overlapping buffers rejected.
void convert_grayscale(ImageView<const uint8_t> source, ImageView<uint8_t> destination);
