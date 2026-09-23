#include "undistort.h"
#include "../kernels/interpolation.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

void remap_bilinear(ImageView<const uint8_t> src, ImageView<const float> map_x, ImageView<const float> map_y,
                    ImageView<uint8_t> dst, RemapBorder border) {
    if (!src.valid() || !dst.valid() || !map_x.valid() || !map_y.valid() || (src.c != 1 && src.c != 3) ||
        dst.c != src.c || map_x.c != 1 || map_y.c != 1 || map_x.w != dst.w || map_x.h != dst.h ||
        map_y.w != dst.w || map_y.h != dst.h || overlaps(src, dst) || overlaps(map_x, dst) ||
        overlaps(map_y, dst))
        throw std::invalid_argument("Invalid or overlapping remap buffers");
    auto sample = [&](int x, int y, int c) -> float {
        if (x < 0 || y < 0 || x >= src.w || y >= src.h) {
            if (border == RemapBorder::ConstantBlack)
                return 0;
            x = std::clamp(x, 0, src.w - 1);
            y = std::clamp(y, 0, src.h - 1);
        }
        return src.row(y)[size_t(x) * src.c + c];
    };
    // Raster traversal is control; sample() represents memory reads.
    for (int y = 0; y < dst.h; ++y) {
        for (int x = 0; x < dst.w; ++x) {
            auto* pixel = dst.row(y) + size_t(x) * dst.c;
            std::fill_n(pixel, dst.c, uint8_t(0));
            float sx = map_x.row(y)[x], sy = map_y.row(y)[x];
            if (!std::isfinite(sx) || !std::isfinite(sy))
                continue;
            if (border == RemapBorder::Replicate) {
                sx = std::clamp(sx, 0.f, float(src.w - 1));
                sy = std::clamp(sy, 0.f, float(src.h - 1));
            } else if (sx <= -1 || sy <= -1 || sx >= src.w || sy >= src.h)
                continue;
            int ix = int(std::floor(sx)), iy = int(std::floor(sy));
            float dx = sx - ix, dy = sy - iy;
            for (int c = 0; c < src.c; ++c) {
                float value = kernels::bilinear(sample(ix, iy, c), sample(ix + 1, iy, c),
                                                sample(ix, iy + 1, c), sample(ix + 1, iy + 1, c), dx, dy);
                pixel[c] = uint8_t(std::clamp(std::lround(value), 0L, 255L));
            }
        }
    }
}

void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst, RemapBorder border) {
    if (!image_view(src).valid() || (src.c != 1 && src.c != 3) || table.w <= 0 || table.h <= 0 ||
        table.map_x.w != table.w || table.map_x.h != table.h || table.map_y.w != table.w ||
        table.map_y.h != table.h)
        throw std::invalid_argument("Invalid source image or remap table");
    GrayImage output(table.w, table.h, src.c);
    remap_bilinear(image_view(src), image_view(table.map_x), image_view(table.map_y), image_view(output),
                   border);
    dst = std::move(output);
}
