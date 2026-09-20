#include "undistort.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

static void forward_distort_norm(f32 nx, f32 ny, const CameraParams& cam, f32& xd, f32& yd) {
    f32 r2 = nx * nx + ny * ny;
    f32 radial = 1.0f + cam.k1 * r2 + cam.k2 * r2 * r2 + cam.k3 * r2 * r2 * r2;
    f32 xt = 2.0f * cam.p1 * nx * ny + cam.p2 * (r2 + 2.0f * nx * nx);
    f32 yt = cam.p1 * (r2 + 2.0f * ny * ny) + 2.0f * cam.p2 * nx * ny;
    xd = nx * radial + xt;
    yd = ny * radial + yt;
}

RemapTable build_remap_table(int w, int h, const CameraParams& cam) {
    if (w <= 0 || h <= 0 || !std::isfinite(cam.fx) || !std::isfinite(cam.fy) ||
        cam.fx <= 0 || cam.fy <= 0 || !std::isfinite(cam.cx) || !std::isfinite(cam.cy) ||
        !std::isfinite(cam.k1) || !std::isfinite(cam.k2) || !std::isfinite(cam.k3) ||
        !std::isfinite(cam.p1) || !std::isfinite(cam.p2))
        throw std::invalid_argument("Invalid image dimensions or camera parameters for remap");
    RemapTable table;
    table.w = w;
    table.h = h;
    table.map_x = FloatMap(w, h);
    table.map_y = FloatMap(w, h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            f32 nx = ((f32)x - cam.cx) / cam.fx;
            f32 ny = ((f32)y - cam.cy) / cam.fy;
            f32 xd, yd;
            forward_distort_norm(nx, ny, cam, xd, yd);
            table.map_x.set(x, y, cam.fx * xd + cam.cx);
            table.map_y.set(x, y, cam.fy * yd + cam.cy);
        }
    }

    return table;
}

void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst, RemapBorder border) {
    if (!src.data || src.w <= 0 || src.h <= 0 || (src.c != 1 && src.c != 3) ||
        table.w <= 0 || table.h <= 0 || !table.map_x.data || !table.map_y.data ||
        table.map_x.w != table.w || table.map_x.h != table.h || table.map_x.c != 1 ||
        table.map_y.w != table.w || table.map_y.h != table.h || table.map_y.c != 1)
        throw std::invalid_argument("Invalid source image or remap table");
    GrayImage output(table.w, table.h, src.c);
    auto sample = [&](int x, int y, int c) -> float {
        if (x < 0 || y < 0 || x >= src.w || y >= src.h) {
            if (border == RemapBorder::ConstantBlack) return 0;
            x = std::clamp(x, 0, src.w - 1); y = std::clamp(y, 0, src.h - 1);
        }
        return src.data[(size_t(y) * src.w + x) * src.c + c];
    };
    for (int y = 0; y < table.h; ++y) {
        for (int x = 0; x < table.w; ++x) {
            f32 sx = table.map_x.get(x, y);
            f32 sy = table.map_y.get(x, y);
            if (!std::isfinite(sx) || !std::isfinite(sy)) continue;
            if (border == RemapBorder::Replicate) {
                sx = std::clamp(sx, 0.f, float(src.w - 1));
                sy = std::clamp(sy, 0.f, float(src.h - 1));
            } else if (sx <= -1 || sy <= -1 || sx >= src.w || sy >= src.h) continue;
            int ix = int(std::floor(sx)), iy = int(std::floor(sy));
            float dx = sx - ix, dy = sy - iy;
            for (int c = 0; c < src.c; ++c) {
                float top = (1-dx)*sample(ix,iy,c) + dx*sample(ix+1,iy,c);
                float bottom = (1-dx)*sample(ix,iy+1,c) + dx*sample(ix+1,iy+1,c);
                output.data[(size_t(y)*table.w+x)*src.c+c] = uint8_t(std::clamp(std::lround((1-dy)*top+dy*bottom),0L,255L));
            }
        }
    }
    dst = std::move(output);
}
