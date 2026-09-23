#include "undistort.h"
#include "../kernels/distortion.h"
#include <cmath>
#include <stdexcept>
RemapTable build_remap_table(int w, int h, const CameraParams& cam) {
    if (w <= 0 || h <= 0 || !std::isfinite(cam.fx) || !std::isfinite(cam.fy) || cam.fx <= 0 || cam.fy <= 0 ||
        !std::isfinite(cam.cx) || !std::isfinite(cam.cy) || !std::isfinite(cam.k1) ||
        !std::isfinite(cam.k2) || !std::isfinite(cam.k3) || !std::isfinite(cam.p1) || !std::isfinite(cam.p2))
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
            auto distorted =
                kernels::distort(nx, ny, kernels::Distortion<float>{cam.k1, cam.k2, cam.k3, cam.p1, cam.p2});
            f32 xd = distorted.x, yd = distorted.y;

            table.map_x.set(x, y, cam.fx * xd + cam.cx);
            table.map_y.set(x, y, cam.fy * yd + cam.cy);
        }
    }

    return table;
}
