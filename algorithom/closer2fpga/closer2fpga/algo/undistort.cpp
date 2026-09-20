#define _CRT_SECURE_NO_WARNINGS
#include "undistort.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

void forward_distort_norm(f32 nx, f32 ny, const CameraParams& cam, f32& xd, f32& yd) {
    f32 r2 = nx * nx + ny * ny;
    f32 radial = 1.0f + cam.k1 * r2 + cam.k2 * r2 * r2 + cam.k3 * r2 * r2 * r2;
    f32 xt = 2.0f * cam.p1 * nx * ny + cam.p2 * (r2 + 2.0f * nx * nx);
    f32 yt = cam.p1 * (r2 + 2.0f * ny * ny) + 2.0f * cam.p2 * nx * ny;
    xd = nx * radial + xt;
    yd = ny * radial + yt;
}

void inverse_distort_norm(const CameraParams& cam, f32 xd, f32 yd, f32& nx, f32& ny) {
    if (std::fabs(cam.k1) < 1e-4f && std::fabs(cam.k2) < 1e-4f && std::fabs(cam.k3) < 1e-4f &&
        std::fabs(cam.p1) < 1e-4f && std::fabs(cam.p2) < 1e-4f) {
        nx = xd;
        ny = yd;
        return;
    }

    nx = xd;
    ny = yd;

    f32 w = (std::fabs(cam.k1) > 0.5f) ? 0.5f : 1.0f;
    f32 xd_pred, yd_pred;
    forward_distort_norm(nx, ny, cam, xd_pred, yd_pred);
    f32 diff_x = xd - xd_pred;
    f32 diff_y = yd - yd_pred;
    f32 prev_res = diff_x * diff_x + diff_y * diff_y;

    for (int iter = 0; iter < 50; ++iter) {
        f32 dnx = w * diff_x;
        f32 dny = w * diff_y;

        nx += dnx;
        ny += dny;

        forward_distort_norm(nx, ny, cam, xd_pred, yd_pred);
        diff_x = xd - xd_pred;
        diff_y = yd - yd_pred;
        f32 res = diff_x * diff_x + diff_y * diff_y;

        if (res > prev_res && w > 0.05f) {
            w *= 0.5f;
            nx -= dnx;
            ny -= dny;
            forward_distort_norm(nx, ny, cam, xd_pred, yd_pred);
            diff_x = xd - xd_pred;
            diff_y = yd - yd_pred;
            prev_res = diff_x * diff_x + diff_y * diff_y;
            continue;
        }

        prev_res = res;

        if (std::fabs(diff_x) < 1e-8f && std::fabs(diff_y) < 1e-8f) break;
    }
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

static f32 clamp_f32(f32 v, f32 lo, f32 hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

void make_distorted_image(const GrayImage& ideal, const CameraParams& cam, GrayImage& distorted) {
    distorted.w = ideal.w;
    distorted.h = ideal.h;
    distorted.c = 1;
    delete[] distorted.data;
    distorted.data = new uint8_t[distorted.w * distorted.h]();

    f32 max_x = (f32)(ideal.w - 1);
    f32 max_y = (f32)(ideal.h - 1);

    for (int v = 0; v < ideal.h; ++v) {
        for (int u = 0; u < ideal.w; ++u) {
            f32 xd_n = ((f32)u - cam.cx) / cam.fx;
            f32 yd_n = ((f32)v - cam.cy) / cam.fy;

            f32 nx, ny;
            inverse_distort_norm(cam, xd_n, yd_n, nx, ny);

            f32 sx = cam.fx * nx + cam.cx;
            f32 sy = cam.fy * ny + cam.cy;

            sx = clamp_f32(sx, 0.0f, max_x);
            sy = clamp_f32(sy, 0.0f, max_y);

            int ix = (int)sx;
            int iy = (int)sy;
            f32 fx_ = sx - ix;
            f32 fy_ = sy - iy;

            if (ix >= ideal.w - 1) { ix = ideal.w - 2; fx_ = 1.0f; }
            if (iy >= ideal.h - 1) { iy = ideal.h - 2; fy_ = 1.0f; }

            f32 v00 = ideal.get(ix,     iy);
            f32 v10 = ideal.get(ix + 1, iy);
            f32 v01 = ideal.get(ix,     iy + 1);
            f32 v11 = ideal.get(ix + 1, iy + 1);

            f32 top = v00 + fx_ * (v10 - v00);
            f32 bot = v01 + fx_ * (v11 - v01);
            f32 val = top + fy_ * (bot - top);

            distorted.set(u, v, (uint8_t)(val + 0.5f));
        }
    }
}
