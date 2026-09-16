#define _CRT_SECURE_NO_WARNINGS
#include "undistort.h"
#include <cmath>

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

void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst) {
    dst.w = table.w;
    dst.h = table.h;
    dst.c = 1;
    delete[] dst.data;
    dst.data = new uint8_t[dst.w * dst.h]();

    f32 max_x = (f32)(src.w - 1);
    f32 max_y = (f32)(src.h - 1);

    for (int y = 0; y < table.h; ++y) {
        for (int x = 0; x < table.w; ++x) {
            f32 sx = table.map_x.get(x, y);
            f32 sy = table.map_y.get(x, y);

            sx = clamp_f32(sx, 0.0f, max_x);
            sy = clamp_f32(sy, 0.0f, max_y);

            int ix = (int)sx;
            int iy = (int)sy;
            f32 fx_ = sx - ix;
            f32 fy_ = sy - iy;

            if (ix >= src.w - 1) { ix = src.w - 2; fx_ = 1.0f; }
            if (iy >= src.h - 1) { iy = src.h - 2; fy_ = 1.0f; }

            f32 v00 = src.get(ix,     iy);
            f32 v10 = src.get(ix + 1, iy);
            f32 v01 = src.get(ix,     iy + 1);
            f32 v11 = src.get(ix + 1, iy + 1);

            f32 top = v00 + fx_ * (v10 - v00);
            f32 bot = v01 + fx_ * (v11 - v01);
            f32 val = top + fy_ * (bot - top);

            dst.set(x, y, (uint8_t)(val + 0.5f));
        }
    }
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