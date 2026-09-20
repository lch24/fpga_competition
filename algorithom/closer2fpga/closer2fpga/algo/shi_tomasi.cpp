#include "shi_tomasi.h"
#include "../kernels/gradient.h"
#include <algorithm>
#include <cmath>
#include <vector>

static f32 safe_get(const FloatMap& m, int x, int y, f32 def) {
    if (x < 0 || x >= m.w || y < 0 || y >= m.h)
        return def;
    return m.get(x, y);
}

void sobel_xy(const GrayImage& src, FloatMap& Ix, FloatMap& Iy) {
    Ix = FloatMap(src.w, src.h);
    Iy = FloatMap(src.w, src.h);

    for (int y = 0; y < src.h; ++y) {
        for (int x = 0; x < src.w; ++x) {
            int xm = std::max(0, x - 1);
            int xp = std::min(src.w - 1, x + 1);
            int ym = std::max(0, y - 1);
            int yp = std::min(src.h - 1, y + 1);

            f32 tl = (f32)src.get(xm, ym);
            f32 tc = (f32)src.get(x, ym);
            f32 tr = (f32)src.get(xp, ym);
            f32 ml = (f32)src.get(xm, y);
            f32 mr = (f32)src.get(xp, y);
            f32 bl = (f32)src.get(xm, yp);
            f32 bc = (f32)src.get(x, yp);
            f32 br = (f32)src.get(xp, yp);

            auto gradient = kernels::sobel(tl, tc, tr, ml, mr, bl, bc, br);
            Ix.set(x, y, gradient.x);
            Iy.set(x, y, gradient.y);
        }
    }
}

void shi_tomasi_response(const FloatMap& Ix, const FloatMap& Iy, FloatMap& resp, int win_size) {
    int r = win_size / 2;
    resp = FloatMap(Ix.w, Ix.h);

    for (int y = 0; y < Ix.h; ++y) {
        for (int x = 0; x < Ix.w; ++x) {
            f32 sum_Ixx = 0.0f, sum_Iyy = 0.0f, sum_Ixy = 0.0f;

            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    f32 ix = safe_get(Ix, x + dx, y + dy, 0.0f);
                    f32 iy = safe_get(Iy, x + dx, y + dy, 0.0f);
                    auto tensor = kernels::outer_product(ix, iy);
                    sum_Ixx += tensor.xx;
                    sum_Iyy += tensor.yy;
                    sum_Ixy += tensor.xy;
                }
            }

            resp.set(x, y, kernels::min_eigenvalue(sum_Ixx, sum_Ixy, sum_Iyy));
        }
    }
}

void shi_tomasi_detect(const GrayImage& src, std::vector<Point2f>& corners, f32 threshold_ratio,
                       int win_size) {
    corners.clear();
    if (!src.data || src.w < 5 || src.h < 5 || win_size < 3 || win_size % 2 == 0)
        return;
    FloatMap Ix, Iy, resp;
    // Pass 1: produce/store responses. Window arithmetic is in kernels/gradient.h.
    sobel_xy(src, Ix, Iy);
    shi_tomasi_response(Ix, Iy, resp, win_size);

    f32 rmax = *std::max_element(resp.data, resp.data + resp.w * resp.h);
    // Frame barrier: the threshold depends on every response in this frame.
    if (!std::isfinite(rmax) || rmax <= 0)
        return;
    f32 thr = rmax * std::clamp(threshold_ratio, 0.001f, 1.0f);

    int r = win_size / 2;
    int nms = 3;
    int nr = nms / 2;

    // Pass 2: read stored responses for thresholding and local NMS.

    for (int y = nr + r; y < resp.h - nr - r; ++y) {
        for (int x = nr + r; x < resp.w - nr - r; ++x) {
            f32 v = resp.get(x, y);
            if (v <= 0 || v < thr)
                continue;

            bool is_max = true;
            for (int dy = -nr; dy <= nr && is_max; ++dy)
                for (int dx = -nr; dx <= nr && is_max; ++dx)
                    if (dx != 0 || dy != 0)
                        if (resp.get(x + dx, y + dy) > v)
                            is_max = false;

            if (is_max)
                corners.push_back(Point2f((f32)x, (f32)y));
        }
    }
}
