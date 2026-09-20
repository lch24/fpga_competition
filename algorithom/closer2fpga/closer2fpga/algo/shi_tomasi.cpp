#include "shi_tomasi.h"
#include <algorithm>
#include <cmath>
#include <vector>

static f32 safe_get(const FloatMap& m, int x, int y, f32 def) {
    if (x < 0 || x >= m.w || y < 0 || y >= m.h) return def;
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
            f32 tc = (f32)src.get(x,  ym);
            f32 tr = (f32)src.get(xp, ym);
            f32 ml = (f32)src.get(xm, y);
            f32 mr = (f32)src.get(xp, y);
            f32 bl = (f32)src.get(xm, yp);
            f32 bc = (f32)src.get(x,  yp);
            f32 br = (f32)src.get(xp, yp);

            f32 gx = -tl - 2.0f * ml - bl + tr + 2.0f * mr + br;
            f32 gy = -tl - 2.0f * tc - tr + bl + 2.0f * bc + br;

            Ix.set(x, y, gx);
            Iy.set(x, y, gy);
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
                    sum_Ixx += ix * ix;
                    sum_Iyy += iy * iy;
                    sum_Ixy += ix * iy;
                }
            }

            f32 det = sum_Ixx * sum_Iyy - sum_Ixy * sum_Ixy;
            f32 trace = sum_Ixx + sum_Iyy;
            f64 arg = (f64)trace * (f64)trace - 4.0 * (f64)det;
            if (arg < 0.0) arg = 0.0;
            f32 val = (f32)(trace - std::sqrt(arg)) * 0.5f;

            resp.set(x, y, val);
        }
    }
}

void shi_tomasi_detect(const GrayImage& src, std::vector<Point2f>& corners,
                       f32 threshold_ratio, int win_size) {
    corners.clear();
    if (!src.data || src.w < 5 || src.h < 5 || win_size < 3 || win_size % 2 == 0) return;
    FloatMap Ix, Iy, resp;
    sobel_xy(src, Ix, Iy);
    shi_tomasi_response(Ix, Iy, resp, win_size);

    f32 rmax = *std::max_element(resp.data, resp.data + resp.w * resp.h);
    if (!std::isfinite(rmax) || rmax <= 0) return;
    f32 thr = rmax * std::clamp(threshold_ratio, 0.001f, 1.0f);

    int r = win_size / 2;
    int nms = 3;
    int nr = nms / 2;

    for (int y = nr + r; y < resp.h - nr - r; ++y) {
        for (int x = nr + r; x < resp.w - nr - r; ++x) {
            f32 v = resp.get(x, y);
            if (v <= 0 || v < thr) continue;

            bool is_max = true;
            for (int dy = -nr; dy <= nr && is_max; ++dy)
                for (int dx = -nr; dx <= nr && is_max; ++dx)
                    if (dx != 0 || dy != 0)
                        if (resp.get(x + dx, y + dy) > v)
                            is_max = false;

            if (is_max) corners.push_back(Point2f((f32)x, (f32)y));
        }
    }
}
