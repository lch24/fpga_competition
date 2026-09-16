#define _CRT_SECURE_NO_WARNINGS
#include "subpixel.h"
#include <vector>
#include <cmath>

void refine_subpixel(const GrayImage& img, std::vector<Point2f>& corners, int half_win) {
    for (auto& c : corners) {
        int cx = (int)(c.x + 0.5f);
        int cy = (int)(c.y + 0.5f);

        int x0 = cx - half_win;
        int y0 = cy - half_win;
        int x1 = cx + half_win;
        int y1 = cy + half_win;

        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 >= img.w) x1 = img.w - 1;
        if (y1 >= img.h) y1 = img.h - 1;

        f32 sum_all = 0;
        int cnt = 0;
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                sum_all += (f32)img.get(x, y);
                cnt++;
            }
        }
        f32 mean = sum_all / (f32)cnt;

        f32 sx_hi = 0, sy_hi = 0, w_hi = 0;
        f32 sx_lo = 0, sy_lo = 0, w_lo = 0;

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                f32 v = (f32)img.get(x, y);
                if (v >= mean) {
                    f32 w = v - mean;
                    sx_hi += (f32)x * w;
                    sy_hi += (f32)y * w;
                    w_hi += w;
                } else {
                    f32 w = mean - v;
                    sx_lo += (f32)x * w;
                    sy_lo += (f32)y * w;
                    w_lo += w;
                }
            }
        }

        f32 hx, hy, lx, ly;
        if (w_hi > 1e-3f) { hx = sx_hi / w_hi; hy = sy_hi / w_hi; }
        else { hx = (f32)cx; hy = (f32)cy; }
        if (w_lo > 1e-3f) { lx = sx_lo / w_lo; ly = sy_lo / w_lo; }
        else { lx = (f32)cx; ly = (f32)cy; }

        c.x = (hx + lx) * 0.5f;
        c.y = (hy + ly) * 0.5f;
    }
}