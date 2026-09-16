#define _CRT_SECURE_NO_WARNINGS
#include "threshold.h"

int otsu_threshold(const GrayImage& src) {
    int hist[256] = {};
    int total = src.w * src.h;

    for (int y = 0; y < src.h; ++y)
        for (int x = 0; x < src.w; ++x)
            hist[src.get(x, y)]++;

    f64 sum_all = 0.0;
    for (int i = 0; i < 256; ++i)
        sum_all += i * (f64)hist[i];

    f64 sum_b = 0.0;
    int w_b = 0;
    f64 max_var = 0.0;
    int best_t = 0;

    for (int t = 0; t < 256; ++t) {
        w_b += hist[t];
        if (w_b == 0) continue;
        int w_f = total - w_b;
        if (w_f == 0) break;

        sum_b += t * (f64)hist[t];
        f64 m_b = sum_b / w_b;
        f64 m_f = (sum_all - sum_b) / w_f;
        f64 var = (f64)w_b * (f64)w_f * (m_b - m_f) * (m_b - m_f);

        if (var > max_var) {
            max_var = var;
            best_t = t;
        }
    }

    return best_t;
}

void binarize(const GrayImage& src, int threshold, GrayImage& dst) {
    dst.w = src.w;
    dst.h = src.h;
    dst.c = 1;
    delete[] dst.data;
    dst.data = new uint8_t[dst.w * dst.h]();

    for (int y = 0; y < src.h; ++y)
        for (int x = 0; x < src.w; ++x)
            dst.set(x, y, src.get(x, y) > threshold ? 255 : 0);
}