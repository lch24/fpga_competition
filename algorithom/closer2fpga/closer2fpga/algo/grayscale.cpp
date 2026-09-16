#define _CRT_SECURE_NO_WARNINGS
#include "grayscale.h"
#include <cstdint>

void rgb_to_gray(const RgbImage& in, GrayImage& out) {
    out.w = in.w;
    out.h = in.h;
    out.c = 1;
    delete[] out.data;
    out.data = new uint8_t[out.w * out.h]();

    int npixels = in.w * in.h;
    for (int i = 0; i < npixels; ++i) {
        const uint8_t* p = &in.data[i * 3];
        float y = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
        out.data[i] = (uint8_t)(y + 0.5f);
    }
}