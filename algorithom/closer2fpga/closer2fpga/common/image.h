#pragma once
#include <cstdlib>
#include <cstdint>
#include "types.h"

template <typename T>
class Image {
public:
    int w, h, c;
    T* data;

    Image() : w(0), h(0), c(0), data(nullptr) {}

    Image(int width, int height, int channels = 1) : w(width), h(height), c(channels) {
        data = new T[w * h * c]();
    }

    ~Image() {
        delete[] data;
    }

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    Image(Image&& o) noexcept : w(o.w), h(o.h), c(o.c), data(o.data) {
        o.w = 0; o.h = 0; o.c = 0; o.data = nullptr;
    }
    Image& operator=(Image&& o) noexcept {
        if (this != &o) {
            delete[] data;
            w = o.w; h = o.h; c = o.c; data = o.data;
            o.w = 0; o.h = 0; o.c = 0; o.data = nullptr;
        }
        return *this;
    }

    T& at(int x, int y)        { return data[y * w + x]; }
    T  get(int x, int y) const { return data[y * w + x]; }
    void set(int x, int y, T v){ data[y * w + x] = v; }

    int total() const { return w * h * c; }
};

using GrayImage = Image<uint8_t>;
using RgbImage  = Image<uint8_t>;
using FloatMap  = Image<f32>;