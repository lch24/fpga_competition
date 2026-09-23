#pragma once
#include "image.h"
#include <cstddef>
#include <limits>
#include <stdexcept>

// Borrowed CPU-visible frame buffer, not a physical DDR address or AXI port.
// Stride counts elements (bytes for uint8_t, floats for float maps).
// Caller owns storage and guarantees its lifetime/capacity. No allocation here.
template <class T> struct ImageView {
    T* data = nullptr;
    int w = 0, h = 0, c = 1;
    size_t stride = 0;
    T* row(int y) const {
        return data + size_t(y) * stride;
    }
    bool valid() const {
        if (!data || w <= 0 || h <= 0 || c <= 0)
            return false;
        size_t limit = std::numeric_limits<size_t>::max() / sizeof(T);
        if (size_t(w) > limit / size_t(c))
            return false;
        size_t width = size_t(w) * size_t(c);
        return stride >= width && size_t(h - 1) <= (limit - width) / stride;
    }
};

template <class T> ImageView<T> image_view(Image<T>& image) {
    return {image.data, image.w, image.h, image.c, size_t(image.w) * image.c};
}
template <class T> ImageView<const T> image_view(const Image<T>& image) {
    return {image.data, image.w, image.h, image.c, size_t(image.w) * image.c};
}

// Conservative span check includes row padding; borrowed operations are out-of-place.
template <class A, class B> bool overlaps(ImageView<A> a, ImageView<B> b) {
    auto start_a = reinterpret_cast<uintptr_t>(a.data);
    auto start_b = reinterpret_cast<uintptr_t>(b.data);
    size_t bytes_a = ((size_t(a.h) - 1) * a.stride + size_t(a.w) * a.c) * sizeof(A);
    size_t bytes_b = ((size_t(b.h) - 1) * b.stride + size_t(b.w) * b.c) * sizeof(B);
    return start_a <= start_b ? start_b - start_a < bytes_a : start_a - start_b < bytes_b;
}
