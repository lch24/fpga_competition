#pragma once
#include "types.h"

struct LMtx {
    int rows, cols;
    f64* data;

    LMtx() : rows(0), cols(0), data(nullptr) {}
    LMtx(int r, int c) : rows(r), cols(c), data(new f64[r * c]()) {}

    LMtx(const LMtx&) = delete;
    LMtx& operator=(const LMtx&) = delete;

    LMtx(LMtx&& o) noexcept : rows(o.rows), cols(o.cols), data(o.data) {
        o.data = nullptr; o.rows = 0; o.cols = 0;
    }
    LMtx& operator=(LMtx&& o) noexcept {
        if (this != &o) {
            delete[] data;
            rows = o.rows; cols = o.cols; data = o.data;
            o.data = nullptr; o.rows = 0; o.cols = 0;
        }
        return *this;
    }

    ~LMtx() { delete[] data; }

    f64& at(int r, int c) { return data[r * cols + c]; }
    f64 at(int r, int c) const { return data[r * cols + c]; }

};

// Partial-pivot Gaussian elimination; returns an empty matrix on failure.
LMtx solve_gauss(LMtx A, LMtx b);