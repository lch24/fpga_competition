#pragma once
#include "../common/types.h"
#include <cstdio>
#include <cstring>
#include <utility>

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

    LMtx T() const {
        LMtx t(cols, rows);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                t.at(c, r) = at(r, c);
        return t;
    }

    LMtx operator*(const LMtx& B) const {
        LMtx C(rows, B.cols);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < B.cols; ++c) {
                f64 s = 0;
                for (int k = 0; k < cols; ++k)
                    s += at(r, k) * B.at(k, c);
                C.at(r, c) = s;
            }
        return C;
    }

    LMtx operator+(const LMtx& B) const {
        LMtx C(rows, cols);
        for (int i = 0; i < rows * cols; ++i)
            C.data[i] = data[i] + B.data[i];
        return C;
    }

    LMtx operator-(const LMtx& B) const {
        LMtx C(rows, cols);
        for (int i = 0; i < rows * cols; ++i)
            C.data[i] = data[i] - B.data[i];
        return C;
    }

    LMtx operator*(f64 s) const {
        LMtx C(rows, cols);
        for (int i = 0; i < rows * cols; ++i)
            C.data[i] = data[i] * s;
        return C;
    }

    void print(const char* name = "") const {
        std::printf("%s (%dx%d)\n", name, rows, cols);
        for (int r = 0; r < rows; ++r) {
            std::printf("  [");
            for (int c = 0; c < cols; ++c)
                std::printf("% .6f ", at(r, c));
            std::printf("]\n");
        }
    }
};

LMtx solve_gauss(LMtx A, LMtx b);