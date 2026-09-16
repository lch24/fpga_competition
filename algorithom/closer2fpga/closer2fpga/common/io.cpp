#define _CRT_SECURE_NO_WARNINGS
#include "io.h"
#include <cstdio>
#include <cctype>
#include <cstdint>

static bool skip_ws_and_comments(FILE* f) {
    int c;
    while ((c = fgetc(f)) != EOF) {
        if (c == '#') {
            while ((c = fgetc(f)) != EOF && c != '\n');
        } else if (!std::isspace(c)) {
            ungetc(c, f);
            return true;
        }
    }
    return false;
}

static bool read_ppm_header(FILE* f, int& w, int& h, int& maxval, bool& is_binary) {
    char magic[3] = {0};
    if (fscanf(f, "%2s", magic) != 1) return false;
    if (magic[0] != 'P') return false;
    int type = magic[1] - '0';
    if (type != 5 && type != 6) return false;
    is_binary = true;

    skip_ws_and_comments(f);
    if (fscanf(f, "%d", &w) != 1) return false;
    skip_ws_and_comments(f);
    if (fscanf(f, "%d", &h) != 1) return false;
    skip_ws_and_comments(f);
    if (fscanf(f, "%d", &maxval) != 1) return false;
    fgetc(f);
    return true;
}

bool load_pgm(const char* path, GrayImage& out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    int w, h, maxval;
    bool binary;
    if (!read_ppm_header(f, w, h, maxval, binary)) { fclose(f); return false; }

    delete[] out.data;
    out.w = w;
    out.h = h;
    out.c = 1;
    out.data = new uint8_t[w * h]();

    if (binary) {
        if (maxval > 255) {
            uint16_t* tmp = new uint16_t[w * h];
            fread(tmp, 2, w * h, f);
            for (int i = 0; i < w * h; ++i)
                out.data[i] = (uint8_t)(tmp[i] * 255 / maxval);
            delete[] tmp;
        } else {
            fread(out.data, 1, w * h, f);
        }
    } else {
        if (maxval > 255) {
            for (int i = 0; i < w * h; ++i) {
                int v; fscanf(f, "%d", &v);
                out.data[i] = (uint8_t)(v * 255 / maxval);
            }
        } else {
            for (int i = 0; i < w * h; ++i) {
                int v; fscanf(f, "%d", &v);
                out.data[i] = (uint8_t)v;
            }
        }
    }

    fclose(f);
    return true;
}

bool load_ppm(const char* path, RgbImage& out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    int w, h, maxval;
    bool binary;
    if (!read_ppm_header(f, w, h, maxval, binary)) { fclose(f); return false; }

    delete[] out.data;
    out.w = w;
    out.h = h;
    out.c = 3;
    out.data = new uint8_t[w * h * 3]();

    int npixels = w * h;
    if (binary) {
        if (maxval > 255) {
            uint16_t* tmp = new uint16_t[npixels * 3];
            fread(tmp, 2, npixels * 3, f);
            for (int i = 0; i < npixels * 3; ++i)
                out.data[i] = (uint8_t)(tmp[i] * 255 / maxval);
            delete[] tmp;
        } else {
            fread(out.data, 1, npixels * 3, f);
        }
    } else {
        if (maxval > 255) {
            for (int i = 0; i < npixels * 3; ++i) {
                int v; fscanf(f, "%d", &v);
                out.data[i] = (uint8_t)(v * 255 / maxval);
            }
        } else {
            for (int i = 0; i < npixels * 3; ++i) {
                int v; fscanf(f, "%d", &v);
                out.data[i] = (uint8_t)v;
            }
        }
    }

    fclose(f);
    return true;
}

bool save_pgm(const char* path, const GrayImage& in) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P5\n%d %d\n255\n", in.w, in.h);
    fwrite(in.data, 1, in.w * in.h, f);
    fclose(f);
    return true;
}

bool save_ppm(const char* path, const RgbImage& in) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", in.w, in.h);
    fwrite(in.data, 1, in.w * in.h * 3, f);
    fclose(f);
    return true;
}