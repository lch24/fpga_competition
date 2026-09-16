#pragma once
#include "../common/types.h"
#include "../common/image.h"

struct RemapTable {
    int w, h;
    FloatMap map_x;
    FloatMap map_y;
};

RemapTable build_remap_table(int w, int h, const CameraParams& cam);

void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst);

void forward_distort_norm(f32 nx, f32 ny, const CameraParams& cam, f32& xd, f32& yd);

void inverse_distort_norm(const CameraParams& cam, f32 xd, f32 yd, f32& nx, f32& ny);

void make_distorted_image(const GrayImage& ideal, const CameraParams& cam, GrayImage& distorted);