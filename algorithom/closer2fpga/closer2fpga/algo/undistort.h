#pragma once
#include "../common/types.h"
#include "../common/image.h"

struct RemapTable {
    int w = 0, h = 0;
    FloatMap map_x;
    FloatMap map_y;
};

RemapTable build_remap_table(int w, int h, const CameraParams& cam);

enum class RemapBorder { Replicate, ConstantBlack };

// Supports one-channel gray or three-channel interleaved color. Channel order
// is preserved. Source and destination may refer to the same Image object.
void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst,
    RemapBorder border = RemapBorder::Replicate);

void forward_distort_norm(f32 nx, f32 ny, const CameraParams& cam, f32& xd, f32& yd);

void inverse_distort_norm(const CameraParams& cam, f32 xd, f32 yd, f32& nx, f32& ny);

void make_distorted_image(const GrayImage& ideal, const CameraParams& cam, GrayImage& distorted);
