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
