#pragma once
#include "../common/image.h"
#include "../common/types.h"
#include <vector>

void sobel_xy(const GrayImage& src, FloatMap& Ix, FloatMap& Iy);

void shi_tomasi_response(const FloatMap& Ix, const FloatMap& Iy, FloatMap& resp, int win_size = 3);

void shi_tomasi_detect(const GrayImage& src, std::vector<Point2f>& corners, f32 threshold_ratio = 0.3f,
                       int win_size = 3);