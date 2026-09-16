#pragma once
#include "../common/types.h"
#include "../common/image.h"
#include <vector>

void refine_subpixel(const GrayImage& img, std::vector<Point2f>& corners, int half_win);