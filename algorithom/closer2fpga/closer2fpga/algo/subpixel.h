#pragma once
#include "../common/types.h"
#include "../common/image.h"
#include <vector>

// Iterative weighted gradient intersection. Unsupported/nonconvergent points
// remain unchanged; half_win is clamped to [2, 15]. No OpenCV dependency.
void refine_subpixel(const GrayImage& img, std::vector<Point2f>& corners, int half_win);
