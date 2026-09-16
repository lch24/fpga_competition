#pragma once
#include "../common/image.h"
#include <vector>

void row_projection(const GrayImage& bin, std::vector<f32>& out);
void col_projection(const GrayImage& bin, std::vector<f32>& out);

void find_peaks(const std::vector<f32>& proj, std::vector<int>& peaks, f32 threshold_ratio = 0.3f);