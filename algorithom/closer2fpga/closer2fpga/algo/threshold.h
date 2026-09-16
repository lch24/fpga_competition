#pragma once
#include "../common/image.h"

int otsu_threshold(const GrayImage& src);

void binarize(const GrayImage& src, int threshold, GrayImage& dst);