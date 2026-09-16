#pragma once
#include "image.h"

bool load_pgm(const char* path, GrayImage& out);
bool load_ppm(const char* path, RgbImage& out);
bool save_pgm(const char* path, const GrayImage& in);
bool save_ppm(const char* path, const RgbImage& in);