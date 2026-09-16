#define _CRT_SECURE_NO_WARNINGS
#include "integral_projection.h"
#include <algorithm>

void row_projection(const GrayImage& bin, std::vector<f32>& out) {
    out.assign(bin.h, 0.0f);
    for (int y = 0; y < bin.h; ++y) {
        f32 sum = 0.0f;
        for (int x = 0; x < bin.w; ++x)
            sum += (f32)bin.get(x, y);
        out[y] = sum;
    }
}

void col_projection(const GrayImage& bin, std::vector<f32>& out) {
    out.assign(bin.w, 0.0f);
    for (int x = 0; x < bin.w; ++x) {
        f32 sum = 0.0f;
        for (int y = 0; y < bin.h; ++y)
            sum += (f32)bin.get(x, y);
        out[x] = sum;
    }
}

void find_peaks(const std::vector<f32>& proj, std::vector<int>& peaks, f32 threshold_ratio) {
    f32 pmax = *std::max_element(proj.begin(), proj.end());
    f32 pmin = *std::min_element(proj.begin(), proj.end());
    f32 threshold = pmin + (pmax - pmin) * threshold_ratio;

    peaks.clear();
    int n = (int)proj.size();

    for (int i = 1; i < n - 1; ++i) {
        if (proj[i] > threshold && proj[i] >= proj[i - 1] && proj[i] >= proj[i + 1])
            peaks.push_back(i);
    }

    std::vector<int> filtered;
    int min_gap = n / 20;
    for (int p : peaks) {
        if (filtered.empty() || p - filtered.back() > min_gap) {
            filtered.push_back(p);
        } else if (proj[p] > proj[filtered.back()]) {
            filtered.back() = p;
        }
    }
    peaks = filtered;
}