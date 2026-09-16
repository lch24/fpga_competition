#pragma once
#include "../common/types.h"
#include <vector>

struct CameraParams;

struct CalibDistort {
    f64 k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
};

f64 radial_distort(f64 x_n, f64 y_n, const CalibDistort& d);

void project_distorted(f64 x_ideal, f64 y_ideal,
                       f64 fx, f64 fy, f64 cx, f64 cy,
                       const CalibDistort& d,
                       f64& x_d, f64& y_d);

struct LMResult {
    CalibDistort d;
    f64 final_error;
    int iterations;
    bool converged;
};

LMResult lm_calibrate_distort(
    const std::vector<Point2f>& image_pts,
    const std::vector<Point2f>& ideal_pts,
    f64 fx, f64 fy, f64 cx, f64 cy,
    const CalibDistort& init,
    int max_iter = 200,
    f64 tol = 1e-6
);