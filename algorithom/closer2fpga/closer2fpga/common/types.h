#pragma once

using f32 = float;
using f64 = double;

struct Point2f  { f32 x, y; };
struct Point2i  { int x, y; };
struct Point3f  { f32 x, y, z; };

struct CameraParams {
    f32 fx, fy;
    f32 cx, cy;
    f32 k1, k2, k3;
    f32 p1, p2;
    bool valid;
};