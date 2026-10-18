// Plain-text UART corner replay through the production C++ calibration path.
// Input: width height rows cols views, followed by view-major x y pairs.
#include "../../../algorithom/closer2fpga/closer2fpga/algo/calibrate.h"
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    std::ifstream in(argv[1]);
    int w, h, rows, cols, views;
    if (!(in >> w >> h >> rows >> cols >> views) || rows<3 || cols<3 || views<3) return 2;
    std::vector<std::vector<Point2f>> points(views, std::vector<Point2f>(rows*cols));
    for (auto& view : points) for (auto& p : view) if (!(in >> p.x >> p.y)) return 2;
    const auto r = calibrate_camera(points, w, h, rows, cols);
    const auto& k = r.camera;
    std::cout << std::setprecision(17)
        << "{\"valid\":" << (k.valid ? "true" : "false")
        << ",\"converged\":" << (r.converged ? "true" : "false")
        << ",\"rms\":" << r.rms << ",\"iterations\":" << r.iterations
        << ",\"camera\":{\"fx\":" << k.fx << ",\"fy\":" << k.fy
        << ",\"cx\":" << k.cx << ",\"cy\":" << k.cy
        << ",\"k1\":" << k.k1 << ",\"k2\":" << k.k2 << ",\"k3\":" << k.k3
        << ",\"p1\":" << k.p1 << ",\"p2\":" << k.p2 << "}}\n";
    return 0;
}
