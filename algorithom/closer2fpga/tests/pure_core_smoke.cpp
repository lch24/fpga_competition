#include "../closer2fpga/algo/calibrate.h"
#include "../closer2fpga/algo/undistort.h"
#include <cmath>
#include <cstdio>

// This executable is built without OpenCV headers or libraries. Its input is
// analytically generated using elementary Euler rotations, independent of the
// Rodrigues implementation in the solver.
int main() {
    std::vector<std::vector<Point2f>> frames;
    const double angles[3][2]={{.3,-.25},{-.25,.35},{.4,.2}};
    for (int i=0;i<3;++i) {
        std::vector<Point2f> frame;
        double a=angles[i][0],b=angles[i][1];
        for (int r=0;r<5;++r) for (int c=0;c<8;++c) {
            double X=c-3.5,Y=r-2.;
            double x=std::cos(b)*X;
            double y=std::sin(a)*std::sin(b)*X+std::cos(a)*Y;
            double z=-std::cos(a)*std::sin(b)*X+std::sin(a)*Y+13+i;
            x/=z; y/=z;
            double rr=x*x+y*y,radial=1-.15*rr+.03*rr*rr;
            frame.push_back({float(560*x*radial+319),float(575*y*radial+240)});
        }
        frames.push_back(frame);
    }
    auto fit=calibrate_camera(frames,640,480,5,8);
    if (!fit.camera.valid || fit.rms>.001 || std::fabs(fit.camera.fx-560)>.2 ||
        std::fabs(fit.camera.k1+.15)>.001) return 1;
    auto table=build_remap_table(640,480,fit.camera);
    GrayImage source(640,480,3),corrected;
    for (int i=0;i<source.total();++i) source.data[i]=uint8_t(i%251);
    remap_bilinear(source,table,corrected,RemapBorder::ConstantBlack);
    if (corrected.w!=640 || corrected.h!=480 || corrected.c!=3) return 1;
    std::printf("Pure C++ calibration/remap: PASS (RMS %.8f px; no OpenCV linked)\n",fit.rms);
    return 0;
}
