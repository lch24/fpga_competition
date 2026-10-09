// Serialization fixtures, not calibration accuracy tests. No OpenCV required.
#include "../closer2fpga/desktop/calibration_export.h"
#include <iostream>
int main() {
    std::vector<desktop::CalibrationExportView> views(3);
    std::vector<std::vector<Point2f>> points(3, std::vector<Point2f>(40));
    for (int v=0;v<3;++v) {
        views[v]={"E:/测试/quoted\"name\\line\n.jpg",640,480,true,true};
        for(int i=0;i<40;++i)points[v][i]={float(i+.123456789+v),float(i*.75-v)};
    }
    points[0][0]={-0.0f,std::numeric_limits<float>::denorm_min()};
    CameraCalibrationResult r{};
    r.camera={800,820,320,240,.02f,-.005f,0,.001f,-.002f,true};
    r.converged=true;r.weak_geometry=true;r.iterations=25;
    r.rms=.12345678901234567;r.max_error=.67891234567890123;r.per_view_rms={.1,.2,.3};
    r.message="quotes\" backslash\\ newline\n tab\t";
    r.poses.resize(3);
    for(int v=0;v<3;++v) {
        for(int i=0;i<9;++i)r.poses[v].rotation[i]=(i%4==0)?1.0:0.0;
        r.poses[v].translation={v+.12345678901234567,-.0,25.0+v};
    }
    std::ofstream list("export_fixture_paths.txt");
    auto save=[&](const CameraCalibrationResult* result) {
        auto dir=desktop::export_calibration_run("export_fixture",views,points,640,480,5,8,25.,result);
        list<<dir.string()<<'\n';
    };
    save(&r);
    r.camera.valid=false;r.converged=false;save(&r);
    views[1].board_valid=false;points[1].resize(7);save(nullptr);
    points[0][0].x=std::numeric_limits<float>::infinity();
    r.rms=std::numeric_limits<double>::quiet_NaN();save(&r);
    list.close();
    // A regular file cannot be used as the export root; failures must throw.
    bool failed=false;
    try { desktop::export_calibration_run("export_fixture_paths.txt",views,points,640,480,5,8,25.,&r); }
    catch(const std::exception&) { failed=true; }
    if(!failed)return 1;
    std::cout<<"EXPORT_FIXTURE_WRITTEN cases=4 io_failure_checked=1\n";
}
