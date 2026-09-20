#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>
#include "../closer2fpga/algo/calibrate.h"
#include "../closer2fpga/algo/chessboard.h"
#include "../closer2fpga/algo/undistort.h"

GrayImage gray_image(const cv::Mat& image) {
    GrayImage gray(image.cols,image.rows);
    for (int y=0;y<gray.h;++y) for (int x=0;x<gray.w;++x) gray.set(x,y,image.at<uint8_t>(y,x));
    return gray;
}
void report(const char* label,const CameraCalibrationResult& r) {
    const auto& k=r.camera;
    std::printf("%s valid=%d converged=%d rms=%.8f max=%.5f iterations=%d\n",label,k.valid,r.converged,r.rms,r.max_error,r.iterations);
    std::printf("  K: %.8f %.8f %.8f %.8f\n  D: %.8f %.8f %.8f %.8f %.8f\n  %s\n",k.fx,k.fy,k.cx,k.cy,k.k1,k.k2,k.p1,k.p2,k.k3,r.message.c_str());
    for (size_t i=0;i<r.per_view_rms.size();++i) std::printf("  view%zu RMS %.8f\n",i,r.per_view_rms[i]);
}
int main(int argc,char** argv) {
    int failures=0;
    std::vector<cv::Point3f> object;
    for (int r=0;r<5;++r) for (int c=0;c<8;++c) object.push_back({float(c),float(r),0});
    cv::Mat truthK=(cv::Mat_<double>(3,3)<<900,0,630,0,920,355,0,0,1);
    cv::Mat truthD=(cv::Mat_<double>(1,5)<<-.22,.075,.002,-.001,0);
    std::vector<cv::Vec3d> rotations{{.25,-.3,.05},{-.3,.2,-.12},{.15,.4,.18},{-.2,-.4,.3},{.4,.05,-.3},{-.1,.25,.2}};
    std::vector<cv::Vec3d> translations{{-3.5,-2,12},{-3,-2,14},{-4,-2,13},{-3.2,-1.8,12.5},{-3,-2,13.5},{-3.8,-1.5,11.5}};
    std::vector<std::vector<Point2f>> synthetic;
    for (int i=0;i<6;++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object,rotations[i],translations[i],truthK,truthD,projected);
        std::vector<Point2f> points;
        for (auto p:projected) points.push_back({p.x,p.y});
        synthetic.push_back(points);
    }
    std::vector<std::vector<Point2f>> three(synthetic.begin(),synthetic.begin()+3);
    auto result=calibrate_camera(three,1280,720,5,8);
    report("synthetic3",result);
    if (!result.camera.valid || result.rms>.001 || std::fabs(result.camera.fx-900)>.1 ||
        std::fabs(result.camera.fy-920)>.1 || std::fabs(result.camera.cx-630)>.1 ||
        std::fabs(result.camera.cy-355)>.1 || std::fabs(result.camera.k1+.22)>.001 ||
        std::fabs(result.camera.k2-.075)>.001 || std::fabs(result.camera.p1-.002)>.0001 ||
        std::fabs(result.camera.p2+.001)>.0001 || result.camera.k3!=0) ++failures;
    auto scaled=calibrate_camera(three,1280,720,5,8,25.0);
    double translation_error=0;
    for (int i=0;i<3;++i) for (int axis=0;axis<3;++axis)
        translation_error=std::max(translation_error,std::fabs(scaled.poses[i].translation[axis]-translations[i][axis]*25));
    std::printf("square-size translation error %.8f\n",translation_error);
    if (!scaled.camera.valid || translation_error>.01 || std::fabs(scaled.camera.fx-result.camera.fx)>.001) ++failures;
    auto flipped=three;
    for (int r=0;r<5;++r) for (int c=0;c<8;++c) flipped[1][r*8+c]=three[1][r*8+7-c];
    auto flip_fit=calibrate_camera(flipped,1280,720,5,8);
    if (!flip_fit.camera.valid || std::fabs(flip_fit.camera.fx-900)>.1 || flip_fit.rms>.001) ++failures;
    auto noisy=synthetic;
    cv::RNG rng(24680);
    for (auto& frame:noisy) for (auto& p:frame) {p.x+=float(rng.gaussian(.15)); p.y+=float(rng.gaussian(.15));}
    auto noisy_fit=calibrate_camera(noisy,1280,720,5,8);
    report("synthetic6 noisy",noisy_fit);
    if (!noisy_fit.camera.valid || noisy_fit.rms>.3 || std::fabs(noisy_fit.camera.fx-900)>27) ++failures;
    // A fifth coefficient is supported explicitly, but not enabled for the
    // three-image demo. Project independently with OpenCV for this oracle.
    truthD.at<double>(0,4)=.04;
    std::vector<std::vector<Point2f>> five_term;
    for (int i=0;i<6;++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object,rotations[i],translations[i],truthK,truthD,projected);
        std::vector<Point2f> frame;
        for (auto p:projected) frame.push_back({p.x,p.y});
        five_term.push_back(frame);
    }
    CameraCalibrationOptions all_terms; all_terms.estimate_k3=true;
    auto five_fit=calibrate_camera(five_term,1280,720,5,8,1,all_terms);
    report("synthetic6 k3",five_fit);
    if (!five_fit.camera.valid || five_fit.rms>.001 || std::fabs(five_fit.camera.k3-.04)>.005) ++failures;
    // Failure paths must never authorize remapping.
    if (calibrate_camera({three[0],three[1]},1280,720,5,8).camera.valid) ++failures;
    if (calibrate_camera({three[0],three[0],three[0]},1280,720,5,8).camera.valid) ++failures;
    auto broken=three; broken[1].pop_back();
    if (calibrate_camera(broken,1280,720,5,8).camera.valid) ++failures;
    broken=three; broken[0][0].x=std::numeric_limits<float>::quiet_NaN();
    if (calibrate_camera(broken,1280,720,5,8).camera.valid) ++failures;
    if (calibrate_camera(three,1280,720,5,8,-1).camera.valid) ++failures;
    CameraCalibrationOptions no_iterations; no_iterations.max_iterations=1;
    if (calibrate_camera(three,1280,720,5,8,1,no_iterations).camera.valid) ++failures;
    std::string root=argc>1 ? argv[1]:"E:/fpga/algorithom";
    std::vector<std::vector<Point2f>> real;
    std::vector<std::vector<cv::Point2f>> cvpoints;
    for (int i=0;i<3;++i) {
        cv::Mat image=cv::imread(root+"/test"+std::to_string(i)+".jpg");
        if (image.empty()) return 2;
        GrayImage gray(image.cols,image.rows);
        for (int y=0;y<gray.h;++y) for (int x=0;x<gray.w;++x) {
            auto bgr=image.at<cv::Vec3b>(y,x);
            gray.set(x,y,uint8_t((299*bgr[2]+587*bgr[1]+114*bgr[0]+500)/1000));
        }
        auto board=detect_chessboard(gray,5,8);
        if (!board.valid) return 2;
        real.push_back(board.corners);
        std::vector<cv::Point2f> pts;
        for (auto p:board.corners) pts.push_back({p.x,p.y});
        cvpoints.push_back(pts);
    }
    auto fitted=calibrate_camera(real,1280,720,5,8);
    report("real3",fitted);
    cv::Mat k,d;
    std::vector<cv::Mat> rvecs,tvecs;
    double cv_rms=cv::calibrateCamera(std::vector<std::vector<cv::Point3f>>(3,object),cvpoints,{1280,720},k,d,rvecs,tvecs,cv::CALIB_FIX_K3,
        cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,300,1e-12));
    std::printf("OpenCV reference (same measured corners) RMS %.8f\n",cv_rms);
    std::cout<<"K "<<k<<"\nD "<<d<<"\n";
    if (!fitted.camera.valid || std::fabs(fitted.rms-cv_rms)>.03) ++failures;
    if (std::fabs(fitted.camera.fx-k.at<double>(0,0))>.5 ||
        std::fabs(fitted.camera.cy-k.at<double>(1,2))>.5) ++failures;
    auto camera=fitted.camera;
    auto table=build_remap_table(1280,720,camera);
    cv::Mat camera_matrix=(cv::Mat_<double>(3,3)<<camera.fx,0,camera.cx,0,camera.fy,camera.cy,0,0,1);
    cv::Mat distortion=(cv::Mat_<double>(1,5)<<camera.k1,camera.k2,camera.p1,camera.p2,camera.k3);
    cv::Mat reference_x,reference_y;
    cv::initUndistortRectifyMap(camera_matrix,distortion,cv::Mat(),camera_matrix,{1280,720},CV_32FC1,reference_x,reference_y);
    double map_error=0;
    for (int y=0;y<720;++y) for (int x=0;x<1280;++x) {
        map_error=std::max(map_error,double(std::fabs(table.map_x.get(x,y)-reference_x.at<float>(y,x))));
        map_error=std::max(map_error,double(std::fabs(table.map_y.get(x,y)-reference_y.at<float>(y,x))));
    }
    std::printf("remap coordinate maximum difference %.8f px\n",map_error);
    if (map_error>.0005) ++failures;
    for (int i=0;i<3;++i) {
        auto color=cv::imread(root+"/test"+std::to_string(i)+".jpg");
        GrayImage source(color.cols,color.rows,3);
        for (int y=0;y<color.rows;++y) for (int x=0;x<color.cols;++x) for (int c=0;c<3;++c)
            source.data[(y*color.cols+x)*3+c]=color.at<cv::Vec3b>(y,x)[c];
        GrayImage corrected;
        remap_bilinear(source,table,corrected,RemapBorder::ConstantBlack);
        cv::Mat actual(color.rows,color.cols,CV_8UC3,corrected.data),reference;
        cv::remap(color,reference,reference_x,reference_y,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        double mae=cv::norm(actual,reference,cv::NORM_L1)/(color.total()*3);
        double maxdiff=cv::norm(actual,reference,cv::NORM_INF);
        std::printf("test%d correction vs OpenCV MAE %.6f max %.0f\n",i,mae,maxdiff);
        if (mae>.25 || maxdiff>8) ++failures;
        cv::imwrite("calibration_test"+std::to_string(i)+"_undistorted.png",actual);
    }
    CameraParams identity{}; identity.fx=300; identity.fy=300; identity.cx=1; identity.cy=1;
    auto identity_map=build_remap_table(3,2,identity);
    for (int channels:{1,3}) {
        GrayImage source(3,2,channels);
        for (int i=0;i<source.total();++i) source.data[i]=uint8_t(i*11);
        remap_bilinear(source,identity_map,source,RemapBorder::ConstantBlack);
        for (int i=0;i<source.total();++i) if (source.data[i]!=uint8_t(i*11)) ++failures;
    }
    auto one_map=build_remap_table(1,1,identity);
    GrayImage one(1,1,3); one.data[0]=10; one.data[1]=100; one.data[2]=200;
    GrayImage one_out;
    remap_bilinear(one,one_map,one_out);
    for (int c=0;c<3;++c) if (one.data[c]!=one_out.data[c]) ++failures;
    one_map.map_x.set(0,0,-.5f); one_map.map_y.set(0,0,0);
    remap_bilinear(one,one_map,one_out,RemapBorder::ConstantBlack);
    for (int c=0;c<3;++c) if (one_out.data[c]!=one.data[c]/2) ++failures;
    one_map.map_x.set(0,0,std::numeric_limits<float>::quiet_NaN());
    remap_bilinear(one,one_map,one_out);
    for (int c=0;c<3;++c) if (one_out.data[c]!=0) ++failures;
    try { GrayImage empty; remap_bilinear(empty,one_map,one_out); ++failures; }
    catch (const std::invalid_argument&) {}
    try { auto invalid=identity; invalid.fx=0; build_remap_table(2,2,invalid); ++failures; }
    catch (const std::invalid_argument&) {}
    std::printf("FAILURES %d\n",failures);
    return failures ? 1:0;
}
