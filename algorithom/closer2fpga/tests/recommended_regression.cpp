#include "../closer2fpga/algo/calibrate.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

// No OpenCV: real exported points and independent Euler/Brown synthetic data.
// Single-algorithm regression; golden data comes from the validated prior run.
namespace fs = std::filesystem;
struct Dataset {
    std::string name;
    int w=1280, h=720, rows=5, cols=8;
    std::vector<std::vector<Point2f>> points;
    CameraParams truth{};
    bool synthetic=false;
};
static std::array<double,2> distort(const CameraParams& k, double x, double y) {
    double r=x*x+y*y, s=1+k.k1*r+k.k2*r*r+k.k3*r*r*r;
    return {k.fx*(x*s+2*k.p1*x*y+k.p2*(r+2*x*x))+k.cx,
            k.fy*(y*s+k.p1*(r+2*y*y)+2*k.p2*x*y)+k.cy};
}
static std::array<double,2> map(const CameraParams& k, double u, double v) {
    return distort(k,(u-k.cx)/k.fx,(v-k.cy)/k.fy);
}
static std::array<double,2> map_error(const CameraParams& a,const CameraParams& b,int w,int h) {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    if(!(a.fx>0 && a.fy>0 && b.fx>0 && b.fy>0)) return {nan,nan};
    double sum=0, maximum=0;
    for(int y=0;y<=60;++y) for(int x=0;x<=80;++x) {
        auto p=map(a,(w-1.)*x/80,(h-1.)*y/60),q=map(b,(w-1.)*x/80,(h-1.)*y/60);
        double e=std::hypot(p[0]-q[0],p[1]-q[1]);
        if(!std::isfinite(e)) return {nan,nan};
        sum+=e*e; maximum=std::max(maximum,e);
    }
    return {std::sqrt(sum/(81*61)),maximum};
}
static Dataset synthetic(std::string name,int rows,int cols,int views,double p1,double p2,double k2,double noise,
                         double focal_scale=1,double distance_scale=1,double angle_scale=1) {
    Dataset d; d.name=name; d.rows=rows; d.cols=cols; d.synthetic=true;
    d.truth={1000,1020,628,351,-.22f,float(k2),0,float(p1),float(p2),true};
    d.truth.fx*=float(focal_scale); d.truth.fy*=float(focal_scale);
    for(int i=0;i<views;++i) {
        double a=((i%2 ? -.32:.28)+.025*i)*angle_scale, b=(-.38+.15*i)*angle_scale;
        std::vector<Point2f> points;
        for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
            double X=c-(cols-1.)*.5,Y=r-(rows-1.)*.5;
            double x=std::cos(b)*X+(i%3-1)*1.3;
            double y=std::sin(a)*std::sin(b)*X+std::cos(a)*Y+(i%2 ? .9:-.9);
            double z=-std::cos(a)*std::sin(b)*X+std::sin(a)*Y+(std::max(rows,cols)*1.65+.3*i)*distance_scale;
            auto p=distort(d.truth,x/z,y/z);
            // Deterministic noise independent of the solver; keep exact case too.
            p[0]+=noise*std::sin(13.7*(1+r*cols+c)+i*4.1);
            p[1]+=noise*std::cos(9.3*(1+r*cols+c)+i*2.3);
            points.push_back({float(p[0]),float(p[1])});
        }
        d.points.push_back(points);
    }
    return d;
}
static Dataset real_data(const fs::path& folder) {
    Dataset d; d.name="real";
    std::ifstream meta(folder/"calibration.json");
    if(!meta) throw std::runtime_error("Cannot read calibration.json");
    std::string json((std::istreambuf_iterator<char>(meta)),{});
    auto integer=[&](const char* key) { std::smatch m; std::regex re(std::string("\"")+key+"\"\\s*:\\s*([0-9]+)");
        if(!std::regex_search(json,m,re)) throw std::runtime_error("Missing metadata dimension");
        return std::stoi(m[1]); };
    d.w=integer("width"); d.h=integer("height"); d.rows=integer("rows"); d.cols=integer("cols");
    std::ifstream file(folder/"corners.csv"); if(!file) throw std::runtime_error("Cannot read corners.csv");
    std::string line; std::getline(file,line);
    while(std::getline(file,line)) {
        if(line.empty()) continue;
        std::replace(line.begin(),line.end(),',',' '); std::istringstream in(line);
        int view,index; double x,y;
        if(!(in>>view>>index>>x>>y)||view<0||view>99) throw std::runtime_error("Bad corner CSV");
        if(view==int(d.points.size())) d.points.emplace_back();
        if(view>=int(d.points.size())||index!=int(d.points[view].size())) throw std::runtime_error("Unordered corner CSV");
        d.points[view].push_back({float(x),float(y)});
    }
    return d;
}
int main(int argc,char** argv) try {
    if(argc!=4) throw std::runtime_error("Usage: recommended_regression REAL_EXPORT_DIR OUTPUT_DIR GOLDEN_CSV");
    fs::create_directories(argv[2]);
    std::vector<Dataset> sets{real_data(argv[1]),
        synthetic("radial_exact_3v",5,8,3,0,0,.06,0),
        synthetic("tangential_6v",6,9,6,.008,-.005,.06,.15),
        synthetic("radial_noisy_4v",7,7,4,0,0,.06,.15),
        synthetic("strong_k2_6v",6,9,6,0,0,.45,.05),
        synthetic("wide_6v",6,9,6,.008,-.005,.06,.15,.6,.85),
        synthetic("long_focal_4v",5,8,4,.002,-.001,.06,.10,2.2,2.2),
        synthetic("weak_pose_3v",5,8,3,.002,-.001,.06,.15,1,1.5,.15)};
    std::ifstream golden(argv[3]);
    if(!golden) throw std::runtime_error("Cannot read golden CSV");
    std::string line; std::getline(golden,line);
    std::ofstream csv(fs::path(argv[2])/"summary.csv");
    csv<<std::setprecision(12)<<"dataset,rms,map_truth_max,residual_passes,active\n";
    int failures=0;
    for(const auto& d:sets) {
        if(!std::getline(golden,line)) throw std::runtime_error("Missing golden row");
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream row(line);
        std::string name; double rms; std::array<double,8> expected{};
        if(!(row>>name>>rms)||name!=d.name) throw std::runtime_error("Golden dataset mismatch");
        for(auto& v:expected) if(!(row>>v)) throw std::runtime_error("Bad golden parameters");
        auto fit=calibrate_camera(d.points,d.w,d.h,d.rows,d.cols);
        const auto& k=fit.camera;
        std::array<double,8> actual{k.fx,k.fy,k.cx,k.cy,k.k1,k.k2,k.p1,k.p2};
        if(!k.valid || !fit.converged || std::fabs(fit.rms-rms)>.001 ||
           k.k3!=0 || fit.work.max_active!=8+6*int(d.points.size())) ++failures;
        CameraParams reference=k;
        reference.fx=expected[0]; reference.fy=expected[1];
        reference.cx=expected[2]; reference.cy=expected[3];
        reference.k1=expected[4]; reference.k2=expected[5];
        reference.p1=expected[6]; reference.p2=expected[7];
        auto baseline_delta=map_error(k,reference,d.w,d.h);
        if(!std::isfinite(baseline_delta[1]) || baseline_delta[1]>.1) ++failures;
        for(double value:actual) if(!std::isfinite(value)) ++failures;
        auto delta=map_error(k,d.synthetic?d.truth:k,d.w,d.h);
        if(d.name=="radial_exact_3v" && (fit.rms>.001 || delta[1]>.002)) ++failures;
        csv<<d.name<<','<<fit.rms<<','<<(d.synthetic?delta[1]:std::numeric_limits<double>::quiet_NaN())
           <<','<<fit.work.residual_passes<<','<<fit.work.max_active<<'\n';
        auto repeated=d.points; for(auto& view:repeated) view=d.points[0];
        if(calibrate_camera(repeated,d.w,d.h,d.rows,d.cols).camera.valid) ++failures;
        auto broken=d.points; broken[0].pop_back();
        if(calibrate_camera(broken,d.w,d.h,d.rows,d.cols).camera.valid) ++failures;
        broken=d.points; for(auto& view:broken) for(auto& point:view) point={10,10};
        if(calibrate_camera(broken,d.w,d.h,d.rows,d.cols).camera.valid) ++failures;
        std::cout<<d.name<<": valid="<<k.valid<<" rms="<<fit.rms<<" passes="<<fit.work.residual_passes<<'\n';
    }
    csv.close(); if(!csv) throw std::runtime_error("Cannot write summary.csv");
    std::cout<<"Recommended regression failures: "<<failures<<'\n';
    return failures?1:0;
} catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
