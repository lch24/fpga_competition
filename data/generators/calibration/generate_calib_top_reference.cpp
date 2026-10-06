// Calls original calibration kernels and the public C++ entry point.
#include "../../../algorithom/closer2fpga/closer2fpga/algo/calibration/internal.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace calibration;
static std::string word(double x) {uint64_t u;std::memcpy(&u,&x,8);std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(16)<<u;return s.str();}
static std::string word32(float x) {uint32_t u;std::memcpy(&u,&x,4);std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(8)<<u;return s.str();}
static std::string pack(const State& p) {std::string s;for(auto i=p.rbegin();i!=p.rend();++i)s+=word(*i);return s;}
int main() {
    std::ofstream out("calib_top_vectors.txt"),trace("calib_top_reference.txt");
    if(!out || !trace)return 2;
    State truth={std::log(800.),std::log(820.),.5,.5,.02,-.005,.001,-.002,0,
      .2,-.3,.1,.3,-.2,std::log(14.),-.3,.1,-.15,-.2,.3,std::log(16.),.1,.4,.2,.1,.2,std::log(18.)};
    Points points(3,std::vector<Point2f>(40,{0,0}));std::vector<double> predicted;
    residuals(truth,points,640,480,5,8,predicted);
    for(int v=0;v<3;++v)for(int j=0;j<40;++j)points[v][j]={float(predicted[(v*40+j)*2]),float(predicted[(v*40+j)*2+1])};
    auto result=calibrate_camera(points,640,480,5,8,25.);
    if(!result.camera.valid || result.poses.size()!=3)return 1;
    std::vector<M3> hom(3);for(int v=0;v<3;++v)if(!homography(points[v],5,8,hom[v]))return 1;
    std::array<double,4> k;std::vector<std::pair<int,std::array<double,4>>> seeds;
    if(zhang_intrinsics(hom,640,480,k))seeds.push_back({0,k});
    int id=1;for(double f:{.6,1.,1.8,3.})seeds.push_back({id++,{640*f,640*f,319.5,239.5}});
    double best_cost=infinity;int best_id=7;trace<<std::setprecision(17);
    for(auto seed:seeds) {
        State p;if(!initialize(hom,seed.second,640,480,p))continue;
        std::vector<int> active{0,1,2,3};for(int i=9;i<27;++i)active.push_back(i);
        int accepted=0;
        for(int stage=0;stage<3;++stage) {
            if(stage==1)active.push_back(4);if(stage==2)active.insert(active.end(),{5,6,7});
            bool c=optimize(p,points,640,480,5,8,active,150,accepted);
            double cost=residuals(p,points,640,480,5,8,predicted);
            trace<<"seed="<<seed.first<<" stage="<<stage<<" cost="<<cost<<" converged="<<c<<" accepted_total="<<accepted<<'\n';
            if(stage==2 && cost<best_cost){best_cost=cost;best_id=seed.first;}
        }
    }
    std::string packed_points;
    for(int v=2;v>=0;--v)for(int j=39;j>=0;--j)packed_points+=word32(points[v][j].y)+word32(points[v][j].x);
    auto kcam=result.camera;
    std::string camera=word32(kcam.p2)+word32(kcam.p1)+word32(kcam.k3)+word32(kcam.k2)+word32(kcam.k1)+word32(kcam.cy)+word32(kcam.cx)+word32(kcam.fy)+word32(kcam.fx);
    State diag={result.rms};for(double x:result.per_view_rms)diag.push_back(x);diag.push_back(result.max_error);
    for(auto pose:result.poses){for(double x:pose.rotation)diag.push_back(x);for(double x:pose.translation)diag.push_back(x);}
    out<<best_id<<' '<<result.iterations<<' '<<result.converged<<' '<<result.weak_geometry<<' '<<camera<<' '<<pack(diag)<<' '<<packed_points<<'\n';
    trace<<"best_seed="<<best_id<<" accepted="<<result.iterations<<" rms="<<result.rms<<" valid="<<result.camera.valid<<'\n';
    std::cout<<"CPP full pipeline best_seed="<<best_id<<" accepted="<<result.iterations<<" rms="<<result.rms<<'\n';
    return 0;
}
