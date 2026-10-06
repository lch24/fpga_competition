// Expected results call the original C++ optimize/residuals/solve_gauss, not an RTL translation.
#include "../../../algorithom/closer2fpga/closer2fpga/algo/calibration/internal.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace calibration;
static std::string word(double x){uint64_t u;std::memcpy(&u,&x,8);std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(16)<<u;return s.str();}
static std::string pack(const State& p){std::string s;for(auto i=p.rbegin();i!=p.rend();++i)s+=word(*i);return s;}
static std::string pack_points(const Points& p){std::string s;for(int v=2;v>=0;--v)for(int j=39;j>=0;--j)for(int c=1;c>=0;--c){float x=c?p[v][j].y:p[v][j].x;uint32_t u;std::memcpy(&u,&x,4);std::ostringstream o;o<<std::hex<<std::setfill('0')<<std::setw(8)<<u;s+=o.str();}return s;}
int main(){
 std::ofstream out("lm_controller_vectors.txt"),info("lm_controller_cases.txt");
 if(!out||!info)return 2;
 for(int stage=0;stage<3;++stage){
  State truth={std::log(800.),std::log(820.),.5,.5,stage>0?.02:0,stage>1?-.005:0,stage>1?.001:0,stage>1?-.002:0,0,
    .2,-.3,.1,.3,-.2,std::log(14.),-.3,.1,-.15,-.2,.3,std::log(16.),.1,.4,.2,.1,.2,std::log(18.)};
  Points points(3,std::vector<Point2f>(40,{0,0}));std::vector<double> predicted;
  residuals(truth,points,640,480,5,8,predicted);
  for(int v=0;v<3;++v)for(int j=0;j<40;++j)points[v][j]={float(predicted[(v*40+j)*2]),float(predicted[(v*40+j)*2+1])};
  std::vector<int> active={0,1,2,3};for(int k=9;k<27;++k)active.push_back(k);if(stage>0)active.push_back(4);if(stage>1)for(int k=5;k<8;++k)active.push_back(k);
  State initial=truth;for(size_t k=0;k<active.size();++k)initial[active[k]]+=(int(k%5)-2)*1e-5;
  State result=initial;int accepted=0;bool converged=optimize(result,points,640,480,5,8,active,150,accepted);
  std::vector<double> r;double cost=residuals(result,points,640,480,5,8,r);
  out<<stage<<' '<<converged<<' '<<accepted<<' '<<word(cost)<<' '<<pack(initial)<<' '<<pack_points(points)<<' '<<pack(result)<<'\n';
  info<<"stage"<<stage<<" physical Brown model, accepted="<<accepted<<" converged="<<converged<<" cost="<<std::setprecision(17)<<cost<<'\n';
  std::cout<<"CPP stage="<<stage<<" accepted="<<accepted<<" converged="<<converged<<" cost="<<cost<<'\n';
  if(!converged || accepted==0)return 1;
 }
 return 0;
}
