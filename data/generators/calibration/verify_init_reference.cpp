// Verify committed test vectors against the project's original C++ kernels.
// Compile with initialization.cpp, pose_init.cpp, math3.cpp, symmetric_eigen.cpp.
#include "../../../algorithom/closer2fpga/closer2fpga/algo/calibration/internal.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace calibration;
static int checked=0,skipped=0,errors=0;
static std::vector<double> unpack(const std::string& s,int bits=64) {
    std::vector<double> a;
    for(size_t end=s.size();end;end-=bits/4) {
        uint64_t word=std::stoull(s.substr(end-bits/4,bits/4),nullptr,16);
        if(bits==64){double d;std::memcpy(&d,&word,8);a.push_back(d);}
        else {uint32_t w=static_cast<uint32_t>(word);float f;std::memcpy(&f,&w,4);a.push_back(f);}
    }
    return a;
}
static bool finite(const std::vector<double>& a){return std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);});}
static std::vector<M3> matrices(const std::vector<double>& a){std::vector<M3> m(3);for(int i=0;i<27;i++)m[i/9][i%9]=a[i];return m;}
static std::vector<Point2f> points(const std::vector<double>& a,int view=0){std::vector<Point2f> p;for(int i=0;i<40;i++)p.push_back({float(a[view*80+2*i]),float(a[view*80+2*i+1])});return p;}
template<class A> static void compare(const A& actual,const std::string& expected,double tol,const std::string& label){
    auto e=unpack(expected);for(size_t i=0;i<e.size();i++)if(!std::isfinite(actual[i]) || std::abs(actual[i]-e[i])>tol*(1+std::abs(e[i]))){
        ++errors;std::cerr<<label<<" item "<<i<<" actual "<<actual[i]<<" expected "<<e[i]<<"\n";
    }
}
static void check_status(bool ok,int status,const std::string& label){if(ok!=(status==0)){++errors;std::cerr<<label<<" status mismatch\n";}}
static std::ifstream open(const std::string& name){std::ifstream f(name+"_vectors.txt");if(!f)throw std::runtime_error("cannot open "+name);return f;}
int main(){try{
    std::string line,hs,ks,es,ps;int status,w,h,view,drop,row;
    auto f=open("homography");row=0;
    while(std::getline(f,line)){++row;std::istringstream in(line);in>>status>>w>>h>>view>>drop>>ps>>es;auto a=unpack(ps,32);
        // Image bounds/config/memory protocol are checked by hardware or calibrate.cpp, not homography().
        bool valid=w>=2&&h>=2&&view<3&&drop<0&&finite(a);for(size_t i=0;i<a.size();i++)valid=valid&&a[i]>=0&&a[i]<(i%2?h:w);
        if(!valid){++skipped;continue;}M3 result;bool ok=homography(points(a),5,8,result);std::string label="homography "+std::to_string(row);check_status(ok,status,label);if(ok&&status==0)compare(result,es,2e-8,label);++checked;
    }
    f=open("zhang");row=0;
    while(std::getline(f,line)){++row;std::istringstream in(line);in>>status>>w>>h>>hs>>es;auto a=unpack(hs);if(w<2||h<2||!finite(a)){++skipped;continue;}
        std::array<double,4> result;bool ok=zhang_intrinsics(matrices(a),w,h,result);std::string label="zhang "+std::to_string(row);check_status(ok,status,label);if(ok&&status==0)compare(result,es,2e-7,label);++checked;
    }
    f=open("pose_init");row=0;
    while(std::getline(f,line)){++row;std::istringstream in(line);in>>status>>w>>h>>hs>>ks>>es;auto a=unpack(hs),k=unpack(ks);if(w<2||h<2||!finite(a)||!finite(k)||k[0]<=0||k[1]<=0){++skipped;continue;}
        State result;bool ok=initialize(matrices(a),{k[0],k[1],k[2],k[3]},w,h,result);std::string label="pose_init "+std::to_string(row);check_status(ok,status,label);if(ok&&status==0)compare(result,es,2e-9,label);++checked;
    }
    f=open("init_controller");row=0;
    while(std::getline(f,line)){++row;int mode,count,mask;std::string expected[5];std::istringstream in(line);in>>status>>w>>h>>mode>>count>>ps;for(auto& x:expected)in>>x;in>>mask;
        if(w<2||h<2){++skipped;continue;}auto a=unpack(ps,32);std::vector<M3> hom(3);bool ok=true;
        for(int v=0;v<3;v++)if(!homography(points(a,v),5,8,hom[v])){ok=false;break;}
        double diversity=0;if(ok)for(int v=1;v<3;v++)for(int j=0;j<8;j++)diversity+=std::abs(hom[v][j]-hom[0][j]);if(diversity<1e-6)ok=false;
        int got=0,n=0;std::string label="init_controller "+std::to_string(row);
        if(ok){std::array<double,4> zk;bool zok=zhang_intrinsics(hom,w,h,zk);double factors[]={.6,1.,1.8,3.};
            for(int id=0;id<5;id++){if(id==0&&(!zok||mode==1))continue;auto k=id?std::array<double,4>{w*factors[id-1],w*factors[id-1],(w-1)*.5,(h-1)*.5}:zk;State p;
                if(!initialize(hom,k,w,h,p)||mode==3||(mode==2&&id==2))continue;got|=1<<id;++n;compare(p,expected[id],3e-7,label+" seed "+std::to_string(id));
            }
        }
        check_status(n>0,status,label);if(n!=count||got!=mask){++errors;std::cerr<<label<<" seed mask/count mismatch\n";}++checked;
    }
    std::cout<<"CPP_REFERENCE checked="<<checked<<" protocol_or_invalid_input_skipped="<<skipped<<" errors="<<errors<<"\n";
    return errors?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
