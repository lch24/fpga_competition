// Reference values call the original report.cpp and residuals.cpp without modifying them.
#include "../../../algorithom/closer2fpga/closer2fpga/algo/calibration/internal.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
using namespace calibration;
static std::ofstream vectors("validate_result_vectors.txt"),names("validate_result_cases.txt");
static int count=0,cpp_count=0;
static std::string word(double x){uint64_t u;std::memcpy(&u,&x,8);std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(16)<<u;return s.str();}
static std::string fword(float x){uint32_t u;std::memcpy(&u,&x,4);std::ostringstream s;s<<std::hex<<std::setfill('0')<<std::setw(8)<<u;return s.str();}
static std::string pack(const State& p){std::string s;for(auto i=p.rbegin();i!=p.rend();++i)s+=word(*i);return s;}
static State base(){return {std::log(800.),std::log(820.),.5,.5,.02,-.005,.001,-.002,0,
 .2,-.3,.1,.3,-.2,std::log(14.),-.3,.1,-.15,-.2,.3,std::log(16.),.1,.4,.2,.1,.2,std::log(18.)};}
static Points observe(const State& p){Points points(3,std::vector<Point2f>(40,{0,0}));std::vector<double> r;
 if(!std::isfinite(residuals(p,points,640,480,5,8,r)))return points;
 for(int v=0;v<3;v++)for(int i=0;i<40;i++)points[v][i]={float(r[2*(40*v+i)]),float(r[2*(40*v+i)+1])};return points;}
static std::string pack_points(const Points& p){std::string s;for(int v=2;v>=0;--v)for(int j=39;j>=0;--j)s+=fword(p[v][j].y)+fword(p[v][j].x);return s;}
static void add(const std::string& name,State p=base(),double square=1,bool converged=true,double override_cost=-1,int w=640,int h=480,int drop=-1,int guard_status=-1,int point_mode=0){
 Points points=observe(p);if(point_mode==1)for(int v=0;v<3;v++)for(int i=0;i<40;i++){points[v][i].x+=float((v+1)*.125);points[v][i].y-=float((i%7)*.0625);}
 if(point_mode==2)points[2][39].x=std::numeric_limits<float>::infinity();
 std::vector<double> residual;double cost=residuals(p,points,w,h,5,8,residual);if(override_cost!=-1)cost=override_cost;
 CameraCalibrationResult result;int status=guard_status,metrics=0,usable=0;State values(41,0);std::string camera(72,'0');
 if(guard_status<0){finish_result(p,points,w,h,5,8,square,cost,converged,0,result);++cpp_count;metrics=1;usable=result.camera.valid;status=usable?0:4;
  values[0]=result.rms;for(int v=0;v<3;v++)values[1+v]=result.per_view_rms[v];values[4]=result.max_error;
  for(int v=0;v<3;v++){for(int j=0;j<9;j++)values[5+12*v+j]=result.poses[v].rotation[j];for(int j=0;j<3;j++)values[5+12*v+9+j]=result.poses[v].translation[j];}
  if(usable){auto k=result.camera;camera=fword(k.p2)+fword(k.p1)+fword(k.k3)+fword(k.k2)+fword(k.k1)+fword(k.cy)+fword(k.cx)+fword(k.fy)+fword(k.fx);}
 }
 vectors<<status<<' '<<usable<<' '<<metrics<<' '<<w<<' '<<h<<' '<<converged<<' '<<drop<<' '<<word(square)<<' '<<word(cost)<<' '<<pack(p)<<' '<<pack_points(points)<<' '<<camera<<' '<<pack(values)<<'\n';
 names<<++count<<": "<<name<<" status="<<status<<" metrics="<<metrics<<" usable="<<usable<<'\n';
}
int main(){if(!vectors || !names)return 2;add("physical");add("no_convergence",base(),1,false);add("square_25",base(),25);add("square_0.025",base(),.025);add("residual_offsets",base(),1,true,-1,640,480,-1,-1,1);
 State p=base();p[8]=.003;add("camera_order_k3",p);
 for(double cost:{0.,-0.,1079.999,1080.,1080.001,1e300})add("best_cost_"+word(cost),base(),1,true,cost);
 for(int slot:{0,1})for(double focal:{31.99,32.,32.0000001,32.01,12799.,12800.,12801.}){p=base();p[slot]=std::log(focal);add("focal_"+std::to_string(slot)+"_"+std::to_string(focal),p);}
 for(int slot:{2,3})for(double norm:{-.001,-0.,0.,.999,.999999999,1.}){p=base();p[slot]=norm;add("principal_"+std::to_string(slot)+"_"+word(norm),p);}
 for(double angle:{0.,.009,.011,.1}){p=base();for(int v=0;v<3;v++)for(int j=0;j<3;j++)p[9+6*v+j]=0;p[15]=angle;add("normal_angle_"+std::to_string(angle),p);}
 for(double k1:{-1.3,-1.36,-2.}){p=base();p[4]=k1;p[5]=p[6]=p[7]=0;add("radial_mapping_"+std::to_string(k1),p);}
 // 最远网格点(0,0)的径向Jacobian行列式跨越1e-4；最终判定仍用原C++的FP32量化参数。
 for(double determinant:{0.99e-4,1.01e-4}){p=base();double r2=std::pow(320./800.,2)+std::pow(240./820.,2);
  p[4]=(-4+std::sqrt(4+12*determinant))/(6*r2);p[5]=p[6]=p[7]=0;add("mapping_determinant_"+word(determinant),p);}
 p=base();p[6]=.8;add("tangential_mapping",p);
 add("bad_width",base(),1,true,0,1,480,-1,1);add("bad_height",base(),1,true,0,640,1,-1,1);
 for(double size:{0.,-1.,std::numeric_limits<double>::infinity()})add("bad_square_"+word(size),base(),size,true,0,640,480,-1,1);
 add("bad_cost_negative",base(),1,true,-2,640,480,-1,1);add("bad_cost_inf",base(),1,true,std::numeric_limits<double>::infinity(),640,480,-1,1);
 for(int slot:{0,4,8,9,26}){p=base();p[slot]=std::numeric_limits<double>::quiet_NaN();add("nan_state_"+std::to_string(slot),p,1,true,0,640,480,-1,4);}
 p=base();p[0]=std::log(1e8);add("residual_focal_range",p,1,true,0,640,480,-1,4);
 p=base();p[4]=1e39;add("FP32_conversion_overflow",p,1,true,0,640,480,-1,4);
 add("pose_translation_overflow",base(),1e308,true,-1,640,480,-1,4);
 add("nonfinite_observation",base(),1,true,0,640,480,-1,4,2);
 add("missing_first_read",base(),1,true,-1,640,480,0,5);add("missing_last_read",base(),1,true,-1,640,480,119,5);
 names<<"TOTAL cases="<<count<<" original_cpp="<<cpp_count<<" interface_guards="<<(count-cpp_count)<<'\n';
 std::cout<<"VALIDATE_REFERENCE cases="<<count<<" original_cpp="<<cpp_count<<" interface_guards="<<(count-cpp_count)<<'\n';return 0;}
