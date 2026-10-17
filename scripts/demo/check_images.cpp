// Pixel-domain test: artificial coefficients never enter detect_chessboard or calibrate_camera.
#include "../../algorithom/closer2fpga/closer2fpga/algo/chessboard.h"
#include "../../algorithom/closer2fpga/closer2fpga/algo/calibrate.h"
#include "../../algorithom/closer2fpga/closer2fpga/algo/grayscale.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
namespace fs=std::filesystem;
constexpr int W=1280,H=720;
static cv::Point2d project(const CameraParams& p,double u,double v){
 double x=(u-p.cx)/p.fx,y=(v-p.cy)/p.fy,r=x*x+y*y,s=1+p.k1*r+p.k2*r*r+p.k3*r*r*r;
 return {p.fx*(x*s+2*p.p1*x*y+p.p2*(r+2*x*x))+p.cx,p.fy*(y*s+p.p1*(r+2*y*y)+2*p.p2*x*y)+p.cy};
}
static cv::Point2d inverse_artificial(double u,double v){
 double x=(u-639.5)/1024,y=(v-359.5)/1024,r=std::hypot(x,y),z=r;
 for(int i=0;i<30;i++)z-=(z*(1-.18*z*z)-r)/(1-.54*z*z);
 double q=r>0?z/r:1;return {639.5+1024*x*q,359.5+1024*y*q};
}
int main(int argc,char**argv){
 if(argc<3)return 2;bool synthetic=argc>3;fs::path input=argv[1],out=argv[2];fs::create_directories(out);
 cv::Mat mx(H,W,CV_32F),my(H,W,CV_32F);double max_error=0;
 for(int y=0;y<H;y++)for(int x=0;x<W;x++){
  float xd=(x-639.5f)/1024,yd=(y-359.5f)/1024,nx=xd,ny=yd;
  for(int k=0;k<16;k++){float r=nx*nx+ny*ny;float d=1+(-.18f*r);nx=xd/d;ny=yd/d;}
  float sx=1024*nx+639.5f,sy=1024*ny+359.5f;mx.at<float>(y,x)=sx;my.at<float>(y,x)=sy;
  auto ref=inverse_artificial(x,y);max_error=std::max(max_error,std::hypot(sx-ref.x,sy-ref.y));
 }
 std::cout<<"full-map max inverse error="<<max_error<<std::endl;if(max_error>.05)return 3;
 std::vector<std::vector<Point2f>> points;std::vector<cv::Mat> warped;
 std::ofstream corners(out/"corners.csv");corners<<"view,index,x,y\n";
 for(int v=0;v<3;v++){
  cv::Mat image=cv::imread((input/("test"+std::to_string(v)+".jpg")).string());if(image.empty())return 4;
  cv::resize(image,image,{W,H});
  if(synthetic){
   cv::Mat board(480,720,CV_8UC3);
   for(int row=0;row<6;row++)for(int col=0;col<9;col++)
    cv::rectangle(board,{col*80,row*80,80,80},cv::Scalar::all((row+col)%2?230:20),cv::FILLED);
   double ax[3]={.25,-.3,.38},ay[3]={-.3,.35,.12};
   double ca=cos(ax[v]),sa=sin(ax[v]),cb=cos(ay[v]),sb=sin(ay[v]);
   cv::Point2f q[4],b[4]={{0,0},{719,0},{719,479},{0,479}};
   for(int j=0;j<4;j++){
    double X=(j==1||j==2?4.5:-4.5),Y=(j>=2?3.:-3.);
    double xx=cb*X+(v-1)*.8,yy=sa*sb*X+ca*Y+(v==1?.7:-.4),zz=-ca*sb*X+sa*Y+13+v*.4;
    q[j]={float(639.5+1024*xx/zz),float(359.5+1024*yy/zz)};
   }
   cv::warpPerspective(board,image,cv::getPerspectiveTransform(b,q),{W,H},cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar::all(127));
   cv::imwrite((out/("ideal"+std::to_string(v)+".png")).string(),image);
  }
cv::Mat distorted;cv::remap(image,distorted,mx,my,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
  cv::imwrite((out/("distorted"+std::to_string(v)+".png")).string(),distorted);warped.push_back(distorted);
  GrayImage rgb(W,H,3),gray(W,H);std::copy(distorted.data,distorted.data+W*H*3,rgb.data);
  convert_grayscale(image_view(static_cast<const GrayImage&>(rgb)),image_view(gray));
  auto board=detect_chessboard(gray,5,8);std::cout<<"view="<<v<<" detected="<<board.valid<<" corners="<<board.corners.size()<<std::endl;
  if(!board.valid||board.corners.size()!=40)return 5;points.push_back(board.corners);
  for(int j=0;j<40;j++)corners<<v<<','<<j<<','<<board.corners[j].x<<','<<board.corners[j].y<<'\n';
 }
 auto fit=calibrate_camera(points,W,H,5,8,1.0);const auto&p=fit.camera;
 std::cout<<"independent calibration valid="<<p.valid<<" rms="<<fit.rms<<" k1="<<p.k1<<" k2="<<p.k2<<" k3="<<p.k3<<std::endl;
 if(!p.valid||!std::isfinite(fit.rms)||fit.rms>1)return 6;
 for(int y=0;y<H;y++)for(int x=0;x<W;x++){auto q=project(p,x,y);mx.at<float>(y,x)=float(q.x);my.at<float>(y,x)=float(q.y);}
 for(int v=0;v<3;v++){cv::Mat corrected;cv::remap(warped[v],corrected,mx,my,cv::INTER_LINEAR,cv::BORDER_CONSTANT);cv::imwrite((out/("corrected"+std::to_string(v)+".png")).string(),corrected);}
 // Geometric straightness: trace lines in the corrected output back to the
 // original ideal input through the fitted map and the injected inverse.
 double before=0,after=0;int count=0;
 for(int axis=0;axis<2;axis++)for(int line=1;line<=5;line++){
  std::vector<cv::Point2f> raw,corrected;
  for(int i=0;i<=60;i++){
   double u=axis?160+i*16:160+line*160,v=axis?80+line*90:80+i*9;
   auto b=inverse_artificial(u,v),q=project(p,u,v),a=inverse_artificial(q.x,q.y);
   if(b.x<0||b.x>=W||b.y<0||b.y>=H||a.x<0||a.x>=W||a.y<0||a.y>=H)continue;
   raw.emplace_back(float(b.x),float(b.y));corrected.emplace_back(float(a.x),float(a.y));
  }
  if(raw.size()<10)continue;cv::Vec4f l0,l1;cv::fitLine(raw,l0,cv::DIST_L2,0,.01,.01);cv::fitLine(corrected,l1,cv::DIST_L2,0,.01,.01);
  for(size_t i=0;i<raw.size();i++){double b=(raw[i].x-l0[2])*l0[1]-(raw[i].y-l0[3])*l0[0],a=(corrected[i].x-l1[2])*l1[1]-(corrected[i].y-l1[3])*l1[0];before+=b*b;after+=a*a;count++;}
 }
 double ratio=std::sqrt(after/before);
 std::ofstream report(out/"result.json");report<<"{\"rms\":"<<fit.rms<<",\"map_max_error\":"<<max_error<<",\"line_error_before\":"<<std::sqrt(before/count)<<",\"line_error_after\":"<<std::sqrt(after/count)<<",\"line_error_ratio\":"<<ratio<<"}\n";
 std::cout<<(synthetic?"synthetic geometry ":"real image diagnostic (native lens distortion included) ")<<"straightness ratio="<<ratio<<" (target <=0.2)"<<std::endl;
 if(synthetic&&ratio>.2)return 7;
 std::cout<<"PASS real pixel distortion -> production C++ detection -> independent calibration -> correction"<<std::endl;
}

