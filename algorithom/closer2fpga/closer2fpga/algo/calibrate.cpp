#include "calibrate.h"
#include "../common/matrix.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace {
using M3 = std::array<double, 9>;
using V3 = std::array<double, 3>;
using State = std::vector<double>;
constexpr double infinity = std::numeric_limits<double>::infinity();
double dot(V3 a, V3 b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
V3 cross(V3 a, V3 b) { return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]}; }
V3 scaled(V3 a, double s) { for (auto& v : a) v *= s; return a; }
M3 multiply(const M3& a, const M3& b) {
    M3 c{};
    for (int r=0;r<3;++r) for (int col=0;col<3;++col)
        for (int k=0;k<3;++k) c[r*3+col] += a[r*3+k]*b[k*3+col];
    return c;
}

// Jacobi eigensolver for small real symmetric matrices (normalized DLT/Zhang).
// Eigenvectors are columns; sorting leaves the smallest eigenpair first.
bool eigen_symmetric(std::vector<double> a, int n, std::vector<double>& values, std::vector<double>& vectors) {
    vectors.assign(n*n, 0);
    for (int i=0;i<n;++i) vectors[i*n+i] = 1;
    bool done = false;
    for (int iteration=0;iteration<100*n*n;++iteration) {
        int p=0,q=1; double largest=0, diagonal=0;
        for (int i=0;i<n;++i) {
            diagonal = std::max(diagonal, std::fabs(a[i*n+i]));
            for (int j=i+1;j<n;++j) if (std::fabs(a[i*n+j]) > largest) {
                largest=std::fabs(a[i*n+j]); p=i; q=j;
            }
        }
        if (largest <= 1e-14*std::max(diagonal,1e-30)) { done=true; break; }
        double phi=0.5*std::atan2(2*a[p*n+q], a[q*n+q]-a[p*n+p]);
        double c=std::cos(phi), s=std::sin(phi);
        double app=a[p*n+p], aqq=a[q*n+q], apq=a[p*n+q];
        for (int k=0;k<n;++k) if (k!=p && k!=q) {
            double akp=a[k*n+p], akq=a[k*n+q];
            a[k*n+p]=a[p*n+k]=c*akp-s*akq;
            a[k*n+q]=a[q*n+k]=s*akp+c*akq;
        }
        a[p*n+p]=c*c*app-2*s*c*apq+s*s*aqq;
        a[q*n+q]=s*s*app+2*s*c*apq+c*c*aqq;
        a[p*n+q]=a[q*n+p]=0;
        for (int k=0;k<n;++k) {
            double vkp=vectors[k*n+p], vkq=vectors[k*n+q];
            vectors[k*n+p]=c*vkp-s*vkq; vectors[k*n+q]=s*vkp+c*vkq;
        }
    }
    if (!done) return false;
    std::vector<int> order(n); std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int i,int j){return a[i*n+i]<a[j*n+j];});
    auto original=vectors; values.resize(n);
    for (int j=0;j<n;++j) {
        values[j]=a[order[j]*n+order[j]];
        for (int i=0;i<n;++i) vectors[i*n+j]=original[i*n+order[j]];
    }
    return true;
}

void accumulate_outer(std::vector<double>& a, const std::vector<double>& row) {
    const int n=int(row.size());
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) a[i*n+j]+=row[i]*row[j];
}

bool homography(const std::vector<Point2f>& points,int rows,int cols,M3& h) {
    // Object coordinates are centered and expressed in board-square units.
    double mx=0,my=0;
    for (auto p:points) { mx+=p.x; my+=p.y; }
    mx/=points.size(); my/=points.size();
    double di=0, dw=0;
    for (int r=0;r<rows;++r) for (int c=0;c<cols;++c) {
        auto p=points[r*cols+c];
        di+=std::hypot(p.x-mx,p.y-my);
        dw+=std::hypot(c-(cols-1)*.5,r-(rows-1)*.5);
    }
    if (di<1e-6 || dw<1e-6) return false;
    double si=std::sqrt(2.)*points.size()/di, sw=std::sqrt(2.)*points.size()/dw;
    std::vector<double> ata(81,0),eval,evec;
    for (int r=0;r<rows;++r) for (int c=0;c<cols;++c) {
        auto p=points[r*cols+c];
        double x=(c-(cols-1)*.5)*sw, y=(r-(rows-1)*.5)*sw;
        double u=(p.x-mx)*si, v=(p.y-my)*si;
        accumulate_outer(ata,{-x,-y,-1,0,0,0,u*x,u*y,u});
        accumulate_outer(ata,{0,0,0,-x,-y,-1,v*x,v*y,v});
    }
    if (!eigen_symmetric(ata,9,eval,evec) || eval[1]<eval[8]*1e-10) return false;
    M3 normalized{};
    for (int k=0;k<9;++k) normalized[k]=evec[k*9];
    h=multiply(multiply({1/si,0,mx,0,1/si,my,0,0,1},normalized),{sw,0,0,0,sw,0,0,0,1});
    if (std::fabs(h[8])<1e-12) return false;
    double scale=h[8]; for (auto& v:h) v/=scale;
    return true;
}

std::vector<double> vij(const M3& h,int i,int j) {
    return {h[i]*h[j],h[i]*h[3+j]+h[3+i]*h[j],h[3+i]*h[3+j],
        h[6+i]*h[j]+h[i]*h[6+j],h[6+i]*h[3+j]+h[3+i]*h[6+j],h[6+i]*h[6+j]};
}

bool zhang_intrinsics(const std::vector<M3>& homographies,int w,int h,std::array<double,4>& k) {
    // Normalize image coordinates before forming the conic constraints to avoid
    // mixing pixel^4 terms with order-one entries in the eigensystem.
    std::vector<double> ata(36,0),eval,evec;
    M3 t{1./w,0,-.5,0,1./w,-double(h)/(2*w),0,0,1};
    for (const auto& hom:homographies) {
        M3 a=multiply(t,hom);
        auto v12=vij(a,0,1), v11=vij(a,0,0), v22=vij(a,1,1);
        for (int i=0;i<6;++i) v11[i]-=v22[i];
        accumulate_outer(ata,v12); accumulate_outer(ata,v11);
    }
    if (!eigen_symmetric(ata,6,eval,evec)) return false;
    double b[6]; for (int i=0;i<6;++i) b[i]=evec[i*6];
    if (b[0]<0) for (auto& v:b) v=-v;
    double denominator=b[0]*b[2]-b[1]*b[1];
    if (b[0]<=0 || denominator<=1e-14) return false;
    double cy=(b[1]*b[3]-b[0]*b[4])/denominator;
    double lambda=b[5]-(b[3]*b[3]+cy*(b[1]*b[3]-b[0]*b[4]))/b[0];
    if (lambda<=0) return false;
    double fx=std::sqrt(lambda/b[0]), fy=std::sqrt(lambda*b[0]/denominator);
    double skew=-b[1]*fx*fx*fy/lambda;
    double cx=skew*cy/fy-b[3]*fx*fx/lambda;
    k={fx*w,fy*w,cx*w+w*.5,cy*w+h*.5};
    return std::isfinite(k[0]) && std::isfinite(k[1]) && k[0]>.05*w && k[1]>.05*w
        && k[0]<20*w && k[1]<20*w && std::fabs(k[2]-w*.5)<w && std::fabs(k[3]-h*.5)<h;
}

M3 rodrigues(V3 v) {
    double t2=dot(v,v), a,b;
    if (t2<1e-12) { a=1-t2/6; b=.5-t2/24; }
    else { double t=std::sqrt(t2); a=std::sin(t)/t; b=(1-std::cos(t))/t2; }
    M3 s{0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0}, ss=multiply(s,s), r{};
    for (int i=0;i<9;++i) r[i]=(i%4==0 ? 1.:0.)+a*s[i]+b*ss[i];
    return r;
}

V3 rotation_vector(const M3& r) {
    // Quaternion conversion remains stable for row/column-flipped boards whose
    // rotations can be close to pi (unlike division by sin(theta)).
    double qw,qx,qy,qz, trace=r[0]+r[4]+r[8];
    if (trace>0) {
        double s=2*std::sqrt(trace+1); qw=s/4; qx=(r[7]-r[5])/s; qy=(r[2]-r[6])/s; qz=(r[3]-r[1])/s;
    } else if (r[0]>r[4] && r[0]>r[8]) {
        double s=2*std::sqrt(1+r[0]-r[4]-r[8]); qw=(r[7]-r[5])/s; qx=s/4; qy=(r[1]+r[3])/s; qz=(r[2]+r[6])/s;
    } else if (r[4]>r[8]) {
        double s=2*std::sqrt(1+r[4]-r[0]-r[8]); qw=(r[2]-r[6])/s; qx=(r[1]+r[3])/s; qy=s/4; qz=(r[5]+r[7])/s;
    } else {
        double s=2*std::sqrt(1+r[8]-r[0]-r[4]); qw=(r[3]-r[1])/s; qx=(r[2]+r[6])/s; qy=(r[5]+r[7])/s; qz=s/4;
    }
    if (qw<0) { qw=-qw; qx=-qx; qy=-qy; qz=-qz; }
    double n=std::sqrt(qx*qx+qy*qy+qz*qz);
    double scale=n>1e-12 ? 2*std::atan2(n,qw)/n : 2;
    return {qx*scale,qy*scale,qz*scale};
}

bool initialize(const std::vector<M3>& hom,const std::array<double,4>& k,int w,int h,State& p) {
    p.assign(9+6*hom.size(),0);
    p[0]=std::log(k[0]); p[1]=std::log(k[1]); p[2]=k[2]/w; p[3]=k[3]/h;
    for (size_t i=0;i<hom.size();++i) {
        const auto& a=hom[i];
        auto kinv=[&](int c)->V3 {return {(a[c]-k[2]*a[6+c])/k[0],(a[3+c]-k[3]*a[6+c])/k[1],a[6+c]};};
        V3 v1=kinv(0),v2=kinv(1),t=kinv(2);
        double n1=std::sqrt(dot(v1,v1)),n2=std::sqrt(dot(v2,v2));
        if (n1<1e-12 || n2<1e-12) return false;
        double scale=2/(n1+n2); if (t[2]<0) scale=-scale;
        t=scaled(t,scale); V3 r1=scaled(v1,scale>0 ? 1/n1:-1/n1);
        double parallel=dot(r1,v2); for (int j=0;j<3;++j) v2[j]-=parallel*r1[j];
        double norm=std::sqrt(dot(v2,v2)); if (norm<1e-12 || t[2]<=0) return false;
        V3 r2=scaled(v2,scale>0 ? 1/norm:-1/norm),r3=cross(r1,r2);
        M3 rotation{r1[0],r2[0],r3[0],r1[1],r2[1],r3[1],r1[2],r2[2],r3[2]};
        V3 rv=rotation_vector(rotation);
        size_t offset=9+6*i;
        for (int j=0;j<3;++j) p[offset+j]=rv[j];
        p[offset+3]=t[0]; p[offset+4]=t[1]; p[offset+5]=std::log(t[2]);
    }
    return true;
}

double residuals(const State& p,const std::vector<std::vector<Point2f>>& points,
    int w,int h,int rows,int cols,std::vector<double>& residual) {
    for (double v:p) if (!std::isfinite(v)) return infinity;
    double fx=std::exp(p[0]),fy=std::exp(p[1]),cx=p[2]*w,cy=p[3]*h;
    if (fx<1e-3 || fy<1e-3 || fx>1e7 || fy>1e7) return infinity;
    residual.resize(points.size()*rows*cols*2);
    double cost=0;
    for (size_t i=0;i<points.size();++i) {
        size_t offset=9+6*i;
        M3 r=rodrigues({p[offset],p[offset+1],p[offset+2]});
        double tz=std::exp(p[offset+5]);
        for (int y=0;y<rows;++y) for (int x=0;x<cols;++x) {
            double X=x-(cols-1)*.5,Y=y-(rows-1)*.5;
            double z=r[6]*X+r[7]*Y+tz;
            if (z<=1e-5) return infinity;
            double nx=(r[0]*X+r[1]*Y+p[offset+3])/z;
            double ny=(r[3]*X+r[4]*Y+p[offset+4])/z;
            double r2=nx*nx+ny*ny, radial=1+p[4]*r2+p[5]*r2*r2+p[8]*r2*r2*r2;
            double xd=nx*radial+2*p[6]*nx*ny+p[7]*(r2+2*nx*nx);
            double yd=ny*radial+p[6]*(r2+2*ny*ny)+2*p[7]*nx*ny;
            size_t id=2*(i*rows*cols+y*cols+x);
            double dx=fx*xd+cx-points[i][y*cols+x].x;
            double dy=fy*yd+cy-points[i][y*cols+x].y;
            residual[id]=dx; residual[id+1]=dy; cost+=dx*dx+dy*dy;
        }
    }
    return std::isfinite(cost) ? cost:infinity;
}

bool optimize(State& p,const std::vector<std::vector<Point2f>>& points,int w,int h,int rows,int cols,
    const std::vector<int>& active,int limit,int& iterations) {
    std::vector<double> r;
    double cost=residuals(p,points,w,h,rows,cols,r),lambda=1e-3;
    int n=int(active.size()),m=int(r.size());
    if (!std::isfinite(cost)) return false;
    for (int it=0;it<limit;++it) {
        if (cost<1e-16) return true;
        std::vector<double> j(m*n),scales(n),plus,minus;
        for (int k=0;k<n;++k) {
            State q=p; double step=1e-6*(1+std::fabs(p[active[k]]));
            q[active[k]]+=step;
            if (!std::isfinite(residuals(q,points,w,h,rows,cols,plus))) return false;
            q[active[k]]-=2*step;
            if (!std::isfinite(residuals(q,points,w,h,rows,cols,minus))) return false;
            double norm=0;
            for (int t=0;t<m;++t) { double v=(plus[t]-minus[t])/(2*step); j[t*n+k]=v; norm+=v*v; }
            scales[k]=1/std::max(std::sqrt(norm),1e-12);
            for (int t=0;t<m;++t) j[t*n+k]*=scales[k];
        }
        std::vector<double> normal(n*n,0),gradient(n,0);
        for (int t=0;t<m;++t) for (int a=0;a<n;++a) {
            gradient[a]+=j[t*n+a]*r[t];
            for (int b=0;b<=a;++b) normal[a*n+b]+=j[t*n+a]*j[t*n+b];
        }
        double max_gradient=0;
        for (double g:gradient) max_gradient=std::max(max_gradient,std::fabs(g));
        if (max_gradient<1e-8*(1+std::sqrt(cost))) return true;
        bool accepted=false;
        for (int attempt=0;attempt<16;++attempt) {
            LMtx a(n,n),b(n,1);
            for (int x=0;x<n;++x) {
                b.at(x,0)=-gradient[x];
                for (int y=0;y<n;++y) a.at(x,y)=x>=y ? normal[x*n+y]:normal[y*n+x];
                a.at(x,x)+=lambda;
            }
            auto delta=solve_gauss(std::move(a),std::move(b));
            if (delta.rows!=n) {lambda*=10; continue;}
            State q=p; double step_norm=0;
            for (int k=0;k<n;++k) {
                double d=delta.at(k,0)*scales[k]; q[active[k]]+=d;
                step_norm=std::max(step_norm,std::fabs(d)/(1+std::fabs(p[active[k]])));
            }
            std::vector<double> trial;
            double new_cost=residuals(q,points,w,h,rows,cols,trial);
            if (new_cost<cost) {
                double reduction=cost-new_cost;
                p=std::move(q); r=std::move(trial); cost=new_cost; ++iterations;
                lambda=std::max(lambda*.3,1e-12); accepted=true;
                if (step_norm<1e-9 || reduction<1e-11*(1+cost)) return true;
                break;
            }
            lambda*=10;
        }
        if (!accepted) return max_gradient<1e-5*(1+std::sqrt(cost));
    }
    return false;
}

bool mapping_is_regular(const CameraParams& k,int w,int h) {
    // Require positive Jacobian determinant throughout the output field to
    // reject folding maps caused by extrapolated high-order radial coefficients.
    for (int iy=0;iy<=24;++iy) for (int ix=0;ix<=32;++ix) {
        double x=((w-1)*ix/32.-k.cx)/k.fx,y=((h-1)*iy/24.-k.cy)/k.fy;
        double r2=x*x+y*y,radial=1+k.k1*r2+k.k2*r2*r2+k.k3*r2*r2*r2;
        double dr=k.k1+2*k.k2*r2+3*k.k3*r2*r2;
        double a=radial+2*x*x*dr+2*k.p1*y+6*k.p2*x;
        double b=2*x*y*dr+2*k.p1*x+2*k.p2*y;
        double d=radial+2*y*y*dr+6*k.p1*y+2*k.p2*x;
        if (!std::isfinite(a*d-b*b) || a<=0 || d<=0 || a*d-b*b<=1e-4) return false;
    }
    return true;
}
}

CameraCalibrationResult calibrate_camera(const std::vector<std::vector<Point2f>>& points,
    int width,int height,int rows,int cols,double square_size,const CameraCalibrationOptions& options) {
    CameraCalibrationResult result;
    result.k3_estimated=options.estimate_k3;
    if (points.size()<3 || points.size()>100 || width<2 || height<2 || rows<3 || cols<3 || rows>100 || cols>100 ||
        !std::isfinite(square_size) || square_size<=0 || options.max_iterations<1) {
        result.message="Need at least three views, valid image/board dimensions and positive square size."; return result;
    }
    std::vector<M3> hom(points.size());
    for (size_t i=0;i<points.size();++i) {
        if (points[i].size()!=size_t(rows*cols)) {result.message="Corner count mismatch."; return result;}
        for (auto p:points[i]) if (!std::isfinite(p.x)||!std::isfinite(p.y)||p.x<0||p.y<0||p.x>=width||p.y>=height) {
            result.message="Non-finite or out-of-image corner."; return result;
        }
        if (!homography(points[i],rows,cols,hom[i])) {result.message="Degenerate board geometry."; return result;}
    }
    // Reject repeated homographies; three copies of one observation are not
    // three independent calibration views, even though LM could fit them.
    double diversity=0;
    for (size_t i=1;i<hom.size();++i) for (int j=0;j<8;++j)
        diversity+=std::fabs(hom[i][j]-hom[0][j]);
    if (diversity<1e-6) {result.message="Repeated views do not constrain camera intrinsics."; return result;}
    std::vector<std::array<double,4>> seeds;
    std::array<double,4> zhang;
    if (zhang_intrinsics(hom,width,height,zhang)) seeds.push_back(zhang);
    for (double factor:{.6,1.,1.8,3.}) seeds.push_back({width*factor,width*factor,(width-1)*.5,(height-1)*.5});
    State best; double best_cost=infinity; bool best_converged=false; int best_iterations=0;
    for (const auto& seed:seeds) {
        State state;
        if (!initialize(hom,seed,width,height,state)) continue;
        std::vector<int> active{0,1,2,3};
        for (int i=9;i<int(state.size());++i) active.push_back(i);
        int iterations=0;
        optimize(state,points,width,height,rows,cols,active,options.max_iterations,iterations);
        active.push_back(4);
        optimize(state,points,width,height,rows,cols,active,options.max_iterations,iterations);
        active.insert(active.end(),{5,6,7});
        bool converged=optimize(state,points,width,height,rows,cols,active,options.max_iterations,iterations);
        if (options.estimate_k3) {
            active.push_back(8);
            converged=optimize(state,points,width,height,rows,cols,active,options.max_iterations,iterations);
        }
        std::vector<double> residual;
        double cost=residuals(state,points,width,height,rows,cols,residual);
        if (cost<best_cost) {best_cost=cost; best=std::move(state); best_converged=converged; best_iterations=iterations;}
    }
    if (best.empty()) {result.message="Calibration optimization failed."; return result;}
    auto& k=result.camera;
    k.fx=float(std::exp(best[0])); k.fy=float(std::exp(best[1])); k.cx=float(best[2]*width); k.cy=float(best[3]*height);
    k.k1=float(best[4]); k.k2=float(best[5]); k.p1=float(best[6]); k.p2=float(best[7]); k.k3=float(best[8]);
    result.converged=best_converged; result.iterations=best_iterations;
    std::vector<double> residual;
    residuals(best,points,width,height,rows,cols,residual);
    result.rms=std::sqrt(best_cost/(points.size()*rows*cols));
    for (size_t i=0;i<points.size();++i) {
        double cost=0;
        for (int j=0;j<rows*cols;++j) {
            size_t id=2*(i*rows*cols+j); double error=std::hypot(residual[id],residual[id+1]);
            cost+=error*error; result.max_error=std::max(result.max_error,error);
        }
        result.per_view_rms.push_back(std::sqrt(cost/(rows*cols)));
        size_t offset=9+6*i;
        CameraPose pose;
        pose.rotation=rodrigues({best[offset],best[offset+1],best[offset+2]});
        // Report translation relative to the first inner corner (0,0), not
        // the centered coordinates used internally for conditioning.
        for (int j=0;j<3;++j) {
            double t=j==2 ? std::exp(best[offset+5]):best[offset+3+j];
            pose.translation[j]=(t-pose.rotation[j*3]*(cols-1)*.5-pose.rotation[j*3+1]*(rows-1)*.5)*square_size;
        }
        result.poses.push_back(pose);
    }
    double max_angle=0;
    for (size_t i=0;i<result.poses.size();++i) for (size_t j=0;j<i;++j) {
        auto a=result.poses[i].rotation,b=result.poses[j].rotation;
        double cosine=std::fabs(a[2]*b[2]+a[5]*b[5]+a[8]*b[8]);
        max_angle=std::max(max_angle,std::acos(std::clamp(cosine,0.,1.)));
    }
    result.weak_geometry=max_angle<.17 || points.size()<5;
    bool intrinsics_ok=k.fx>.05*width && k.fy>.05*width && k.fx<20*width && k.fy<20*width
        && k.cx>=0 && k.cx<width && k.cy>=0 && k.cy<height;
    k.valid=best_converged && intrinsics_ok && result.rms<3 && mapping_is_regular(k,width,height) && max_angle>.01;
    if (!k.valid) result.message="Unreliable calibration: check convergence, intrinsics, reprojection error and map folding. No correction should be applied.";
    else if (result.weak_geometry) result.message="Only a few views or small pose variation: provisional parameters; low training RMS does not prove accuracy outside the board.";
    else result.message="Calibration converged.";
    return result;
}
