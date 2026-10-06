// Same expressions/layout as corner tests/rtl/{gen_grid_angles,ring_tables,
// export_m5}. Standard C++ only. Run from repository root.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
static uint32_t bits(float v){uint32_t b;std::memcpy(&b,&v,4);return b;}
int main(){
 const float pi=3.14159265358979323846f;
 FILE* f=std::fopen("data/rom/grid_cos_sin.mem","w");if(!f)return 1;
 for(int k=0;k<90;++k){float a=(-90+2*k)*pi/180;std::fprintf(f,"%08x\n%08x\n",bits(std::cos(a)),bits(std::sin(a)));}std::fclose(f);
 f=std::fopen("data/rom/ring_cos_sin.mem","w");if(!f)return 1;
 // The RTL indexes rom[2*k] and rom[2*k+1] (interleaved).
 for(int k=0;k<32;++k){float a=2*pi*k/32;std::fprintf(f,"%08x\n%08x\n",bits(std::cos(a)),bits(std::sin(a)));}std::fclose(f);
 f=std::fopen("data/rom/gaussian_weights.mem","w");if(!f)return 1;
 for(int r=2;r<=15;++r)for(int y=-r;y<=r;++y)for(int x=-r;x<=r;++x){
  double v=std::exp(-double(x*x+y*y)/double(r*r));uint64_t b;std::memcpy(&b,&v,8);std::fprintf(f,"%016llx\n",(unsigned long long)b);
 }std::fclose(f);return 0;
}
