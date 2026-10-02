// gen_grid_angles.cpp — organize_grid 用 90 方向 cos/sin 表导出
//------------------------------------------------------------------------------
// 输出 grid_cos_sin.mem：180 行，每行 8 位十六进制 fp32 位模式（$readmemh）
//   行 2k (k=0..89)     = std::cos(degree_k * pi / 180)   （degree = -90 + 2k）
//   行 2k+1 (k=0..89)   = std::sin(degree_k * pi / 180)
// 与 export_m4.cpp organize_grid_det 的 `std::cos(a)/std::sin(a)`（a 为 float）
// 一致——用本机 g++ libm cosf/sinf 位模式，RTL ROM 直接存这些位。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cmath>
#include <cstdint>

int main(int argc, char** argv) {
    const char* path = argc >= 2 ? argv[1] : "grid_cos_sin.mem";
    FILE* fp = std::fopen(path, "w");
    if (!fp) return 1;
    const float pi = 3.14159265358979323846f;
    for (int k = 0; k < 90; ++k) {
        int degree = -90 + 2 * k;
        float a = degree * pi / 180;
        float c = std::cos(a), s = std::sin(a);
        union { float f; uint32_t u; } x, y;
        x.f = c; y.f = s;
        std::fprintf(fp, "%08x\n", x.u);
        std::fprintf(fp, "%08x\n", y.u);
        std::fprintf(stderr, "deg=%3d cos=0x%08x sin=0x%08x\n", degree, x.u, y.u);
    }
    std::fclose(fp);
    return 0;
}
