// ring_tables.cpp — ring_check 用 32 方向 cos/sin 表（fp32 位模式）导出
//------------------------------------------------------------------------------
// 输出 ring_cos_sin.mem：64 行，每行 8 位十六进制 fp32 位模式（$readmemh 格式）
//   行 k (0..31)     = std::cos(2*pi*k/32)   （g++ libm cosf 位级）
//   行 32+k (0..31)  = std::sin(2*pi*k/32)
// 必须与本机 g++ libm 一致——RTL ROM 直接存这些位，保证 ring 采样位级复刻。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cmath>
#include <cstdint>

int main(int argc, char** argv) {
    const char* path = argc >= 2 ? argv[1] : "ring_cos_sin.mem";
    FILE* fp = std::fopen(path, "w");
    if (!fp) return 1;
    const float pi = 3.14159265358979323846f;
    for (int k = 0; k < 32; ++k) {
        float a = 2 * pi * k / 32;
        float c = std::cos(a), s = std::sin(a);
        union { float f; uint32_t u; } x, y;
        x.f = c; y.f = s;
        std::fprintf(fp, "%08x\n", x.u);
        // 同时打印核对
        std::fprintf(stderr, "k=%2d cos=0x%08x sin=", k, x.u);
        std::fprintf(fp, "%08x\n", y.u);
        std::fprintf(stderr, "0x%08x\n", y.u);
    }
    std::fclose(fp);
    return 0;
}
