// gen_fp64_min.cpp — 精简定向向量（覆盖全部代码分支，ModelSim 组合仿真性能受限时使用）
//------------------------------------------------------------------------------
// 输出 test_fp64_min.bin，格式与 gen_fp64_vectors.cpp 一致（kind, a_lo,a_hi, b_lo,b_hi, e_lo,e_hi）。
// 覆盖：±0/±1/min&max normal/min&max subnormal/±Inf/NaN/2^53 的 add+div 组合、
//   除法 Q≈0.5/≈1 tie 边界、subnormal 结果（小÷大）、溢出（大÷小）、
//   加法大数相消、subnormal 相加、2^53 附近 RNE tie，外加 100+100 随机 u64。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>

union D { double d; uint64_t u; };
uint64_t db(double v) { D x; x.d = v; return x.u; }
double  bd(uint64_t v) { D x; x.u = v; return x.d; }
static const uint64_t CANON_NAN = 0x7FF8000000000000ull;
uint64_t canon(double v) { return std::isnan(v) ? CANON_NAN : db(v); }

bool write_all(const char* p, const void* d, size_t n) {
    FILE* fp = std::fopen(p, "wb");
    if (!fp) return false;
    std::fwrite(d, 1, n, fp);
    std::fclose(fp);
    return true;
}

int main(int argc, char** argv) {
    const char* out = argc >= 2 ? argv[1] : "test_fp64_min.bin";
    std::mt19937_64 rng(20260926ull);
    std::vector<uint32_t> w;
    auto emit = [&](uint32_t kind, uint64_t a, uint64_t b, uint64_t e) {
        w.push_back(kind);
        w.push_back((uint32_t)(a & 0xFFFFFFFFull)); w.push_back((uint32_t)(a >> 32));
        w.push_back((uint32_t)(b & 0xFFFFFFFFull)); w.push_back((uint32_t)(b >> 32));
        w.push_back((uint32_t)(e & 0xFFFFFFFFull)); w.push_back((uint32_t)(e >> 32));
    };

    // 关键值 × 关键值（add+div）
    const uint64_t spec[] = {
        0x0000000000000000ull, 0x8000000000000000ull,   // ±0
        0x3FF0000000000000ull, 0xBFF0000000000000ull,   // ±1.0
        0x0010000000000000ull,                          // min normal
        0x7FEFFFFFFFFFFFFFull,                          // max normal
        0x000FFFFFFFFFFFFFull,                          // max subnormal
        0x0000000000000001ull,                          // min subnormal
        0x7FF0000000000000ull, 0xFFF0000000000000ull,   // ±Inf
        0x7FF8000000000001ull,                          // NaN
        0x4340000000000000ull,                          // 2^53
        0x3FE0000000000000ull,                          // 0.5
    };
    for (uint64_t a : spec)
        for (uint64_t b : spec) {
            emit(0, a, b, canon(bd(a) + bd(b)));
            emit(1, a, b, canon(bd(a) / bd(b)));
        }

    // 除法 Q≈0.5/≈1 与 subnormal 结果/溢出
    const uint64_t edge_div[][2] = {
        {0x3FF0000000000000ull, 0x4000000000000000ull},
        {0x3FF0000000000000ull, 0x4008000000000000ull},
        {0x4000000000000000ull, 0x4008000000000000ull},
        {0x3FF0000000000000ull, 0x3FF8000000000000ull},
        {0x3FF8000000000000ull, 0x3FF0000000000000ull},
        {0x3FF0000000000000ull, 0x400FFFFFFFFFFFFFull},
        {0x3FE0000000000000ull, 0x3FF0000000000000ull},
        {0x3FEFFFFFFFFFFFFFull, 0x3FF0000000000000ull},
        {0x433FFFFFFFFFFFFFull, 0x4340000000000000ull},
        {0x433FFFFFFFFFFFFFull, 0x433FFFFFFFFFFFFFull},
        {0x3FF0000000000000ull, 0x7FE0000000000000ull},   // 1.0/2^1023 → subnormal
        {0x0000000000000001ull, 0x4000000000000000ull},   // 2^-1074/2 → 0
        {0x7FE0000000000000ull, 0x3FF0000000000000ull},   // 2^1023/1 → Inf
        {0x3FF0000000000000ull, 0x0010000000000000ull},   // 1/2^-1022 → 2^1022
    };
    for (auto& p : edge_div)
        emit(1, p[0], p[1], canon(bd(p[0]) / bd(p[1])));

    // 加法：相消 / subnormal 相加 / 2^53 附近 tie
    for (int i = 0; i < 10; ++i) {
        uint64_t x = (rng() & ((1ull << 52) - 1)) | ((uint64_t)(rng() % 300 + 900) << 52);
        if (rng() & 1) x |= 1ull << 63;
        emit(0, x, x ^ (1ull << 63), canon(bd(x) + bd(x ^ (1ull << 63))));
    }
    const uint64_t subs[] = {0x0000000000000001ull, 0x0000000000000003ull,
                             0x0008000000000000ull, 0x000FFFFFFFFFFFFFull};
    for (uint64_t x : subs)
        for (uint64_t y : subs) {
            emit(0, x, y, canon(bd(x) + bd(y)));
            emit(0, x, y ^ (1ull << 63), canon(bd(x) + bd(y ^ (1ull << 63))));
        }
    const uint64_t bigs2[] = {0x4340000000000000ull, 0x433FFFFFFFFFFFFFull,
                              0x4340000000000001ull};
    for (uint64_t x : bigs2)
        for (uint64_t y : bigs2)
            emit(0, x, y, canon(bd(x) + bd(y)));

    // 随机补充（u64 位型）
    for (int i = 0; i < 100; ++i) {
        uint64_t a = rng(), b = rng();
        emit(0, a, b, canon(bd(a) + bd(b)));
        emit(1, a, b, canon(bd(a) / bd(b)));
    }

    std::vector<uint32_t> all = {(uint32_t)(w.size() / 7)};
    all.insert(all.end(), w.begin(), w.end());
    if (!write_all(out, all.data(), all.size() * 4)) return 1;
    std::printf("[gen_fp64_min] %zu cases\n", w.size() / 7);
    return 0;
}
