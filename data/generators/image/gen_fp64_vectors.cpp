// gen_fp64_vectors.cpp — fp64_add / fp64_div 单元对拍向量生成
//------------------------------------------------------------------------------
// 输出 test_fp64.bin（小端，头部 u32 N，随后每组 7 个 u32）：
//   {kind, a_lo, a_hi, b_lo, b_hi, exp_lo, exp_hi}；u64 = {hi, lo}
//   kind: 0=add(a,b) 1=div(a,b)
// 期望 = C++ double 逐位结果（RNE）；NaN 统一映射为规范 QNaN 0x7FF8000000000000。
// 覆盖：全随机 u64 位型（自动含 subnormal/NaN/Inf）、域内 normal 样本、边界
//   （±0/±1/min&max normal/min&max subnormal/2^53/±Inf/NaN）、大数相消、
//   subnormal 结果（小÷大）与溢出（大÷小）。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstdlib>
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
    const char* out = argc >= 2 ? argv[1] : "test_fp64.bin";
    int N_A = argc >= 3 ? std::atoi(argv[2]) : 40000;   // 随机 add
    int N_D = argc >= 4 ? std::atoi(argv[3]) : 40000;   // 随机 div
    int N_AD = argc >= 5 ? std::atoi(argv[4]) : 20000;  // 域内 add/div 样本数
    std::mt19937_64 rng(20260925ull);
    std::vector<uint32_t> w;

    auto emit = [&](uint32_t kind, uint64_t a, uint64_t b, uint64_t e) {
        w.push_back(kind);
        w.push_back((uint32_t)(a & 0xFFFFFFFFull)); w.push_back((uint32_t)(a >> 32));
        w.push_back((uint32_t)(b & 0xFFFFFFFFull)); w.push_back((uint32_t)(b >> 32));
        w.push_back((uint32_t)(e & 0xFFFFFFFFull)); w.push_back((uint32_t)(e >> 32));
    };

    // ---- 全随机 u64 位型（覆盖 subnormal/NaN/Inf/溢出/下溢全部角落）----
    for (int i = 0; i < N_A; ++i) {
        uint64_t a = rng(), b = rng();
        emit(0, a, b, canon(bd(a) + bd(b)));
    }
    for (int i = 0; i < N_D; ++i) {
        uint64_t a = rng(), b = rng();
        emit(1, a, b, canon(bd(a) / bd(b)));
    }

    // ---- 域内样本（normal，指数 900..1100 附近）----
    for (int i = 0; i < N_AD; ++i) {
        uint64_t a = ((uint64_t)(rng() % 201 + 900) << 52) | (rng() & ((1ull << 52) - 1));
        uint64_t b = ((uint64_t)(rng() % 201 + 900) << 52) | (rng() & ((1ull << 52) - 1));
        if (rng() & 1) a |= 1ull << 63;
        if (rng() & 1) b |= 1ull << 63;
        emit(0, a, b, canon(bd(a) + bd(b)));
    }
    // 亚像素均值域：sum∈[1,1e9] 整数、n∈[1,4096]（除法多步步进）
    for (int i = 0; i < N_AD; ++i) {
        double a = (double)((int64_t)(rng() % 1000000000ull) + 1);
        double b = (double)((int64_t)(rng() % 4096ull) + 1);
        emit(1, db(a), db(b), canon(a / b));
    }

    // ---- 定向边界：spec×spec（add 与 div 各一轮）----
    const uint64_t spec[] = {
        0x0000000000000000ull,   // +0
        0x8000000000000000ull,   // -0
        0x3FF0000000000000ull,   // 1.0
        0xBFF0000000000000ull,   // -1.0
        0x0010000000000000ull,   // min normal
        0x7FEFFFFFFFFFFFFFull,   // max normal
        0x000FFFFFFFFFFFFFull,   // max subnormal
        0x0000000000000001ull,   // min subnormal
        0x0008000000000000ull,   // 中值 subnormal
        0x7FF0000000000000ull,   // +Inf
        0xFFF0000000000000ull,   // -Inf
        0x7FF8000000000001ull,   // NaN(payload)
        0x7FF0000000000001ull,   // sNaN 位型
        0x4340000000000000ull,   // 2^53
        0xC340000000000000ull,   // -2^53
        0x3FE0000000000000ull,   // 0.5
        0x4008000000000000ull,   // 3.0
    };
    for (uint64_t a : spec)
        for (uint64_t b : spec) {
            emit(0, a, b, canon(bd(a) + bd(b)));
            emit(1, a, b, canon(bd(a) / bd(b)));
        }

    // ---- 大数相消：x + (-x) → +0 ----
    for (int i = 0; i < 200; ++i) {
        uint64_t x = (rng() & ((1ull << 52) - 1)) | ((uint64_t)(rng() % 300 + 900) << 52);
        if (rng() & 1) x |= 1ull << 63;
        emit(0, x, x ^ (1ull << 63), canon(bd(x) + bd(x ^ (1ull << 63))));
    }

    // ---- subnormal 结果：小 ÷ 大（→ subnormal/0）与 大 ÷ 小（→ 溢出/Inf）----
    const uint64_t smalls[] = {
        0x3FF0000000000000ull, 0x0010000000000000ull, 0x0000000000000001ull,
        0x000FFFFFFFFFFFFFull, 0x3FE0000000000000ull, 0x3F00000000000000ull,
        0x3B00000000000000ull,
    };
    const uint64_t bigs[] = {
        0x7FEFFFFFFFFFFFFFull, 0x7FE0000000000000ull, 0x7FD0000000000000ull,
        0x43F0000000000000ull, 0x40F0000000000000ull,
    };
    for (uint64_t s : smalls)
        for (uint64_t bg : bigs) {
            emit(1, s, bg, canon(bd(s) / bd(bg)));
            emit(1, bg, s, canon(bd(bg) / bd(s)));
        }

    // ---- subnormal 相加（同号/异号）----
    const uint64_t subs[] = {0x0000000000000001ull, 0x0000000000000002ull,
                             0x0000000000000003ull, 0x0008000000000000ull,
                             0x000FFFFFFFFFFFFFull, 0x0010000000000000ull};
    for (uint64_t x : subs)
        for (uint64_t y : subs) {
            emit(0, x, y, canon(bd(x) + bd(y)));
            emit(0, x, y ^ (1ull << 63), canon(bd(x) + bd(y ^ (1ull << 63))));
        }

    // ---- 2^53 附近累加（RNE tie 密集区）----
    const uint64_t bigs2[] = {0x4340000000000000ull, 0x433FFFFFFFFFFFFFull,
                              0x4340000000000001ull};
    for (uint64_t x : bigs2)
        for (uint64_t y : bigs2)
            emit(0, x, y, canon(bd(x) + bd(y)));

    // ---- 除法尾数舍入边界：Q≈0.5 / Q≈1 / 精确 tie（!ageb 尾数取位回归）----
    const uint64_t edge_div[][2] = {
        {0x3FF0000000000000ull, 0x4000000000000000ull},   // 1.0/2.0 精确 0.5
        {0x3FF0000000000000ull, 0x4008000000000000ull},   // 1.0/3.0 !ageb 舍入
        {0x4000000000000000ull, 0x4008000000000000ull},   // 2.0/3.0
        {0x3FF0000000000000ull, 0x3FF8000000000000ull},   // 1.0/1.5 Q=2/3
        {0x3FF8000000000000ull, 0x3FF0000000000000ull},   // 1.5/1.0 ageb
        {0x3FF0000000000000ull, 0x400FFFFFFFFFFFFFull},   // 1.0/(2-2^-52) Q≈0.5+ulp
        {0x3FE0000000000000ull, 0x3FF0000000000000ull},   // 0.5/1.0
        {0x3FEFFFFFFFFFFFFFull, 0x3FF0000000000000ull},   // (0.5-ulp)/1.0
        {0x4340000000000000ull, 0x4330000000000000ull},   // 2^53/2^52
        {0x433FFFFFFFFFFFFFull, 0x4340000000000000ull},   // (2^53-1)/2^53 Q≈1-ulp
        {0x433FFFFFFFFFFFFFull, 0x433FFFFFFFFFFFFFull},   // 1.0 精确
        {0x3FF0000000000000ull, 0x400FFFFFFFFFFFFFull},   // 1.0/2.0(尾数全1) tie 邻域
    };
    for (auto& p : edge_div)
        emit(1, p[0], p[1], canon(bd(p[0]) / bd(p[1])));
    // 随机但强制 Q∈[0.5,1) 邻域（a 为 1..2^52 整数，b=2a±1 → 商≈0.5 tie）
    for (int i = 0; i < 500; ++i) {
        uint64_t k = (rng() % 4000000ull) + 1000ull;
        double a = (double)k;
        double b = (double)(2ull * k + (rng() & 1));
        emit(1, db(a), db(b), canon(a / b));
    }

    std::vector<uint32_t> all = {(uint32_t)(w.size() / 7)};
    all.insert(all.end(), w.begin(), w.end());
    if (!write_all(out, all.data(), all.size() * 4)) return 1;
    std::printf("[gen_fp64] %zu cases (add+div)\n", w.size() / 7);
    return 0;
}
