// gen_lround_vectors.cpp — M3 ring_check 配套工具
//------------------------------------------------------------------------------
// 1) 打印关键浮点常量位模式（0.22f/0.28f/0.5f/0.25f/4/18/32/20/0.75/1.25），
//    供 RTL 常量核对（0.22f、0.28f 用 g++ 本机浮点值）。
// 2) 校验 ring_cos_sin.mem 与 g++ libm cosf/sinf 的一致性（行间交错布局：
//    mem[2k]=cos(2πk/32)，mem[2k+1]=sin(2πk/32)，0 基）。
// 3) 用 m3_<scene>_merge3.bin + m3_<scene>_gray.bin 重算 nearest/ring，
//    与 m3_<scene>_nearest.bin / m3_<scene>_ring.bin 位级对拍（复刻
//    candidates.cpp::nearest_distance / alternating_ring 与
//    tests/rtl/export_m3.cpp::ring_detail_，cos/sin 直接取 ROM 值，
//    与 RTL 输入完全一致）。全过打印 SANITY OK。
// 4) 生成 tests/build/vectors/lround_vec.mem：每行 "<fp32位模式> <lround
//    结果s32>"，覆盖 ±0.5 边界、随机位模式，供 tb_ring.sv 对 lround_f32
//    函数独立位级验证。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <string>
#include <algorithm>

static constexpr float pi = 3.14159265358979323846f;

uint32_t fb(float v) { union { float f; uint32_t u; } x; x.f = v; return x.u; }
float bf(uint32_t u) { union { float f; uint32_t u; } x; x.u = u; return x.f; }

static bool read_all(const std::string& p, std::vector<uint8_t>& d) {
    FILE* fp = std::fopen(p.c_str(), "rb");
    if (!fp) return false;
    std::fseek(fp, 0, SEEK_END);
    long n = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    d.resize((size_t)n);
    std::fread(d.data(), 1, (size_t)n, fp);
    std::fclose(fp);
    return true;
}
static uint32_t rd32(const std::vector<uint8_t>& d, size_t b) {
    return (uint32_t)d[b] | ((uint32_t)d[b + 1] << 8) |
           ((uint32_t)d[b + 2] << 16) | ((uint32_t)d[b + 3] << 24);
}

// ---- 与 candidates.cpp 一致的复刻（cos/sin 用 ROM 值）----
static float dist_(float ax, float ay, float bx, float by) {
    return std::hypot(ax - bx, ay - by);
}
static float nearest_(const std::vector<float>& xs, const std::vector<float>& ys, size_t i) {
    float r = std::numeric_limits<float>::max();
    for (size_t j = 0; j < xs.size(); ++j)
        if (i != j) r = std::min(r, dist_(xs[i], ys[i], xs[j], ys[j]));
    return r;
}

struct RingDetail { float hi, lo, thr, opp_err; int ntrans; bool sector_ok, pass; };

static bool ring_detail_(const std::vector<uint8_t>& gray, int W, int H,
                         float px, float py, float radius,
                         const float cosv[32], const float sinv[32], RingDetail& d) {
    d = {};
    if (px < radius + 1 || py < radius + 1 || px >= (float)W - radius - 1 ||
        py >= (float)H - radius - 1)
        return false;
    float values[32], smooth[32];
    for (int k = 0; k < 32; ++k) {
        int sx = (int)std::lround(px + radius * cosv[k]);
        int sy = (int)std::lround(py + radius * sinv[k]);
        values[k] = (float)gray[(size_t)sy * W + sx];
    }
    float lo = 255, hi = 0;
    for (int k = 0; k < 32; ++k) {
        smooth[k] = (values[(k + 31) % 32] + 2 * values[k] + values[(k + 1) % 32]) / 4;
        lo = std::min(lo, smooth[k]);
        hi = std::max(hi, smooth[k]);
    }
    if (hi - lo < 20) return false;
    float threshold = (hi + lo) * 0.5f;
    std::vector<int> transitions;
    float opposite_error = 0;
    for (int k = 0; k < 32; ++k) {
        if ((smooth[k] > threshold) != (smooth[(k + 31) % 32] > threshold))
            transitions.push_back(k);
        opposite_error += std::fabs(smooth[k] - smooth[(k + 16) % 32]);
    }
    d.hi = hi; d.lo = lo; d.thr = threshold;
    d.opp_err = opposite_error; d.ntrans = (int)transitions.size();
    if (transitions.size() != 4 || opposite_error > 32 * (hi - lo) * 0.28f) {
        d.sector_ok = false;
        return false;
    }
    d.sector_ok = true;
    for (int k = 0; k < 4; ++k) {
        int length = (transitions[(k + 1) % 4] - transitions[k] + 32) % 32;
        if (length < 3 || length > 13) return false;
    }
    d.pass = true;
    return true;
}

static int check_scene(const std::string& scene, const std::string& dir,
                       int W, int H, const float cosv[32], const float sinv[32]) {
    std::vector<uint8_t> d, g;
    std::string base = dir + "/m3_" + scene;
    if (!read_all(base + "_merge3.bin", d) || !read_all(base + "_gray.bin", g)) {
        std::printf("[%s] skip (no vectors)\n", scene.c_str());
        return 0;
    }
    uint32_t N = rd32(d, 0);
    std::vector<float> xs, ys;
    for (uint32_t k = 0; k < N; ++k) {
        xs.push_back(bf(rd32(d, 4 + (size_t)k * 8)));
        ys.push_back(bf(rd32(d, 4 + (size_t)k * 8 + 4)));
    }
    // nearest
    std::vector<uint8_t> dn;
    read_all(base + "_nearest.bin", dn);
    int nf = 0;
    for (uint32_t i = 0; i < N; ++i) {
        float sp = nearest_(xs, ys, i);
        float radius = std::clamp(sp * 0.22f, 4.0f, 18.0f);
        uint32_t esp = rd32(dn, 4 + (size_t)i * 8);
        uint32_t erad = rd32(dn, 4 + (size_t)i * 8 + 4);
        if (fb(sp) != esp || fb(radius) != erad) {
            ++nf;
            if (nf <= 10)
                std::printf("[%s][NEAREST FAIL] i=%u got %08x/%08x exp %08x/%08x\n",
                            scene.c_str(), i, fb(sp), fb(radius), esp, erad);
        }
    }
    // ring
    std::vector<uint8_t> dr;
    read_all(base + "_ring.bin", dr);
    int rf = 0;
    for (uint32_t i = 0; i < N; ++i) {
        float sp = nearest_(xs, ys, i);
        float radius = std::clamp(sp * 0.22f, 4.0f, 18.0f);
        RingDetail d0, dA, dB;
        ring_detail_(g, W, H, xs[i], ys[i], radius, cosv, sinv, d0);
        ring_detail_(g, W, H, xs[i], ys[i], radius * 0.75f, cosv, sinv, dA);
        ring_detail_(g, W, H, xs[i], ys[i], radius * 1.25f, cosv, sinv, dB);
        bool pass = d0.pass && (dA.pass || dB.pass);
        size_t o = 4 + (size_t)i * 28;
        uint32_t e_hi = rd32(dr, o), e_lo = rd32(dr, o + 4), e_thr = rd32(dr, o + 8);
        uint32_t e_ntr = rd32(dr, o + 12), e_oe = rd32(dr, o + 16);
        uint32_t e_so = rd32(dr, o + 20), e_ps = rd32(dr, o + 24);
        if (fb(d0.hi) != e_hi || fb(d0.lo) != e_lo || fb(d0.thr) != e_thr ||
            (uint32_t)d0.ntrans != e_ntr || fb(d0.opp_err) != e_oe ||
            (d0.sector_ok ? 1u : 0u) != e_so || (pass ? 1u : 0u) != e_ps) {
            ++rf;
            if (rf <= 10)
                std::printf("[%s][RING FAIL] i=%u got hi=%08x lo=%08x thr=%08x ntr=%u "
                            "oe=%08x so=%u pass=%u | exp %08x %08x %08x %u %08x %u %u\n",
                            scene.c_str(), i, fb(d0.hi), fb(d0.lo), fb(d0.thr),
                            d0.ntrans, fb(d0.opp_err), d0.sector_ok, pass,
                            e_hi, e_lo, e_thr, e_ntr, e_oe, e_so, e_ps);
        }
    }
    std::printf("[%s] N=%u nearest_fail=%d ring_fail=%d\n", scene.c_str(), N, nf, rf);
    return (nf == 0 && rf == 0) ? 1 : 0;
}

int main(int argc, char** argv) {
    std::string dir = argc >= 2 ? argv[1] : "vectors";

    std::printf("0.22f=%08x 0.28f=%08x 0.5f=%08x 0.25f=%08x\n",
                fb(0.22f), fb(0.28f), fb(0.5f), fb(0.25f));
    std::printf("4.0f=%08x 18.0f=%08x 32.0f=%08x 20.0f=%08x\n",
                fb(4.0f), fb(18.0f), fb(32.0f), fb(20.0f));
    std::printf("0.75f=%08x 1.25f=%08x FLT_MAX=%08x\n",
                fb(0.75f), fb(1.25f), fb(std::numeric_limits<float>::max()));

    // ROM 校验 + 装入 cos/sin（文本行解析：0 基行 2k=cos，2k+1=sin）
    std::string rompath = dir + "/ring_cos_sin.mem";
    float cosv[32], sinv[32];
    int rom_bad = 0;
    FILE* fp = std::fopen(rompath.c_str(), "r");
    if (!fp) { std::printf("[FATAL] cannot open ROM\n"); return 1; }
    char line[64];
    uint32_t vals[64];
    int n = 0;
    while (n < 64 && std::fgets(line, sizeof line, fp)) {
        uint32_t v = 0;
        if (std::sscanf(line, "%x", &v) == 1) vals[n++] = v;
    }
    std::fclose(fp);
    if (n != 64) { std::printf("[FATAL] ROM lines=%d\n", n); return 1; }
    for (int k = 0; k < 32; ++k) {
        cosv[k] = bf(vals[2 * k]);      // 0 基行 2k = cos(2πk/32)
        sinv[k] = bf(vals[2 * k + 1]);  // 0 基行 2k+1 = sin(2πk/32)
        float ec = std::cos(2 * pi * k / 32);
        float es = std::sin(2 * pi * k / 32);
        if (fb(ec) != vals[2 * k] || fb(es) != vals[2 * k + 1]) {
            ++rom_bad;
            if (rom_bad <= 5)
                std::printf("[ROM] k=%d cos got %08x exp %08x | sin got %08x exp %08x\n",
                            k, vals[2 * k], fb(ec), vals[2 * k + 1], fb(es));
        }
    }
    std::printf("ring_cos_sin.mem: %d mismatches vs cosf/sinf\n", rom_bad);

    int ok = 1;
    ok &= check_scene("small", dir, 32, 24, cosv, sinv);
    ok &= check_scene("texture", dir, 128, 96, cosv, sinv);

    // lround 向量
    FILE* fp2 = std::fopen((dir + "/lround_vec.mem").c_str(), "w");
    if (!fp2) { std::printf("[FATAL] cannot write lround_vec.mem\n"); return 1; }
    std::vector<uint32_t> special = {
        0x00000000, 0x80000000, 0x3f000000, 0xbf000000,  // ±0.5
        0x3fc00000, 0xbfc00000, 0x40200000, 0xc0200000,  // ±1.5
        0x40200001, 0xc0200001,                          // 1.5000001
        0x401fffff, 0xc01fffff,                          // 1.4999999
        0x3effffff, 0xbeffffff,                          // 0.49999997
        0x3f000001, 0xbf000001,                          // 0.50000006
        0x40600000, 0xc0600000, 0x40e00000, 0xc0e00000,  // ±3.5 ±7.5
        0x437f0000, 0xc37f0000,                          // ±255.5
        0x437effff, 0xc37effff,                          // 255.4999
        0x43800000, 0xc3800000,                          // ±256.0
        0x3f7fffff, 0xbf7fffff,                          // 0.9999999
        0x3f800000, 0xbf800000, 0x3f800001, 0xbf800001,  // ±1.0, 1.0000001
        0x4b000000, 0xcb000000,                          // ±2^23
        0x4b000001, 0xcb000001,                          // ±(2^23+1)
        0x4b7fffff, 0xcb7fffff,                          // ±(2^24-1)
        0x4c000000, 0xcc000000,                          // ±2^25
        0x3e800000, 0xbe800000,                          // ±0.25
        0x3dffffff, 0xbdffffff,                          // 0.4999...
    };
    std::mt19937 rng(987654321u);
    int kept = 0;
    while (kept < 600) {
        uint32_t b = rng();
        float f = bf(b);
        if (std::isfinite(f) && std::fabs((double)f) < 2000000.0) {
            special.push_back(b);
            ++kept;
        }
    }
    for (uint32_t b : special) {
        long r = std::lround((double)bf(b));   // 小值域内与 lround(float) 一致
        std::fprintf(fp2, "%08x %08x\n", b, (uint32_t)(int32_t)r);
    }
    std::fclose(fp2);
    std::printf("lround_vec.mem written: %zu entries\n", special.size());

    std::printf(ok ? "SANITY OK\n" : "SANITY FAIL\n");
    return ok ? 0 : 1;
}
