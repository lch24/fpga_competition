// gen_fp32_vectors.cpp — fp32_hypot / fp32_div / fp32_sqrt 单元对拍向量
//------------------------------------------------------------------------------
// 输出 test_fp32.bin（小端，混合三种样本，头部 u32 N 后逐组 3 个 u32）：
//   {kind, a_bits, b_bits, exp_bits}；kind: 0=hypot(a,b) 1=div(a,b) 2=sqrt(a)
// 值域覆盖 merge/nearest/ring 全部浮点场景（整数坐标差 + 小数均值）。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>

union F { float f; uint32_t u; };
uint32_t fb(float v) { F x; x.f = v; return x.u; }

bool write_all(const char* p, const void* d, size_t n) {
    FILE* fp = std::fopen(p, "wb");
    if (!fp) return false;
    std::fwrite(d, 1, n, fp);
    std::fclose(fp);
    return true;
}

int main(int argc, char** argv) {
    const char* out = argc >= 2 ? argv[1] : "test_fp32.bin";
    int N_H = argc >= 3 ? std::atoi(argv[2]) : 20000;   // 每类样本数（默认全量）
    int N_D = argc >= 3 ? std::atoi(argv[2]) : 25000;
    int N_S = argc >= 3 ? std::atoi(argv[2]) : 20000;
    std::mt19937 rng(20260924u);
    std::vector<uint32_t> w;

    auto emit = [&](uint32_t kind, float a, float b, float e) {
        w.push_back(kind); w.push_back(fb(a)); w.push_back(fb(b)); w.push_back(fb(e));
    };

    // hypot：整数坐标差（±12000）+ 小数均值差
    for (int i = 0; i < N_H; ++i) {
        int dx = (int)(rng() % 24001) - 12000;
        int dy = (int)(rng() % 24001) - 12000;
        emit(0, (float)dx, (float)dy, std::hypot((float)dx, (float)dy));
    }
    for (int i = 0; i < N_H / 4; ++i) {
        float fx = ((int)(rng() % 48001) - 24000) / 4.0f;
        float fy = ((int)(rng() % 48001) - 24000) / 4.0f;
        emit(0, fx, fy, std::hypot(fx, fy));
    }
    // div：merge 均值场景（sum∈[1,2e8], n∈[1,4096]）
    for (int i = 0; i < N_D; ++i) {
        float a = (float)((int)(rng() % 200000000) + 1);
        float b = (float)((int)(rng() % 4096) + 1);
        emit(1, a, b, a / b);
    }
    // sqrt：hypot 的平方和域
    for (int i = 0; i < N_S; ++i) {
        float x = (float)((int)(rng() % 200000000) + 1);
        emit(2, x, 0.0f, std::sqrt(x));
    }
    // 边界：完全平方、半整数、1/4 步进
    const float specials[] = {0.25f, 0.5f, 0.75f, 1.0f, 2.0f, 1024.0f, 999999.0f};
    for (float s : specials) {
        emit(2, s, 0.0f, std::sqrt(s));
        emit(0, s, s, std::hypot(s, s));
        emit(1, 3.0f, s, 3.0f / s);
    }

    std::vector<uint32_t> all = {(uint32_t)(w.size() / 4)};
    all.insert(all.end(), w.begin(), w.end());
    if (!write_all(out, all.data(), all.size() * 4)) return 1;
    std::printf("[gen_fp32] %zu cases\n", w.size() / 4);
    return 0;
}
