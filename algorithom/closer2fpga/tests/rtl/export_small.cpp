// export_small.cpp — 小图全链对拍向量导出（M2 集成验证用）
//------------------------------------------------------------------------------
// 生成一张合成小图（棋盘格），走完整 C++ 参考链路，输出 RTL 对拍向量：
//   small_bgr.bin / small_resp.bin / small_candidates.bin / small.dim
// 目的：tb_resp 用 32×24 小图即可在秒级跑完整个 BGR→resp→candidates 链，
//   规避 1280×720 全图的仿真时间爆炸（me 链 ~10ms 仿真需 70 秒墙钟）。
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include "../../closer2fpga/algo/shi_tomasi.h"
#include "../../closer2fpga/kernels/color.h"

namespace {
constexpr int W = 32, H = 24;
constexpr float kThresholdRatio = 0.08f;
constexpr int   kWinSize       = 3;

uint32_t f32_bits(float v) {
    union { float f; uint32_t u; } x;
    x.f = v;
    return x.u;
}
bool write_all(const std::string& p, const void* d, size_t n) {
    FILE* fp = std::fopen(p.c_str(), "wb");
    if (!fp) return false;
    std::fwrite(d, 1, n, fp);
    std::fclose(fp);
    return true;
}
} // namespace

int main(int argc, char** argv) {
    std::string out_dir = argc >= 2 ? argv[1] : ".";

    // 合成棋盘图（灰度），生成 BGR888
    GrayImage gray(W, H);
    std::vector<uint8_t> bgr((size_t)W * H * 3);
    int sq = 4;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            int g = ((x / sq + y / sq) & 1) ? 200 : 30;   // 棋盘 200/30
            gray.set(x, y, g);
            size_t i = ((size_t)y * W + x) * 3;
            bgr[i] = g; bgr[i+1] = g; bgr[i+2] = g;       // BGR 全等
        }

    // 完整参考链路
    FloatMap Ix, Iy, resp;
    sobel_xy(gray, Ix, Iy);
    shi_tomasi_response(Ix, Iy, resp, kWinSize);

    float rmax = *std::max_element(resp.data, resp.data + W * H);
    float thr  = rmax * std::clamp(kThresholdRatio, 0.001f, 1.0f);

    std::vector<Point2f> corners;
    shi_tomasi_detect(gray, corners, kThresholdRatio, kWinSize);

    // 写出
    std::vector<uint8_t> rbytes((size_t)W * H * 4);
    for (int i = 0; i < W * H; ++i) {
        uint32_t b = f32_bits(resp.data[i]);
        rbytes[i*4]   = b & 0xFF;
        rbytes[i*4+1] = (b >> 8) & 0xFF;
        rbytes[i*4+2] = (b >> 16) & 0xFF;
        rbytes[i*4+3] = (b >> 24) & 0xFF;
    }
    std::vector<uint32_t> cw;
    cw.push_back((uint32_t)corners.size());
    for (auto& p : corners) {
        cw.push_back((uint32_t)(int)p.x);
        cw.push_back((uint32_t)(int)p.y);
    }

    char dim[16];
    std::snprintf(dim, sizeof(dim), "%d %d", W, H);

    if (!write_all(out_dir + "/small_bgr.bin", bgr.data(), bgr.size()) ||
        !write_all(out_dir + "/small_resp.bin", rbytes.data(), rbytes.size()) ||
        !write_all(out_dir + "/small_candidates.bin", cw.data(), cw.size() * 4) ||
        !write_all(out_dir + "/small.dim", dim, 6))
        return 1;

    std::printf("[export] small %dx%d: rmax=%.6g thr=%.6g candidates=%zu\n",
                W, H, rmax, thr, corners.size());
    return 0;
}
