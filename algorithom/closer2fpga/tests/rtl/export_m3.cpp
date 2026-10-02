// export_m3.cpp — M3 候选后处理对拍向量导出（无 subpixel 变体）
//------------------------------------------------------------------------------
// 用途：M3 阶段候选处理（MERGE→SUBPIXEL(占位)→MERGE→NEAREST→RING）的 RTL 对拍
//   依据。subpixel 核在 M5，因此本工具导出**跳过 subpixel** 的变体流程
//   （与 detect_native 唯一差异即省去 refine_subpixel 一步，显式标注，
//   不属"悄悄降精度"；subpixel 独立里程碑时用带 subpixel 版本重对拍）。
//
// 复刻逻辑逐行取自 closer2fpga/algo/chessboard/candidates.cpp（merge_duplicates、
//   nearest_distance、alternating_ring）——保持浮点运算顺序与参考一致。
//
// 场景：
//   small  : 32×24 棋盘（复用 tests/build/vectors/small_bgr.bin）
//   texture: 128×96 合成纹理+棋盘（新增，覆盖高候选/近距场景）
//
// 输出（小端，坐标/浮点均以 fp32 位模式存，标量用 32 位）：
//   m3_<scene>_in.bin      u32 N + N×{x,y}
//   m3_<scene>_merge5.bin  u32 N + N×{x,y}
//   m3_<scene>_merge3.bin  u32 N + N×{x,y}
//   m3_<scene>_nearest.bin u32 N + N×{spacing_bits, radius_bits}
//   m3_<scene>_ring.bin    u32 N + N×{hi,lo,thr,ntrans,opp_err,sector_ok,pass_r}
//   m3_<scene>_inner.bin   u32 N + N×{x,y}
//   m3_<scene>_gray.bin    W*H 字节（RTL TB 作灰度随机访问用）
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include "../../closer2fpga/algo/shi_tomasi.h"
#include "../../closer2fpga/kernels/color.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

// 与 candidates.cpp 一致的逐行复刻 ----------------------------------------
float dist_(float ax, float ay, float bx, float by) {
    return std::hypot(ax - bx, ay - by);
}

void merge_dup_(std::vector<Point2f>& pts, float radius) {
    std::vector<bool> used(pts.size(), false);
    std::vector<Point2f> out;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (used[i]) continue;
        Point2f sum = pts[i];
        int n = 1;
        for (size_t j = i + 1; j < pts.size(); ++j) {
            if (!used[j] && dist_(pts[i].x, pts[i].y, pts[j].x, pts[j].y) < radius) {
                used[j] = true;
                sum.x += pts[j].x;
                sum.y += pts[j].y;
                ++n;
            }
        }
        out.push_back({sum.x / n, sum.y / n});
    }
    pts = std::move(out);
}

float nearest_(const std::vector<Point2f>& pts, size_t i) {
    float r = std::numeric_limits<float>::max();
    for (size_t j = 0; j < pts.size(); ++j)
        if (i != j)
            r = std::min(r, dist_(pts[i].x, pts[i].y, pts[j].x, pts[j].y));
    return r;
}

// 复刻 alternating_ring 的逐点中间量（供 ring_check RTL 对拍）
struct RingDetail {
    float hi, lo, thr, opp_err;
    int   ntrans;
    bool  sector_ok, pass;
};

bool ring_detail_(const GrayImage& img, Point2f p, float radius, RingDetail& d) {
    d = {};
    if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 1 ||
        p.y >= img.h - radius - 1)
        return false;
    float values[32], smooth[32];
    for (int k = 0; k < 32; ++k) {
        float a = 2 * pi * k / 32;
        values[k] = float(img.get(int(std::lround(p.x + radius * std::cos(a))),
                                  int(std::lround(p.y + radius * std::sin(a)))));
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

uint32_t fb(float v) { union { float f; uint32_t u; } x; x.f = v; return x.u; }
bool write_all(const std::string& p, const void* d, size_t n) {
    FILE* fp = std::fopen(p.c_str(), "wb");
    if (!fp) return false;
    std::fwrite(d, 1, n, fp);
    std::fclose(fp);
    return true;
}
template <typename T> bool write_vec(const std::string& p, const std::vector<T>& v) {
    return write_all(p, v.data(), v.size() * sizeof(T));
}

void emit_pts(const std::string& path, const std::vector<Point2f>& pts) {
    std::vector<uint32_t> w;
    w.push_back((uint32_t)pts.size());
    for (auto& p : pts) { w.push_back(fb(p.x)); w.push_back(fb(p.y)); }
    write_vec(path, w);
}

void export_scene(const std::string& scene, int W, int H,
                  const std::vector<uint8_t>& bgr, const std::string& out_dir) {
    GrayImage gray(W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            size_t i = ((size_t)y * W + x) * 3;
            gray.set(x, y, kernels::bgr_to_gray(bgr[i], bgr[i + 1], bgr[i + 2]));
        }
    std::vector<uint8_t> grayb((size_t)W * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            grayb[(size_t)y * W + x] = gray.get(x, y);
    write_all(out_dir + "/m3_" + scene + "_gray.bin", grayb.data(), grayb.size());

    std::vector<Point2f> cand;
    shi_tomasi_detect(gray, cand, 0.08f, 3);
    emit_pts(out_dir + "/m3_" + scene + "_in.bin", cand);
    std::printf("[%s] shi_tomasi: %zu\n", scene.c_str(), cand.size());

    merge_dup_(cand, 5.0f);
    emit_pts(out_dir + "/m3_" + scene + "_merge5.bin", cand);
    std::printf("[%s] after merge5: %zu\n", scene.c_str(), cand.size());

    // 无 subpixel（变体）
    merge_dup_(cand, 3.0f);
    emit_pts(out_dir + "/m3_" + scene + "_merge3.bin", cand);
    std::printf("[%s] after merge3: %zu\n", scene.c_str(), cand.size());

    std::vector<uint32_t> nw;
    nw.push_back((uint32_t)cand.size());
    for (size_t i = 0; i < cand.size(); ++i) {
        float sp = nearest_(cand, i);
        float radius = std::clamp(sp * 0.22f, 4.0f, 18.0f);
        nw.push_back(fb(sp)); nw.push_back(fb(radius));
    }
    write_vec(out_dir + "/m3_" + scene + "_nearest.bin", nw);

    std::vector<uint32_t> rw;
    rw.push_back((uint32_t)cand.size());
    std::vector<Point2f> inner;
    for (size_t i = 0; i < cand.size(); ++i) {
        float sp = nearest_(cand, i);
        float radius = std::clamp(sp * 0.22f, 4.0f, 18.0f);
        RingDetail d0, d1, d2;
        bool ok0 = ring_detail_(gray, cand[i], radius, d0);
        bool okA = ring_detail_(gray, cand[i], radius * 0.75f, d1);
        bool okB = ring_detail_(gray, cand[i], radius * 1.25f, d2);
        bool pass = ok0 && (okA || okB);
        rw.push_back(fb(d0.hi)); rw.push_back(fb(d0.lo)); rw.push_back(fb(d0.thr));
        rw.push_back((uint32_t)d0.ntrans);
        rw.push_back(fb(d0.opp_err));
        rw.push_back(d0.sector_ok ? 1u : 0u);
        rw.push_back(pass ? 1u : 0u);
        if (pass)
            inner.push_back(cand[i]);
    }
    write_vec(out_dir + "/m3_" + scene + "_ring.bin", rw);
    emit_pts(out_dir + "/m3_" + scene + "_inner.bin", inner);
    std::printf("[%s] inner ring: %zu\n", scene.c_str(), inner.size());
}

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = argc >= 2 ? argv[1] : ".";

    // scene: small（复用 small_bgr.bin）
    {
        int W = 32, H = 24;
        std::vector<uint8_t> bgr;
        FILE* fp = std::fopen((out_dir + "/small_bgr.bin").c_str(), "rb");
        if (fp) {
            bgr.assign((size_t)W * H * 3, 0);
            std::fread(bgr.data(), 1, bgr.size(), fp);
            std::fclose(fp);
            export_scene("small", W, H, bgr, out_dir);
        } else {
            std::printf("[small] skip (no small_bgr.bin)\n");
        }
    }

    // scene: texture（合成纹理+棋盘，128×96）
    {
        int W = 128, H = 96;
        std::vector<uint8_t> bgr((size_t)W * H * 3);
        std::mt19937 rng(12345u);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                int g;
                bool board = (x >= 16 && x < 80 && y >= 8 && y < 56);
                if (board)
                    g = ((x / 8 + y / 8) & 1) ? 180 : 40;
                else
                    g = (int)(rng() % 256);   // 纹理区（高候选）
                size_t i = ((size_t)y * W + x) * 3;
                bgr[i] = g; bgr[i + 1] = g; bgr[i + 2] = g;
            }
        export_scene("texture", W, H, bgr, out_dir);
    }

    std::printf("[export_m3] DONE\n");
    return 0;
}
