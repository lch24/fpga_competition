// export_vectors.cpp — RTL 对拍向量导出工具（M1）
//------------------------------------------------------------------------------
// 目的：把 C++ 参考实现的逐阶段中间结果导出为二进制文件，供 RTL 仿真
//       testbench 逐项比对（对应 VERILOG_DESIGN_PLAN 第11节"对拍向量"）。
//
// 本工具不依赖 OpenCV：JPEG 解码由 jpg_to_bgr.ps1（.NET System.Drawing）
// 预先完成，本程序读取裸 BGR 字节流。这样整个导出链在任何成员机器上
// 都可复现（g++ 或 cl 均可编译），且 RTL 对拍的输入就是这份 .bgr 文件，
// 与写入 DDR 模型的字节完全一致（自洽）。
//
// 数据路径（与 FPGA 硬件路径一致）：
//   BGR888 (.bgr 文件) → kernels::bgr_to_gray → GrayImage
//   → sobel_xy → Ix/Iy (f32)
//   → shi_tomasi_response(win=3) → resp (f32)
//   → rmax（全图最大响应，帧屏障）
//   → thr = rmax * clamp(0.08, 0.001, 1.0)
//   → 3x3 NMS（与 shi_tomasi.cpp 完全一致的复刻）
//   → 候选点列表（detect_chessboard 在 candidates.cpp 中使用的同一组参数：
//     shi_tomasi_detect(gray, candidates, 0.08f, 3)）
//
// 自校验：本地复刻的 rmax/thr/NMS 结果与官方 shi_tomasi_detect 输出
//         逐点对账，不一致则报错退出——保证导出向量与参考实现一致。
//
// 输入（raw_dir 下，由 jpg_to_bgr.ps1 生成）：
//   testN.bgr : W*H*3 字节，BGR 逐像素、行主序、无行填充
//   testN.dim : ASCII "W H"
//
// 输出（out_dir 下，均为小端裸数据）：
//   testN_bgr.bin        W*H*3 字节（原样拷贝，DDR 模型初始化用）
//   testN_gray.bin       W*H 字节
//   testN_ix.bin         W*H 个 f32（IEEE754）
//   testN_iy.bin         W*H 个 f32（IEEE754）
//   testN_sum.bin        W*H*3 个 f32：每像素 {xx和, xy和, yy和}（光栅序）
//   testN_resp.bin       W*H 个 f32
//   testN_candidates.bin u32 数量 N，随后 N 组 {i32 x, i32 y}
//   testN_manifest.txt   尺寸/参数/关键标量（含 f32 位模式十六进制）
//
// 用法：export_vectors.exe <raw_dir> <out_dir>
//
// 版本记录（数据格式变更必须更新）：
//   v1  首版：BGR/gray/Ix/Iy/resp/rmax/thr/候选点
//   v2  增加 _sum.bin（张量 3×3 窗口和，ZERO 边界），用于前端流水分段对拍
//   v3  增加 test0_eigen.bin（min_eigen 单元对拍：三元组→resp 位模式）
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <random>

#include "../../closer2fpga/algo/shi_tomasi.h"
#include "../../closer2fpga/kernels/color.h"
#include "../../closer2fpga/kernels/gradient.h"

namespace {

constexpr float kThresholdRatio = 0.08f;   // candidates.cpp:80 的实参
constexpr int   kWinSize       = 3;        // candidates.cpp:80 的实参

// 边界安全取值（图外返回 def；与 shi_tomasi.cpp 的 safe_get 语义一致）
static float safe_get_(const FloatMap& m, int x, int y, float def) {
    if (x < 0 || x >= m.w || y < 0 || y >= m.h)
        return def;
    return m.get(x, y);
}

//------------------------------------------------------------------------------
// 小工具
//------------------------------------------------------------------------------
union F32Bits {
    float    f;
    uint32_t u;
};

uint32_t f32_bits(float v) {
    F32Bits fb;
    fb.f = v;
    return fb.u;
}

bool read_file(const std::string& path, std::vector<uint8_t>& out) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) return false;
    std::fseek(fp, 0, SEEK_END);
    long n = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    out.resize((size_t)n);
    size_t rd = n ? std::fread(out.data(), 1, (size_t)n, fp) : 0;
    std::fclose(fp);
    return rd == (size_t)n;
}

bool write_all(const std::string& path, const void* data, size_t bytes) {
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;
    size_t wrote = std::fwrite(data, 1, bytes, fp);
    std::fclose(fp);
    return wrote == bytes;
}

template <typename T>
bool write_vec(const std::string& path, const std::vector<T>& v) {
    return write_all(path, v.data(), v.size() * sizeof(T));
}

bool append_manifest(const std::string& path, const std::string& line) {
    FILE* fp = std::fopen(path.c_str(), "a");
    if (!fp) return false;
    std::fputs(line.c_str(), fp);
    std::fclose(fp);
    return true;
}

//------------------------------------------------------------------------------
// 复刻 shi_tomasi.cpp 的张量 3×3 窗口和（ZERO 边界）—— v2 新增
// 输出 sums：每像素 3 个 f32 {xx, xy, yy} 的窗口和，光栅序。
// 数值为精确整数（< 2^24），f32 与整数位模式一一对应。
//------------------------------------------------------------------------------
void compute_tensor_window_sums(const FloatMap& Ix, const FloatMap& Iy, int win_size,
                                std::vector<float>& sums) {
    int r = win_size / 2;
    sums.assign((size_t)Ix.w * Ix.h * 3, 0.0f);
    for (int y = 0; y < Ix.h; ++y) {
        for (int x = 0; x < Ix.w; ++x) {
            float sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    float ix = safe_get_(Ix, x + dx, y + dy, 0.0f);
                    float iy = safe_get_(Iy, x + dx, y + dy, 0.0f);
                    auto t = kernels::outer_product(ix, iy);
                    sxx += t.xx;
                    sxy += t.xy;
                    syy += t.yy;
                }
            }
            size_t i = ((size_t)y * Ix.w + x) * 3;
            sums[i + 0] = sxx;
            sums[i + 1] = sxy;
            sums[i + 2] = syy;
        }
    }
}

//------------------------------------------------------------------------------
// 复刻 shi_tomasi.cpp 的 rmax/thr/NMS（仅用于导出中间量，官方实现保留权威）
//------------------------------------------------------------------------------
void replicate_threshold_nms(const FloatMap& resp, float thr_ratio, int win_size,
                             float& rmax_out, float& thr_out,
                             std::vector<Point2f>& corners) {
    corners.clear();
    rmax_out = -1.0f;
    thr_out  = 0.0f;
    if (!resp.data)
        return;

    // 与 shi_tomasi_detect 完全一致的帧屏障阈值
    float rmax = *std::max_element(resp.data, resp.data + resp.w * resp.h);
    rmax_out = rmax;
    if (!std::isfinite(rmax) || rmax <= 0)
        return;
    float thr = rmax * std::clamp(thr_ratio, 0.001f, 1.0f);
    thr_out = thr;

    int r   = win_size / 2;
    int nms = 3;
    int nr  = nms / 2;

    for (int y = nr + r; y < resp.h - nr - r; ++y) {
        for (int x = nr + r; x < resp.w - nr - r; ++x) {
            float v = resp.get(x, y);
            if (v <= 0 || v < thr)
                continue;
            bool is_max = true;
            for (int dy = -nr; dy <= nr && is_max; ++dy)
                for (int dx = -nr; dx <= nr && is_max; ++dx)
                    if (dx != 0 || dy != 0)
                        if (resp.get(x + dx, y + dy) > v)
                            is_max = false;
            if (is_max)
                corners.push_back(Point2f((float)x, (float)y));
        }
    }
}

//------------------------------------------------------------------------------
// 单张图的导出
//------------------------------------------------------------------------------
bool export_one(const std::vector<uint8_t>& bgr, int w, int h,
                const std::string& name, const std::string& out_dir,
                int& exported_candidates) {
    if ((int)bgr.size() != w * h * 3) {
        std::printf("[export][ERROR] %s: bgr size %zu != %d\n",
                    name.c_str(), bgr.size(), w * h * 3);
        return false;
    }

    // 1) 灰度（与 kernels::bgr_to_gray 相同的整数路径）
    GrayImage gray(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            size_t i = ((size_t)y * w + x) * 3;
            gray.set(x, y, kernels::bgr_to_gray(bgr[i], bgr[i + 1], bgr[i + 2]));
        }

    // 2) 梯度与响应（官方实现）
    FloatMap Ix, Iy, resp;
    sobel_xy(gray, Ix, Iy);
    shi_tomasi_response(Ix, Iy, resp, kWinSize);

    // 3) rmax / thr / NMS：复刻导出 + 官方结果对账
    float rmax, thr;
    std::vector<Point2f> cand_mine;
    replicate_threshold_nms(resp, kThresholdRatio, kWinSize, rmax, thr, cand_mine);

    std::vector<Point2f> cand_ref;
    shi_tomasi_detect(gray, cand_ref, kThresholdRatio, kWinSize);

    if (cand_mine.size() != cand_ref.size()) {
        std::printf("[export][ERROR] %s: NMS replicate %zu != reference %zu candidates\n",
                    name.c_str(), cand_mine.size(), cand_ref.size());
        return false;
    }
    for (size_t i = 0; i < cand_mine.size(); ++i) {
        if (cand_mine[i].x != cand_ref[i].x || cand_mine[i].y != cand_ref[i].y) {
            std::printf("[export][ERROR] %s: candidate %zu mismatch (%.1f,%.1f) vs (%.1f,%.1f)\n",
                        name.c_str(), i, cand_mine[i].x, cand_mine[i].y,
                        cand_ref[i].x, cand_ref[i].y);
            return false;
        }
    }

    // 4) 写文件
    std::vector<uint8_t> gray_bytes((size_t)w * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            gray_bytes[(size_t)y * w + x] = gray.get(x, y);

    std::vector<float> ix_f((size_t)w * h), iy_f((size_t)w * h), resp_f((size_t)w * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            size_t i = (size_t)y * w + x;
            ix_f[i]   = Ix.get(x, y);
            iy_f[i]   = Iy.get(x, y);
            resp_f[i] = resp.get(x, y);
        }

    // v2：张量窗口和（ZERO 边界，供前端流水分段对拍）
    std::vector<float> sum_f;
    compute_tensor_window_sums(Ix, Iy, kWinSize, sum_f);

    if (!write_all(out_dir + "/" + name + "_bgr.bin", bgr.data(), bgr.size()) ||
        !write_vec(out_dir + "/" + name + "_gray.bin", gray_bytes) ||
        !write_vec(out_dir + "/" + name + "_ix.bin", ix_f) ||
        !write_vec(out_dir + "/" + name + "_iy.bin", iy_f) ||
        !write_vec(out_dir + "/" + name + "_sum.bin", sum_f) ||
        !write_vec(out_dir + "/" + name + "_resp.bin", resp_f))
        return false;

    std::vector<uint32_t> cand_words;
    cand_words.push_back((uint32_t)cand_mine.size());
    for (const auto& p : cand_mine) {
        cand_words.push_back((uint32_t)(int32_t)p.x);
        cand_words.push_back((uint32_t)(int32_t)p.y);
    }
    if (!write_vec(out_dir + "/" + name + "_candidates.bin", cand_words))
        return false;

    // 5) manifest（含 f32 位模式，RTL 比对浮点建议用位级比较）
    char buf[512];
    std::string man_path = out_dir + "/" + name + "_manifest.txt";
    std::remove(man_path.c_str());
    std::snprintf(buf, sizeof(buf), "format_version=3\n");
    append_manifest(man_path, buf);
    std::snprintf(buf, sizeof(buf), "width=%d\nheight=%d\n", w, h);
    append_manifest(man_path, buf);
    std::snprintf(buf, sizeof(buf),
                  "threshold_ratio=%.6f(0x%08x)\nwin_size=%d\n",
                  kThresholdRatio, f32_bits(kThresholdRatio), kWinSize);
    append_manifest(man_path, buf);
    std::snprintf(buf, sizeof(buf),
                  "rmax=%.9g(0x%08x)\nthr=%.9g(0x%08x)\n",
                  rmax, f32_bits(rmax), thr, f32_bits(thr));
    append_manifest(man_path, buf);
    std::snprintf(buf, sizeof(buf), "candidate_count=%zu\n", cand_mine.size());
    append_manifest(man_path, buf);
    std::snprintf(buf, sizeof(buf),
                  "files=%s_bgr.bin(%zuB) %s_gray.bin(%zuB) "
                  "%s_ix.bin(%zuB) %s_iy.bin(%zuB) %s_sum.bin(%zuB) "
                  "%s_resp.bin(%zuB) %s_candidates.bin(%zuB)\n",
                  name.c_str(), bgr.size(),
                  name.c_str(), gray_bytes.size(),
                  name.c_str(), ix_f.size() * 4,
                  name.c_str(), iy_f.size() * 4,
                  name.c_str(), sum_f.size() * 4,
                  name.c_str(), resp_f.size() * 4,
                  name.c_str(), cand_words.size() * 4);
    append_manifest(man_path, buf);

    exported_candidates += (int)cand_mine.size();
    std::printf("[export] %s: %dx%d rmax=%.6g thr=%.6g candidates=%zu (self-check OK)\n",
                name.c_str(), w, h, rmax, thr, cand_mine.size());
    return true;
}

} // namespace

//------------------------------------------------------------------------------
// v3：min_eigen 单元对拍向量（test0_eigen.bin）
// 内容：u32 N，随后 N×{u32 a_bits, b_bits, c_bits, resp_bits}（小端）
//   1) test0 前 3000 像素的真实张量和 (A,B,C)
//   2) 1000 组随机整数三元组（f32 精确域，种子固定可复现）
//   3) 定向边界用例（0/1 值、det<0、det=0、舍入边界 2^24、完全平方等）
// 期望值一律调用官方 kernels::min_eigenvalue（权威实现）。
//------------------------------------------------------------------------------
bool export_eigen_vectors(const std::string& raw_dir, const std::string& out_dir) {
    std::vector<uint8_t> bgr, dim_bytes;
    if (!read_file(raw_dir + "/test0.bgr", bgr) ||
        !read_file(raw_dir + "/test0.dim", dim_bytes))
        return false;
    int w = 0, h = 0;
    if (std::sscanf((const char*)dim_bytes.data(), "%d %d", &w, &h) != 2)
        return false;

    GrayImage gray(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            size_t i = ((size_t)y * w + x) * 3;
            gray.set(x, y, kernels::bgr_to_gray(bgr[i], bgr[i + 1], bgr[i + 2]));
        }
    FloatMap Ix, Iy;
    sobel_xy(gray, Ix, Iy);
    std::vector<float> sums;
    compute_tensor_window_sums(Ix, Iy, kWinSize, sums);

    struct Quad { uint32_t a, b, c, r; };
    std::vector<Quad> quads;

    int n_real = (int)std::min<size_t>(3000, sums.size() / 3);
    for (int i = 0; i < n_real; ++i) {
        float A = sums[(size_t)i * 3 + 0];
        float B = sums[(size_t)i * 3 + 1];
        float C = sums[(size_t)i * 3 + 2];
        quads.push_back({f32_bits(A), f32_bits(B), f32_bits(C),
                         f32_bits(kernels::min_eigenvalue(A, B, C))});
    }

    std::mt19937 rng(20260924u);
    for (int i = 0; i < 1000; ++i) {
        uint32_t a = rng() % (1u << 24);
        uint32_t c = rng() % (1u << 24);
        int32_t  b = (int32_t)(rng() % (1u << 23)) - (1 << 22);
        float fa = (float)a, fb = (float)b, fc = (float)c;
        quads.push_back({f32_bits(fa), f32_bits(fb), f32_bits(fc),
                         f32_bits(kernels::min_eigenvalue(fa, fb, fc))});
    }

    struct Tri { float a, b, c; };
    const Tri directed[] = {
        {0,0,0}, {1,0,0}, {0,0,1}, {0,5,0}, {1,0,2}, {3,0,10},
        {1,1,1}, {2,2,2}, {3,3,3}, {4,0,4}, {9,0,9}, {16,0,25}, {25,0,144},
        {100,50,100}, {100,-50,100}, {1000,999,1000}, {1,10,1}, {100,1000,100},
        {16777215,0,16777215}, {16777215,0,16777214}, {16777216,0,16777216},
        {16777215,0,1}, {1,0,16777215}, {16777215,16777215,16777215},
        {9363600,0,9363600}, {9363600,1040400,9363600}, {8789066,1234567,8781694},
        {65535,65535,65535}, {65536,65536,65536}, {1040400,0,1040400},
        {1040400,1040400,1040400}, {9363599,1,9363599}, {2,1,2}, {8,4,8},
    };
    for (const auto& t : directed)
        quads.push_back({f32_bits(t.a), f32_bits(t.b), f32_bits(t.c),
                         f32_bits(kernels::min_eigenvalue(t.a, t.b, t.c))});

    std::vector<uint32_t> words;
    words.push_back((uint32_t)quads.size());
    for (const auto& q : quads) {
        words.push_back(q.a); words.push_back(q.b);
        words.push_back(q.c); words.push_back(q.r);
    }
    if (!write_vec(out_dir + "/test0_eigen.bin", words))
        return false;

    char buf[256];
    std::snprintf(buf, sizeof(buf), "eigen_count=%zu\neigen_seed=20260924\n", quads.size());
    append_manifest(out_dir + "/test0_manifest.txt", buf);
    std::printf("[export] test0_eigen.bin: %zu cases (real=%d random=1000 directed=%zu)\n",
                quads.size(), n_real, sizeof(directed) / sizeof(directed[0]));
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: export_vectors.exe <raw_dir> <out_dir>\n");
        return 2;
    }
    std::string raw_dir = argv[1];
    std::string out_dir = argv[2];

    int exported_candidates = 0;
    for (int i = 0; i < 3; ++i) {
        std::string name = "test" + std::to_string(i);

        std::vector<uint8_t> bgr;
        if (!read_file(raw_dir + "/" + name + ".bgr", bgr)) {
            std::printf("[export][ERROR] cannot read %s/%s.bgr (run jpg_to_bgr.ps1 first)\n",
                        raw_dir.c_str(), name.c_str());
            return 2;
        }
        std::vector<uint8_t> dim_bytes;
        if (!read_file(raw_dir + "/" + name + ".dim", dim_bytes)) {
            std::printf("[export][ERROR] cannot read %s/%s.dim\n", raw_dir.c_str(), name.c_str());
            return 2;
        }
        int w = 0, h = 0;
        if (std::sscanf((const char*)dim_bytes.data(), "%d %d", &w, &h) != 2 || w <= 0 || h <= 0) {
            std::printf("[export][ERROR] bad dim file for %s\n", name.c_str());
            return 2;
        }

        if (!export_one(bgr, w, h, name, out_dir, exported_candidates))
            return 1;
    }

    if (!export_eigen_vectors(raw_dir, out_dir))
        return 1;

    std::printf("[export] DONE: 3 images, %d candidates total, out_dir=%s\n",
                exported_candidates, out_dir.c_str());
    return 0;
}
