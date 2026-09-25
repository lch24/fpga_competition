// export_m5.cpp — M5 亚像素精定位（refine_subpixel）对拍向量导出
//------------------------------------------------------------------------------
// 用途：M5 阶段 subpixel 数值链的 RTL 对拍依据，位级权威。
//   复刻逻辑逐行取自 closer2fpga/algo/subpixel.cpp（sample/bilinear、
//   Gaussian 加权梯度累加、2×2 求解、迭代收敛/失败回滚）——运算顺序与
//   参考完全一致（float 插值+差分 → double 加权累加 → double 求解）。
//
// 精度链（与 RTL 契约，不得改动）：
//   - sample(x,y)：floor 取整，dx/dy 小数部分（float 域），kernels::bilinear
//     （float，top→bottom→lerp 顺序）→ float 像素
//   - gx = sample(sx+1,sy) - sample(sx-1,sy)：float 减法 → 提升 double（精确）
//   - w = exp(-(x²+y²)/r²)：double，只依赖整数偏移+radius → 离线 ROM
//     （radius=2..15 全表，见 gaussian_weights.mem/.bin，RTL 查表不用 exp）
//   - outer_product：{w*gx*gx, w*gx*gy, w*gy*gy}（左结合，double）
//   - 累加：窗口 y 外循环 x 内循环（(-r,-r)→(r,r)）；bx += xx*x + xy*y
//     （先乘后加再累加，double，逐次舍入）
//   - solve_tensor：det=a*c-b*b；trace=a+c；trace<1e-8 || det<=1e-5*trace²
//     → 失败；dx=(c*bx-b*by)/det；dy=(a*by-b*bx)/det（全部 double）
//   - next = {float(p.x+dx), float(p.y+dy)}（double 加法 → f64_to_f32）
//     非有限 || hypot(next-original)>radius（fp32_hypot 权威）→ 失败
//   - 收敛 dx²+dy²<1e-6（double）→ reliable；40 轮上限，未收敛保留 original
//
// 输出（小端；float 用 fp32 位模式，double 用 u64 位模式，标量 u32）：
//   m5_<scene>_gray.bin   W*H 字节灰度
//   m5_<scene>_pts.bin    u32 N + N×{x,y}            （输入点流 original）
//   m5_<scene>_out.bin    u32 N + N×{x,y,reliable}    （期望输出）
//   m5_bilinear.bin       u32 N + N×{p00,p10,p01,p11,dx,dy} + N×out
//   m5_tensor.bin         u32 N + N×{a,b,c,bx,by}（u64）+ N×{dx,dy,ok}
//   m5_acc.bin            u32 N + N×{x,y,w,gx,gy}（u64）+ N×{a,b,c,bx,by}
//   gaussian_weights.mem  $readmemh 行格式（radius=2..15 段，段内 y 外 x 内）
//   gaussian_weights.bin  u64 数组（同布局，供 g++/TB 直读）
//   m5_const.txt          位级常量（1e-5/1e-8/1e-6 u64）
//   m5_<scene>_traj.txt   诊断：每点每迭代 p/dx/dy/收敛标志（fail 定位用）
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <random>
#include <limits>

#include "../../closer2fpga/common/image.h"
#include "../../closer2fpga/algo/shi_tomasi.h"
#include "../../closer2fpga/algo/chessboard/internal.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

uint32_t fb(float v) { union { float f; uint32_t u; } x; x.f = v; return x.u; }
uint64_t fd(double v) { union { double d; uint64_t u; } x; x.d = v; return x.u; }
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

// --- 浮点运算位级助手（打印常量）---
void export_constants(const std::string& out_dir) {
    FILE* fp = std::fopen((out_dir + "/m5_const.txt").c_str(), "w");
    std::fprintf(fp, "ONE_E_MINUS_5  = %016llx\n", (unsigned long long)fd(1e-5));
    std::fprintf(fp, "ONE_E_MINUS_8  = %016llx\n", (unsigned long long)fd(1e-8));
    std::fprintf(fp, "ONE_E_MINUS_6  = %016llx\n", (unsigned long long)fd(1e-6));
    std::fprintf(fp, "ONE_F          = %08x\n", (unsigned)fb(1.0f));
    std::fprintf(fp, "FOUR_F         = %08x\n", (unsigned)fb(4.0f));
    std::fclose(fp);
}

// --- bilinear（kernels/interpolation.h 逐行复刻，float）---
inline float bilinear(float p00, float p10, float p01, float p11, float dx, float dy) {
    float top = (1 - dx) * p00 + dx * p10;
    float bottom = (1 - dx) * p01 + dx * p11;
    return (1 - dy) * top + dy * bottom;
}

// --- 高斯权重（double，与 subpixel.cpp 同序）---
inline double gauss_w(int x, int y, int radius) {
    return std::exp(-double(x * x + y * y) / (double)(radius * radius));
}

// --- 2×2 求解（kernels/gradient.h::solve_tensor 逐行复刻，double）---
inline bool solve_tensor(double a, double b, double c, double bx, double by,
                         double& dx, double& dy) {
    double det = a * c - b * b, trace = a + c;
    if (trace < 1e-8 || det <= 1e-5 * trace * trace)
        return false;
    dx = (c * bx - b * by) / det;
    dy = (a * by - b * bx) / det;
    return true;
}

// --- refine_subpixel 逐行复刻（subpixel.cpp），带轨迹记录 ---
struct TrajStep { float p_x, p_y; double dx, dy; bool conv; bool fail; };
struct PointResult {
    float out_x, out_y;   // 最终坐标（成功收敛或 original）
    bool  reliable;       // 收敛标志
    int   iters;          // 实际求解轮数（0..40；0=立即失败）
    std::vector<TrajStep> traj;
};

float sample_(const GrayImage& img, float x, float y) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y);
    float dx = x - ix, dy = y - iy;
    return bilinear((float)img.get(ix, iy), (float)img.get(ix + 1, iy),
                    (float)img.get(ix, iy + 1), (float)img.get(ix + 1, iy + 1), dx, dy);
}

PointResult refine_subpixel_traj(const GrayImage& img, Point2f original, int half_win) {
    const int radius = std::clamp(half_win, 2, 15);
    PointResult r{original.x, original.y, false, 0, {}};
    if (!img.data || img.w < 2 * radius + 5 || img.h < 2 * radius + 5)
        return r;
    if (!std::isfinite(original.x) || !std::isfinite(original.y))
        return r;
    Point2f p = original;
    bool reliable = false;
    for (int iter = 0; iter < 40; ++iter) {
        if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 2 ||
            p.y >= img.h - radius - 2) {
            r.iters = iter;
            break;
        }
        double a = 0, b = 0, c = 0, bx = 0, by = 0;
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                float sx = p.x + x, sy = p.y + y;
                double gx = sample_(img, sx + 1, sy) - sample_(img, sx - 1, sy);
                double gy = sample_(img, sx, sy + 1) - sample_(img, sx, sy - 1);
                double w = gauss_w(x, y, radius);
                double xx = w * gx * gx;
                double xy = w * gx * gy;
                double yy = w * gy * gy;
                a += xx; b += xy; c += yy;
                bx += xx * x + xy * y;
                by += xy * x + yy * y;
            }
        }
        double dx = 0, dy = 0;
        if (!solve_tensor(a, b, c, bx, by, dx, dy)) {
            r.traj.push_back({p.x, p.y, dx, dy, false, true});
            r.iters = iter + 1;
            break;
        }
        Point2f next{float(p.x + dx), float(p.y + dy)};
        r.traj.push_back({p.x, p.y, dx, dy, false, false});
        if (!std::isfinite(next.x) || !std::isfinite(next.y) ||
            std::hypot(next.x - original.x, next.y - original.y) > radius) {
            r.iters = iter + 1;
            break;
        }
        p = next;
        if (dx * dx + dy * dy < 1e-6) {
            reliable = true;
            r.iters = iter + 1;
            break;
        }
    }
    // 40 轮未收敛：reliable 保持 false（traj 未记录最后一轮 p；诊断足够）
    if (reliable) { r.out_x = p.x; r.out_y = p.y; r.reliable = true; }
    return r;
}

// --- 单元：bilinear 向量 ---
void export_bilinear(const std::string& out_dir) {
    std::mt19937 rng(20260925u);
    std::vector<uint32_t> w;
    w.push_back(0);
    int cnt = 0;
    auto push = [&](float p00, float p10, float p01, float p11, float dx, float dy) {
        w.push_back(fb(p00)); w.push_back(fb(p10)); w.push_back(fb(p01)); w.push_back(fb(p11));
        w.push_back(fb(dx)); w.push_back(fb(dy));
        w.push_back(fb(bilinear(p00, p10, p01, p11, dx, dy)));
        ++cnt;
    };
    // 随机 + 边界（dx/dy ∈ [0,1)）
    for (int i = 0; i < 500; ++i)
        push((float)(rng() % 256), (float)(rng() % 256), (float)(rng() % 256),
             (float)(rng() % 256), (rng() % 100000) / 100000.0f, (rng() % 100000) / 100000.0f);
    for (int d = 0; d < 10; ++d) {  // dx=0/dy=0 边界
        float dx = (d % 5 == 0) ? 0.0f : ((d % 5 == 1) ? 0.5f : ((rng() % 100000) / 100000.0f));
        float dy = (d % 4 == 0) ? 0.0f : ((d % 4 == 1) ? 0.99999f : ((rng() % 100000) / 100000.0f));
        push(0, 255, 255, 0, dx, dy);
    }
    w[0] = (uint32_t)cnt;
    write_vec(out_dir + "/m5_bilinear.bin", w);
    std::printf("[bilinear] %d cases\n", cnt);
}

// --- 单元：solve_tensor 向量 ---
void export_tensor(const std::string& out_dir) {
    std::mt19937 rng(42u);
    std::vector<uint64_t> w;   // 全部 u64（double 位模式 / ok 用 0/1）
    w.push_back(0);
    int cnt = 0;
    auto push = [&](double a, double b, double c, double bx, double by) {
        w.push_back(fd(a)); w.push_back(fd(b)); w.push_back(fd(c));
        w.push_back(fd(bx)); w.push_back(fd(by));
        double dx = 0, dy = 0;
        bool ok = solve_tensor(a, b, c, bx, by, dx, dy);
        w.push_back(fd(dx)); w.push_back(fd(dy)); w.push_back(ok ? 1ull : 0ull);
        ++cnt;
    };
    for (int i = 0; i < 400; ++i) {
        double a = 1 + (rng() % 1000) / 10.0, b = (rng() % 200 - 100) / 100.0;
        double c = 1 + (rng() % 1000) / 10.0;
        double bx = (rng() % 2000 - 1000) / 100.0, by = (rng() % 2000 - 1000) / 100.0;
        push(a, b, c, bx, by);
    }
    // 失败用例：trace 极小 / det ≤ 1e-5*trace² / 退化
    push(1e-10, 0, 1e-10, 0, 0);              // trace < 1e-8
    push(1, 0, 0, 0.1, 0.1);                   // det = 0 ≤ ...
    push(1, 1, 1, 1, 1);                       // det = 0
    push(0, 0, 0, 0, 0);                       // 全零
    push(100, 1, 100, 50, -50);                // 正常
    push(1e-8, 0, 1e-8, 1e-9, 1e-9);           // trace = 2e-8（临界附近）
    push(1e-5, 1e-5, 1e-5, 1e-3, 1e-3);        // det ≤ 1e-5 trace² 边界
    w[0] = (uint64_t)cnt;
    write_vec(out_dir + "/m5_tensor.bin", w);
    std::printf("[tensor] %d cases\n", cnt);
}

// --- 单元：累加器向量（窗口序列 → 五项累加）---
void export_acc(const std::string& out_dir) {
    std::mt19937 rng(7u);
    std::vector<uint64_t> w;
    w.push_back(0);
    int cnt = 0;
    auto push_win = [&](int radius) {
        double a = 0, b = 0, c = 0, bx = 0, by = 0;
        w.push_back(fd((double)radius));   // 首字段：窗口半径（TB 据其推窗口数）
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x) {
                double gx = (rng() % 4000 - 2000) / 1000.0;
                double gy = (rng() % 4000 - 2000) / 1000.0;
                double wgt = gauss_w(x, y, radius);
                w.push_back(fd((double)x)); w.push_back(fd((double)y));
                w.push_back(fd(wgt)); w.push_back(fd(gx)); w.push_back(fd(gy));
                double xx = wgt * gx * gx, xy = wgt * gx * gy, yy = wgt * gy * gy;
                a += xx; b += xy; c += yy;
                bx += xx * x + xy * y;
                by += xy * x + yy * y;
            }
        w.push_back(fd(a)); w.push_back(fd(b)); w.push_back(fd(c));
        w.push_back(fd(bx)); w.push_back(fd(by));
        ++cnt;
    };
    push_win(2); push_win(2); push_win(3); push_win(4);
    push_win(7); push_win(7); push_win(10); push_win(15);
    w[0] = (uint64_t)cnt;
    write_vec(out_dir + "/m5_acc.bin", w);
    std::printf("[acc] %d windows (r=2..15)\n", cnt);
}

// --- 高斯权重 ROM（radius=2..15；段内 y 外 x 内，与累加顺序一致）---
void export_gauss_rom(const std::string& out_dir) {
    std::vector<uint64_t> rom;
    FILE* mem = std::fopen((out_dir + "/gaussian_weights.mem").c_str(), "w");
    for (int r = 2; r <= 15; ++r) {
        for (int y = -r; y <= r; ++y)
            for (int x = -r; x <= r; ++x) {
                uint64_t v = fd(gauss_w(x, y, r));
                rom.push_back(v);
                std::fprintf(mem, "%016llx\n", (unsigned long long)v);
            }
    }
    std::fclose(mem);
    write_vec(out_dir + "/gaussian_weights.bin", rom);
    size_t total = 0;
    for (int r = 2; r <= 15; ++r) total += (size_t)(2 * r + 1) * (2 * r + 1);
    std::printf("[gauss] %zu entries (r=2..15)\n", total);
}

// --- 场景：合成图 + 点集 → 期望输出（含轨迹诊断）---
struct Scene { const char* name; int half_win; };

void export_scene(const std::string& out_dir, const Scene& sc) {
    // 128×128 合成图：棋盘格 + 水平条纹 + 全黑 + 斜坡区（覆盖多种收敛路径）
    const int W = 128, H = 128, r = std::clamp(sc.half_win, 2, 15);
    GrayImage gray(W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            int g;
            if (x < 64 && y < 64)
                g = ((x / 8 + y / 8) & 1) ? 220 : 30;            // 棋盘格
            else if (x >= 64 && y < 64)
                g = ((y / 6) & 1) ? 200 : 60;                    // 水平条纹
            else if (x < 64 && y >= 64)
                g = 10;                                          // 全黑
            else
                g = 20 + (x + y) / 2;                            // 斜坡
            gray.set(x, y, g);
        }
    std::vector<uint8_t> grayb((size_t)W * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) grayb[(size_t)y * W + x] = gray.get(x, y);
    write_all(out_dir + "/m5_" + sc.name + "_gray.bin", grayb.data(), grayb.size());

    // 点集：棋盘角点（收敛）、条纹点（y 收敛 x 病态）、黑区（失败）、
    //       斜坡、边缘越界点、非有限点（跳过）
    std::vector<Point2f> pts = {
        { 8.5f + r,  8.5f + r},  { 16.5f + r,  8.5f + r}, { 8.5f + r, 16.5f + r},
        { 24.3f + r, 32.1f + r}, { 32.9f + r, 24.7f + r}, { 40.5f + r, 16.3f + r},
        { 48.1f + r, 40.6f + r}, { 56.7f + r, 48.9f + r}, { 16.7f + r, 40.1f + r},
        { 96.0f + r, 12.0f + r}, { 100.0f + r, 24.0f + r},{ 80.0f + r, 90.0f + r},
        { 30.0f + r, 100.0f + r}, { 110.0f + r, 30.0f + r},{ 96.0f + r, 96.0f + r},
        { 0.5f, 0.5f},            { 1.0f, 40.0f},           { 40.0f, 1.0f},
        { 127.0f, 127.0f},        { 0.0f / 0.0f, 8.0f},     { 8.0f, 0.0f / 0.0f},
        { 1.0f / 0.0f, 8.0f},     { 8.0f, -1.0f / 0.0f},
    };
    // 文件需要 2r+5 窗口：黑区/斜坡点确保 p.x、p.y 在 [r+1, W-r-2)
    std::vector<uint32_t> pw;
    pw.push_back((uint32_t)pts.size());
    for (auto& p : pts) { pw.push_back(fb(p.x)); pw.push_back(fb(p.y)); }
    write_vec(out_dir + "/m5_" + sc.name + "_pts.bin", pw);

    std::vector<uint32_t> ow;
    ow.push_back((uint32_t)pts.size());
    FILE* traj = std::fopen((out_dir + "/m5_" + sc.name + "_traj.txt").c_str(), "w");
    std::fprintf(traj, "scene=%s half_win=%d img=%dx%d\n", sc.name, sc.half_win, W, H);
    for (size_t i = 0; i < pts.size(); ++i) {
        PointResult res = refine_subpixel_traj(gray, pts[i], sc.half_win);
        ow.push_back(fb(res.out_x)); ow.push_back(fb(res.out_y));
        ow.push_back(res.reliable ? 1u : 0u);
        std::fprintf(traj, "pt%02zu orig=(%g,%g) out=(%g,%g) rel=%d iters=%d\n",
                     i, pts[i].x, pts[i].y, res.out_x, res.out_y, res.reliable, res.iters);
        for (size_t k = 0; k < res.traj.size(); ++k)
            std::fprintf(traj, "  it%02zu p=(%g,%g) dx=%.12g dy=%.12g\n",
                         k, res.traj[k].p_x, res.traj[k].p_y,
                         res.traj[k].dx, res.traj[k].dy);
    }
    std::fclose(traj);
    write_vec(out_dir + "/m5_" + sc.name + "_out.bin", ow);
    std::printf("[%s] half_win=%d pts=%zu\n", sc.name, sc.half_win, pts.size());
}

// --- 全链场景：board5x8（带 subpixel 的完整 detect_native 变体）---
// 与 export_m4.cpp 同构，唯一差异：merge5 与 merge3 之间插入 refine_subpixel(7)，
// organize_grid 用确定性变体，grid 验证用 cost_ref（log_ref），
// organize 后追加 refine_grid（min_step → subpixel → cost_ref 再验证）。
// 导出供 M5.2 全链对拍：tb_filter（candidate_filter_ctrl 含 S_SUBPX）与
// tb_refine（grid_refine_ctrl）。

float dist_(float ax, float ay, float bx, float by) {
    return std::hypot(ax - bx, ay - by);          // fp32_hypot 权威（M3 已确认）
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
                used[j] = true; sum.x += pts[j].x; sum.y += pts[j].y; ++n;
            }
        }
        out.push_back({sum.x / n, sum.y / n});
    }
    pts = std::move(out);
}

float nearest_(const std::vector<Point2f>& pts, size_t i) {
    float r = std::numeric_limits<float>::max();
    for (size_t j = 0; j < pts.size(); ++j)
        if (i != j) r = std::min(r, dist_(pts[i].x, pts[i].y, pts[j].x, pts[j].y));
    return r;
}

struct RingDetail { float hi, lo, thr, opp_err; int ntrans; bool sector_ok, pass; };

bool ring_detail_(const GrayImage& img, Point2f p, float radius, RingDetail& d) {
    d = {};
    if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 1 ||
        p.y >= img.h - radius - 1) return false;
    float values[32], smooth[32];
    for (int k = 0; k < 32; ++k) {
        float a = 2 * pi * k / 32;
        values[k] = float(img.get(int(std::lround(p.x + radius * std::cos(a))),
                                  int(std::lround(p.y + radius * std::sin(a)))));
    }
    float lo = 255, hi = 0;
    for (int k = 0; k < 32; ++k) {
        smooth[k] = (values[(k + 31) % 32] + 2 * values[k] + values[(k + 1) % 32]) / 4;
        lo = std::min(lo, smooth[k]); hi = std::max(hi, smooth[k]);
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
    d.hi = hi; d.lo = lo; d.thr = threshold; d.opp_err = opposite_error;
    d.ntrans = (int)transitions.size();
    if (transitions.size() != 4 || opposite_error > 32 * (hi - lo) * 0.28f) {
        d.sector_ok = false; return false;
    }
    d.sector_ok = true;
    for (int k = 0; k < 4; ++k) {
        int length = (transitions[(k + 1) % 4] - transitions[k] + 32) % 32;
        if (length < 3 || length > 13) return false;
    }
    d.pass = true;
    return true;
}

float median_sorted(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

float log_ref(float x) {
    const float c13 = 1.0f / 3.0f, c15 = 1.0f / 5.0f, c17 = 1.0f / 7.0f;
    const float c19 = 1.0f / 9.0f, c111 = 1.0f / 11.0f, c113 = 1.0f / 13.0f;
    const float c115 = 1.0f / 15.0f;
    float z = (x - 1.0f) / (x + 1.0f);
    float z2 = z * z;
    float p = c115 + z2 * c113;
    p = c111 + z2 * p;
    p = c19 + z2 * p;
    p = c17 + z2 * p;
    p = c15 + z2 * p;
    p = c13 + z2 * p;
    p = 1.0f + z2 * p;
    return 2.0f * (z * p);
}

float cost_ref(const std::vector<Point2f>& g, int rows, int cols) {
    float cost = 0, sign = 0;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            Point2f p = g[r * cols + c];
            for (int axis = 0; axis < 2; ++axis) {
                int step = axis ? cols : 1;
                int pos = axis ? r : c, count = axis ? rows : cols;
                if (pos + 2 >= count) continue;
                Point2f a = g[r * cols + c + step], b = g[r * cols + c + 2 * step];
                float dx1 = a.x - p.x, dy1 = a.y - p.y;
                float dx2 = b.x - a.x, dy2 = b.y - a.y;
                float l1 = std::hypot(dx1, dy1), l2 = std::hypot(dx2, dy2);
                if (l1 < 4 || l2 < 4 || l2 / l1 < 0.55f || l2 / l1 > 1.8f) return 1e30f;
                float cosine = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
                if (cosine < 0.90f) return 1e30f;
                float change = log_ref(l2 / l1);
                cost += (1 - cosine) + change * change;
            }
            if (r + 1 < rows && c + 1 < cols) {
                Point2f q[4] = {p, g[r * cols + c + 1], g[(r + 1) * cols + c + 1], g[(r + 1) * cols + c]};
                for (int k = 0; k < 4; ++k) {
                    Point2f a = q[k], b = q[(k + 1) % 4], d = q[(k + 2) % 4];
                    float cross = (b.x - a.x) * (d.y - b.y) - (b.y - a.y) * (d.x - b.x);
                    float lengths = dist_(a.x, a.y, b.x, b.y) * dist_(b.x, b.y, d.x, d.y);
                    if (lengths < 16 || std::fabs(cross) < lengths * 0.2f) return 1e30f;
                    if (sign == 0) sign = cross;
                    if (cross * sign <= 0) return 1e30f;
                }
            }
        }
    return cost;
}

bool organize_grid_det(const std::vector<Point2f>& points, int rows, int cols,
                       std::vector<Point2f>& best) {
    float best_cost = 1e30f;
    auto u = [&](int i, float co, float si) { return points[i].x * co + points[i].y * si; };
    auto v = [&](int i, float co, float si) { return -points[i].x * si + points[i].y * co; };
    for (int degree = -90; degree < 90; degree += 2) {
        float a = degree * pi / 180, co = std::cos(a), si = std::sin(a);
        std::vector<int> order(points.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int i, int j) {
            float vi = v(i, co, si), vj = v(j, co, si);
            return (vi < vj) || (vi == vj && i < j);
        });
        std::vector<int> gaps(order.size() - 1);
        std::iota(gaps.begin(), gaps.end(), 0);
        std::sort(gaps.begin(), gaps.end(), [&](int i, int j) {
            float gi = v(order[i + 1], co, si) - v(order[i], co, si);
            float gj = v(order[j + 1], co, si) - v(order[j], co, si);
            return (gi > gj) || (gi == gj && i < j);
        });
        gaps.resize(rows - 1);
        std::sort(gaps.begin(), gaps.end());
        gaps.push_back(int(order.size()) - 1);
        int begin = 0;
        std::vector<Point2f> grid;
        for (int r = 0; r < rows; ++r) {
            int end = gaps[r] + 1;
            if (end - begin < cols) break;
            std::sort(order.begin() + begin, order.begin() + end, [&](int i, int j) {
                float ui = u(i, co, si), uj = u(j, co, si);
                return (ui < uj) || (ui == uj && i < j);
            });
            float row_best = 1e30f; int start_best = -1;
            for (int start = begin; start + cols <= end; ++start) {
                std::vector<float> steps;
                for (int c = 1; c < cols; ++c)
                    steps.push_back(dist_(points[order[start + c - 1]].x,
                                          points[order[start + c - 1]].y,
                                          points[order[start + c]].x,
                                          points[order[start + c]].y));
                float spacing = median_sorted(steps), score = 0;
                if (spacing < 4) continue;
                for (float step : steps) {
                    float change = (step - spacing) / spacing;
                    score += change * change;
                }
                if (score < row_best) { row_best = score; start_best = start; }
            }
            if (start_best < 0) break;
            for (int c = 0; c < cols; ++c)
                grid.push_back(points[order[start_best + c]]);
            begin = end;
        }
        if ((int)grid.size() != rows * cols) continue;
        float cost = cost_ref(grid, rows, cols);
        if (cost < best_cost) { best_cost = cost; best = std::move(grid); }
    }
    if (best.empty()) return false;
    int corner_ids[4] = {0, cols - 1, (rows - 1) * cols, rows * cols - 1};
    int origin = 0;
    for (int k = 1; k < 4; ++k)
        if (best[corner_ids[k]].x + best[corner_ids[k]].y <
            best[corner_ids[origin]].x + best[corner_ids[origin]].y)
            origin = k;
    auto copy = best;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            best[r * cols + c] =
                copy[(origin >= 2 ? rows - 1 - r : r) * cols + (origin % 2 ? cols - 1 - c : c)];
    return true;
}

void refine_subpixel_batch(const GrayImage& img, std::vector<Point2f>& pts, int half_win) {
    for (auto& p : pts) {
        PointResult r = refine_subpixel_traj(img, p, half_win);
        p = {r.out_x, r.out_y};
    }
}

bool refine_grid_ref(const GrayImage& gray, std::vector<Point2f>& corners,
                     int rows, int cols) {
    float min_step = std::numeric_limits<float>::max();
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            int i = r * cols + c;
            if (c + 1 < cols)
                min_step = std::min(min_step, dist_(corners[i].x, corners[i].y,
                                                    corners[i + 1].x, corners[i + 1].y));
            if (r + 1 < rows)
                min_step = std::min(min_step, dist_(corners[i].x, corners[i].y,
                                                    corners[i + cols].x, corners[i + cols].y));
        }
    refine_subpixel_batch(gray, corners, std::clamp(int(min_step * 0.15f), 2, 10));
    bool valid = cost_ref(corners, rows, cols) < 1e30f;
    if (!valid) corners.clear();
    return valid;
}

void emit_pts(const std::string& path, const std::vector<Point2f>& pts) {
    std::vector<uint32_t> w;
    w.push_back((uint32_t)pts.size());
    for (auto& p : pts) { w.push_back(fb(p.x)); w.push_back(fb(p.y)); }
    write_vec(path, w);
}

// 全图 6×17 格棋盘（格 16px，96×272）→ 内角点 80 → 完整 detect_native 变体
void export_fullchain(const std::string& out_dir) {
    const int rows = 5, cols = 8, cell = 16, rows_big = 6, cols_big = 17;
    const int W = cols_big * cell, H = rows_big * cell;
    GrayImage gray(W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            int g = ((x / cell) + (y / cell)) & 1 ? 180 : 40;
            gray.set(x, y, g);
        }
    std::vector<uint8_t> grayb((size_t)W * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) grayb[(size_t)y * W + x] = gray.get(x, y);
    write_all(out_dir + "/m5_board5x8_gray.bin", grayb.data(), grayb.size());

    std::vector<Point2f> cand;
    shi_tomasi_detect(gray, cand, 0.08f, 3);
    std::printf("[board5x8] shi_tomasi: %zu\n", cand.size());
    emit_pts(out_dir + "/m5_board5x8_cand.bin", cand);   // 原始候选（merge5 输入；M5.2 tb_filter 用）
    merge_dup_(cand, 5.0f);
    emit_pts(out_dir + "/m5_board5x8_spx_in.bin", cand);
    std::printf("[board5x8] after merge5: %zu\n", cand.size());

    refine_subpixel_batch(gray, cand, 7);
    emit_pts(out_dir + "/m5_board5x8_spx_out.bin", cand);
    std::printf("[board5x8] after subpixel(7): %zu\n", cand.size());

    merge_dup_(cand, 3.0f);
    emit_pts(out_dir + "/m5_board5x8_merge3.bin", cand);
    std::printf("[board5x8] after merge3: %zu\n", cand.size());

    std::vector<Point2f> inner;
    for (size_t i = 0; i < cand.size(); ++i) {
        float spacing = nearest_(cand, i);
        float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
        RingDetail d0, d1, d2;
        bool ok0 = ring_detail_(gray, cand[i], radius, d0);
        bool okA = ring_detail_(gray, cand[i], radius * 0.75f, d1);
        bool okB = ring_detail_(gray, cand[i], radius * 1.25f, d2);
        if (ok0 && (okA || okB)) inner.push_back(cand[i]);
    }
    emit_pts(out_dir + "/m5_board5x8_inner.bin", inner);
    std::printf("[board5x8] inner: %zu\n", inner.size());

    std::vector<Point2f> grid;
    bool gok = (int)inner.size() >= rows * cols && organize_grid_det(inner, rows, cols, grid);
    std::vector<uint32_t> gw;
    gw.push_back(gok ? 1u : 0u);
    gw.push_back((uint32_t)grid.size());
    for (auto& p : grid) { gw.push_back(fb(p.x)); gw.push_back(fb(p.y)); }
    write_vec(out_dir + "/m5_board5x8_grid.bin", gw);
    std::printf("[board5x8] organize_grid: %s pts=%zu\n", gok ? "OK" : "FAIL", grid.size());

    std::vector<Point2f> refined = grid;
    bool rv = gok && refine_grid_ref(gray, refined, rows, cols);
    std::vector<uint32_t> rw;
    rw.push_back(rv ? 1u : 0u);
    rw.push_back((uint32_t)refined.size());
    for (auto& p : refined) { rw.push_back(fb(p.x)); rw.push_back(fb(p.y)); }
    write_vec(out_dir + "/m5_board5x8_refined.bin", rw);
    std::printf("[board5x8] refine_grid: %s pts=%zu\n", rv ? "OK" : "FAIL", refined.size());
}

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = argc >= 2 ? argv[1] : ".";
    export_constants(out_dir);
    export_bilinear(out_dir);
    export_tensor(out_dir);
    export_acc(out_dir);
    export_gauss_rom(out_dir);
    export_scene(out_dir, {"s7", 7});
    export_scene(out_dir, {"s2", 2});
    export_scene(out_dir, {"s15", 15});
    export_fullchain(out_dir);
    std::printf("[export_m5] DONE\n");
    return 0;
}
