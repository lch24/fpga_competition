// export_m6.cpp — M6 基础件（downsample2x / gray_scan）对拍向量导出
//------------------------------------------------------------------------------
// 用途：M6 阶段两个扫描/缩放模块的 RTL 对拍依据，位级权威。
//   复刻逻辑逐行取自：
//     - downsample：chessboard.cpp::detect_chessboard 递归分支的缩图公式
//         half(x,y) = (a+b+c+d+2)/4，a..d = 源(2x,2y)/(2x+1,2y)/(2x,2y+1)/(2x+1,2y+1)
//         输出 floor(W/2)×floor(H/2)，奇数末行/末列不参与
//     - gray：kernels/color.h::bgr_to_gray（整数截断）
//         gray = (299*R + 587*G + 114*B + 500) / 1000
//         输入字节序 B,G,R；C=1 时直接复制
//
// 文件格式（小端；标量 u32，像素 u8）：
//   m6_down_<scene>.bin  u32 W, u32 H, W*H 字节源灰度, (W/2)*(H/2) 字节期望缩图
//   m6_gray_<scene>.bin  u32 W, u32 H, u32 C(=3/1), W*H*C 字节输入, W*H 字节期望灰度
//   m6_big_gray.bin      u8[1280*720]（big 场景原始灰度，M6.2 全链复用）
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <string>
#include <random>
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>

#include "../../closer2fpga/common/image.h"
#include "../../closer2fpga/algo/shi_tomasi.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

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

// --- downsample（chessboard.cpp 缩图公式，逐行复刻）---
uint8_t down4(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return uint8_t((int(a) + b + c + d + 2) / 4);
}

// --- bgr_to_gray（kernels/color.h，整数截断）---
uint8_t gray_ref(uint8_t b, uint8_t g, uint8_t r) {
    return uint8_t((299 * r + 587 * g + 114 * b + 500) / 1000);
}

// 合成棋盘（与 export_m5 board5x8 同构；cell=格宽像素，hi/lo 两灰）
void fill_checker(GrayImage& g, int cell, uint8_t hi, uint8_t lo) {
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x)
            g.set(x, y, (((x / cell) + (y / cell)) & 1) ? hi : lo);
}

// 确定性伪随机灰度（覆盖任意尺寸/非棋盘内容）
void fill_rand(GrayImage& g, unsigned seed) {
    std::mt19937 rng(seed);
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x)
            g.set(x, y, (uint8_t)(rng() & 0xff));
}

// big 场景：1280×720 平坦背景 + 居中棋盘（17×6 格，格 24px，内角点 16×5=80）
// 设计要点（M6.2 金字塔可行性验证后确定）：
//   - 全幅 16px 棋盘在 1280×720 会出 13904 个 shi_tomasi 候选（>12000 上限，native 失败）
//   - 格 16px 时半分辨率（8px 格）ring 半径=min(4)=半格，采样落在格边界被平均，
//     4 象限交替被破坏 → inner<40；格 24px 后半分辨率 12px 格，半径 4 深入格内 2px，ring 通过
//   - organize_grid 只保留 rows-1 个最大行间隙切 rows 组，要求点集天然恰好 rows 行：
//     10×7 格 → 内角点 6 行，5 组切分把两行合并 → 同 x 交错 → 网格垃圾 cost=1e30。
//     17×6 格 → 内角点 16×5=80（5 自然行，同 board5x8）→ 切分干净
//   - 本场景：背景平坦（零梯度→无候选），半分辨率（640×360，12px 格）native 可成功
//     → 金字塔路径被真实走通（2p+0.5 + refine_grid@全分辨率）
void fill_big(GrayImage& g) {
    const int bx = 436, by = 288, cell = 24, bc = 17, br = 6;
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) {
            if (x >= bx && x < bx + bc * cell && y >= by && y < by + br * cell) {
                int cx = (x - bx) / cell, cy = (y - by) / cell;
                g.set(x, y, ((cx + cy) & 1) ? 180 : 40);
            } else {
                g.set(x, y, 110);
            }
        }
}

// --- downsample 场景导出 ---
void export_down(const std::string& out_dir, const std::string& name,
                 const GrayImage& in) {
    const int hw = in.w / 2, hh = in.h / 2;
    GrayImage half(hw, hh);
    for (int y = 0; y < hh; ++y)
        for (int x = 0; x < hw; ++x)
            half.set(x, y, down4(in.get(2 * x, 2 * y), in.get(2 * x + 1, 2 * y),
                                 in.get(2 * x, 2 * y + 1), in.get(2 * x + 1, 2 * y + 1)));
    FILE* fp = std::fopen((out_dir + "/m6_down_" + name + ".bin").c_str(), "wb");
    if (!fp) { std::fprintf(stderr, "[down:%s] open fail\n", name.c_str()); return; }
    uint32_t W = (uint32_t)in.w, H = (uint32_t)in.h;
    std::fwrite(&W, 4, 1, fp);
    std::fwrite(&H, 4, 1, fp);
    for (int y = 0; y < in.h; ++y)
        for (int x = 0; x < in.w; ++x) { uint8_t v = in.get(x, y); std::fwrite(&v, 1, 1, fp); }
    for (int y = 0; y < hh; ++y)
        for (int x = 0; x < hw; ++x) { uint8_t v = half.get(x, y); std::fwrite(&v, 1, 1, fp); }
    std::fclose(fp);
    std::printf("[down:%s] %dx%d -> %dx%d\n", name.c_str(), in.w, in.h, hw, hh);
}

// --- gray 场景导出：合成灰（R=G=B，验证字节序/打包）---
void export_gray_synth(const std::string& out_dir, const std::string& name,
                       const GrayImage& g, int c_in) {
    FILE* fp = std::fopen((out_dir + "/m6_gray_" + name + ".bin").c_str(), "wb");
    if (!fp) { std::fprintf(stderr, "[gray:%s] open fail\n", name.c_str()); return; }
    uint32_t W = (uint32_t)g.w, H = (uint32_t)g.h, C = (uint32_t)c_in;
    std::fwrite(&W, 4, 1, fp); std::fwrite(&H, 4, 1, fp); std::fwrite(&C, 4, 1, fp);
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) {
            uint8_t v = g.get(x, y);
            if (c_in == 3) {
                uint8_t b = v, gr = v, r = v;   // R=G=B → gray 恒等于 v
                std::fwrite(&b, 1, 1, fp); std::fwrite(&gr, 1, 1, fp); std::fwrite(&r, 1, 1, fp);
            } else {
                std::fwrite(&v, 1, 1, fp);
            }
        }
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) { uint8_t v = g.get(x, y); std::fwrite(&v, 1, 1, fp); }
    std::fclose(fp);
    std::printf("[gray:%s] %dx%d C=%d\n", name.c_str(), g.w, g.h, c_in);
}

// --- gray 场景导出：随机 BGR（验证 299/587/114 权重与字节序）---
void export_gray_rand(const std::string& out_dir, const std::string& name,
                      int w, int h, unsigned seed) {
    std::mt19937 rng(seed);
    FILE* fp = std::fopen((out_dir + "/m6_gray_" + name + ".bin").c_str(), "wb");
    if (!fp) { std::fprintf(stderr, "[gray:%s] open fail\n", name.c_str()); return; }
    uint32_t W = (uint32_t)w, H = (uint32_t)h, C = 3u;
    std::fwrite(&W, 4, 1, fp); std::fwrite(&H, 4, 1, fp); std::fwrite(&C, 4, 1, fp);
    std::vector<uint8_t> exp((size_t)w * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t b = (uint8_t)(rng() & 0xff), gr = (uint8_t)(rng() & 0xff),
                    r = (uint8_t)(rng() & 0xff);
            std::fwrite(&b, 1, 1, fp); std::fwrite(&gr, 1, 1, fp); std::fwrite(&r, 1, 1, fp);
            exp[(size_t)y * w + x] = gray_ref(b, gr, r);
        }
    std::fwrite(exp.data(), 1, exp.size(), fp);
    std::fclose(fp);
    std::printf("[gray:%s] %dx%d random-BGR\n", name.c_str(), w, h);
}

// ================= M6.2 权威：确定性 detect_chessboard 复刻 =================
// RTL 链（candidate_filter_ctrl/grid_order_ctrl/grid_refine_ctrl）实现的是 M4/M5
// 的确定性变体（organize_grid_det / cost_ref+log_ref / refine_grid_ref），与
// 原库 organize_grid 不同（真实库在 >40 点的稀疏点集上会失败，RTL 对拍以本
// 变体为权威）。递归逻辑与 chessboard.cpp::detect_chessboard 逐行一致：
//   W/H>=32 且 max>960 → 缩半图 → 递归 → 成功则 2p+0.5 + refine_grid_ref
//   （失败转该层 native）→ 否则 native(gray)。

uint32_t fb(float v) { union { float f; uint32_t u; } x; x.f = v; return x.u; }

inline float bilinear_ref(float p00, float p10, float p01, float p11, float dx, float dy) {
    float top = (1 - dx) * p00 + dx * p10;
    float bottom = (1 - dx) * p01 + dx * p11;
    return (1 - dy) * top + dy * bottom;
}
inline double gauss_w(int x, int y, int radius) {
    return std::exp(-double(x * x + y * y) / (double)(radius * radius));
}
inline bool solve_tensor(double a, double b, double c, double bx, double by,
                         double& dx, double& dy) {
    double det = a * c - b * b, trace = a + c;
    if (trace < 1e-8 || det <= 1e-5 * trace * trace)
        return false;
    dx = (c * bx - b * by) / det;
    dy = (a * by - b * bx) / det;
    return true;
}
float sample_ref(const GrayImage& img, float x, float y) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y);
    float dx = x - ix, dy = y - iy;
    return bilinear_ref((float)img.get(ix, iy), (float)img.get(ix + 1, iy),
                        (float)img.get(ix, iy + 1), (float)img.get(ix + 1, iy + 1), dx, dy);
}
struct PointResult { float out_x, out_y; bool reliable; int iters; };
PointResult refine_subpixel_ref(const GrayImage& img, Point2f original, int half_win) {
    const int radius = std::clamp(half_win, 2, 15);
    PointResult r{original.x, original.y, false, 0};
    if (!img.data || img.w < 2 * radius + 5 || img.h < 2 * radius + 5) return r;
    if (!std::isfinite(original.x) || !std::isfinite(original.y)) return r;
    Point2f p = original;
    bool reliable = false;
    for (int iter = 0; iter < 40; ++iter) {
        if (p.x < radius + 1 || p.y < radius + 1 || p.x >= img.w - radius - 2 ||
            p.y >= img.h - radius - 2) { r.iters = iter; break; }
        double a = 0, b = 0, c = 0, bx = 0, by = 0;
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x) {
                float sx = p.x + x, sy = p.y + y;
                double gx = sample_ref(img, sx + 1, sy) - sample_ref(img, sx - 1, sy);
                double gy = sample_ref(img, sx, sy + 1) - sample_ref(img, sx, sy - 1);
                double w = gauss_w(x, y, radius);
                double xx = w * gx * gx;
                double xy = w * gx * gy;
                double yy = w * gy * gy;
                a += xx; b += xy; c += yy;
                bx += xx * x + xy * y;
                by += xy * x + yy * y;
            }
        double dx = 0, dy = 0;
        if (!solve_tensor(a, b, c, bx, by, dx, dy)) { r.iters = iter + 1; break; }
        Point2f next{float(p.x + dx), float(p.y + dy)};
        if (!std::isfinite(next.x) || !std::isfinite(next.y) ||
            std::hypot(next.x - original.x, next.y - original.y) > radius) {
            r.iters = iter + 1; break;
        }
        p = next;
        if (dx * dx + dy * dy < 1e-6) { reliable = true; r.iters = iter + 1; break; }
    }
    if (reliable) { r.out_x = p.x; r.out_y = p.y; r.reliable = true; }
    return r;
}

float dist_ref(float ax, float ay, float bx, float by) {
    return std::hypot(ax - bx, ay - by);
}
void merge_dup_ref(std::vector<Point2f>& pts, float radius) {
    std::vector<bool> used(pts.size(), false);
    std::vector<Point2f> out;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (used[i]) continue;
        Point2f sum = pts[i];
        int n = 1;
        for (size_t j = i + 1; j < pts.size(); ++j)
            if (!used[j] && dist_ref(pts[i].x, pts[i].y, pts[j].x, pts[j].y) < radius) {
                used[j] = true; sum.x += pts[j].x; sum.y += pts[j].y; ++n;
            }
        out.push_back({sum.x / n, sum.y / n});
    }
    pts = std::move(out);
}
float nearest_ref(const std::vector<Point2f>& pts, size_t i) {
    float r = std::numeric_limits<float>::max();
    for (size_t j = 0; j < pts.size(); ++j)
        if (i != j) r = std::min(r, dist_ref(pts[i].x, pts[i].y, pts[j].x, pts[j].y));
    return r;
}
bool ring_ref(const GrayImage& img, Point2f p, float radius) {
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
    if (transitions.size() != 4 || opposite_error > 32 * (hi - lo) * 0.28f) return false;
    for (int k = 0; k < 4; ++k) {
        int length = (transitions[(k + 1) % 4] - transitions[k] + 32) % 32;
        if (length < 3 || length > 13) return false;
    }
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
                    float lengths = dist_ref(a.x, a.y, b.x, b.y) * dist_ref(b.x, b.y, d.x, d.y);
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
                    steps.push_back(dist_ref(points[order[start + c - 1]].x,
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
        PointResult r = refine_subpixel_ref(img, p, half_win);
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
                min_step = std::min(min_step, dist_ref(corners[i].x, corners[i].y,
                                                       corners[i + 1].x, corners[i + 1].y));
            if (r + 1 < rows)
                min_step = std::min(min_step, dist_ref(corners[i].x, corners[i].y,
                                                       corners[i + cols].x, corners[i + cols].y));
        }
    refine_subpixel_batch(gray, corners, std::clamp(int(min_step * 0.15f), 2, 10));
    bool valid = cost_ref(corners, rows, cols) < 1e30f;
    if (!valid) corners.clear();
    return valid;
}

struct DetInfo { bool valid; std::vector<Point2f> corners; };
DetInfo detect_native_det(const GrayImage& gray, int rows, int cols) {
    DetInfo info{false, {}};
    if (!gray.data || gray.w < 16 || gray.h < 16 || rows < 2 || cols < 2 || rows > 100 || cols > 100)
        return info;
    std::vector<Point2f> cand;
    shi_tomasi_detect(gray, cand, 0.08f, 3);
    if (cand.size() > 12000 || cand.size() < (size_t)(rows * cols))
        return info;
    merge_dup_ref(cand, 5.0f);
    refine_subpixel_batch(gray, cand, 7);
    merge_dup_ref(cand, 3.0f);
    std::vector<Point2f> inner;
    for (size_t i = 0; i < cand.size(); ++i) {
        float spacing = nearest_ref(cand, i);
        float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
        if (ring_ref(gray, cand[i], radius) &&
            (ring_ref(gray, cand[i], radius * 0.75f) ||
             ring_ref(gray, cand[i], radius * 1.25f)))
            inner.push_back(cand[i]);
    }
    if (inner.size() < (size_t)(rows * cols))
        return info;
    if (!organize_grid_det(inner, rows, cols, info.corners))
        return info;
    bool ok = refine_grid_ref(gray, info.corners, rows, cols);
    info.valid = ok;
    if (!ok) info.corners.clear();
    return info;
}
DetInfo detect_chessboard_ref(const GrayImage& gray, int rows, int cols) {
    if (gray.data && gray.w >= 32 && gray.h >= 32 && std::max(gray.w, gray.h) > 960) {
        GrayImage half(gray.w / 2, gray.h / 2);
        for (int y = 0; y < half.h; ++y)
            for (int x = 0; x < half.w; ++x)
                half.set(x, y, uint8_t((int(gray.get(2 * x, 2 * y)) + gray.get(2 * x + 1, 2 * y) +
                                        gray.get(2 * x, 2 * y + 1) + gray.get(2 * x + 1, 2 * y + 1) + 2) /
                                       4));
        auto coarse = detect_chessboard_ref(half, rows, cols);
        if (coarse.valid) {
            for (auto& p : coarse.corners) { p.x = 2 * p.x + 0.5f; p.y = 2 * p.y + 0.5f; }
            bool ok = refine_grid_ref(gray, coarse.corners, rows, cols);
            if (ok) return coarse;
        }
    }
    return detect_native_det(gray, rows, cols);
}

// --- M6.2 全链向量：big 场景（金字塔路径）与 board5x8（native 路径） ---
void export_chain(const std::string& out_dir, const std::string& name,
                  const GrayImage& gray, int rows, int cols) {
    // 决策路径诊断
    if (gray.w >= 32 && gray.h >= 32 && std::max(gray.w, gray.h) > 960) {
        GrayImage half(gray.w / 2, gray.h / 2);
        for (int y = 0; y < half.h; ++y)
            for (int x = 0; x < half.w; ++x)
                half.set(x, y, uint8_t((int(gray.get(2 * x, 2 * y)) + gray.get(2 * x + 1, 2 * y) +
                                        gray.get(2 * x, 2 * y + 1) + gray.get(2 * x + 1, 2 * y + 1) + 2) /
                                       4));
        auto nh = detect_native_det(half, rows, cols);
        std::printf("[chain:%s] native@half valid=%d corners=%zu\n", name.c_str(),
                    nh.valid, nh.corners.size());
        // 诊断：half native 各阶段计数
        if (!nh.valid) {
            std::vector<Point2f> cand;
            shi_tomasi_detect(half, cand, 0.08f, 3);
            std::printf("[chain:%s]  half: shi_tomasi=%zu\n", name.c_str(), cand.size());
            merge_dup_ref(cand, 5.0f);
            std::printf("[chain:%s]  half: merge5=%zu\n", name.c_str(), cand.size());
            refine_subpixel_batch(half, cand, 7);
            merge_dup_ref(cand, 3.0f);
            std::printf("[chain:%s]  half: merge3=%zu\n", name.c_str(), cand.size());
            std::vector<Point2f> inner;
            for (size_t i = 0; i < cand.size(); ++i) {
                float spacing = nearest_ref(cand, i);
                float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
                if (ring_ref(half, cand[i], radius) &&
                    (ring_ref(half, cand[i], radius * 0.75f) ||
                     ring_ref(half, cand[i], radius * 1.25f)))
                    inner.push_back(cand[i]);
            }
            std::printf("[chain:%s]  half: inner=%zu\n", name.c_str(), inner.size());
            if (inner.size() >= (size_t)(rows * cols)) {
                std::vector<Point2f> grid;
                bool ok = organize_grid_det(inner, rows, cols, grid);
                std::printf("[chain:%s]  half: organize=%d grid=%zu\n", name.c_str(), ok, grid.size());
                if (ok) {
                    bool rk = refine_grid_ref(half, grid, rows, cols);
                    std::printf("[chain:%s]  half: refine=%d\n", name.c_str(), rk);
                }
            }
        }
    }
    auto res = detect_chessboard_ref(gray, rows, cols);
    std::printf("[chain:%s] detect_chessboard_ref valid=%d corners=%zu\n", name.c_str(),
                res.valid, res.corners.size());
    std::vector<uint32_t> w;
    w.push_back(res.valid ? 1u : 0u);
    w.push_back((uint32_t)res.corners.size());
    for (auto& p : res.corners) { w.push_back(fb(p.x)); w.push_back(fb(p.y)); }
    write_vec(out_dir + "/m6_chain_" + name + ".bin", w);
    if (res.valid)
        std::printf("[chain:%s] first=%g,%g last=%g,%g\n", name.c_str(),
                    res.corners.front().x, res.corners.front().y,
                    res.corners.back().x, res.corners.back().y);
}

// --- M7.3 权威：shi_tomasi 全图响应矩阵（= RTL response_store_max RAM 存的 resp 流）---
// 复刻 shi_tomasi_detect 的 Pass1（shi_tomasi.cpp:69-72 调用的两个公开函数）：
//   sobel_xy（shi_tomasi.cpp:13-38，clamp 边界）→ shi_tomasi_response
//   （shi_tomasi.cpp:40-62，win_size=3：3×3 window 累加 tensor，出界补 0，
//    逐像素 min_eigenvalue，gradient.h:19-23）→ resp 为原始 min_eigen 值。
// 不做任何阈值/NMS（那些在 shi_tomasi_detect Pass2，shi_tomasi.cpp:86-102）。
// RTL 对应：detect_ctrl 最深层槽位（恒 native）shi_tomasi_ctrl 的 Pass1 resp 流
//   以光栅序逐像素写入 response_store_max RAM（response_store_max.sv:78-83）。
// 文件格式（小端）：u32 W, u32 H, W*H 个 fp32（fb 位透传，光栅序 x 递增→y 递增）。
void dump_resp_map(const std::string& out_dir, const std::string& name,
                   const GrayImage& gray) {
    FloatMap Ix, Iy, resp;
    sobel_xy(gray, Ix, Iy);
    shi_tomasi_response(Ix, Iy, resp, 3);
    std::vector<uint32_t> w;
    w.reserve((size_t)resp.w * resp.h + 2);
    w.push_back((uint32_t)resp.w);
    w.push_back((uint32_t)resp.h);
    for (int y = 0; y < resp.h; ++y)
        for (int x = 0; x < resp.w; ++x)
            w.push_back(fb(resp.get(x, y)));
    write_vec(out_dir + "/m7_resp_" + name + ".bin", w);
    std::printf("[resp:%s] %dx%d bytes=%zu\n", name.c_str(), resp.w, resp.h,
                w.size() * sizeof(uint32_t));
}

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = argc >= 2 ? argv[1] : ".";

    // ---- downsample 场景 ----
    // board5x8：272 宽 × 96 高（6×17 格 @16px，与 M5 约定一致，勿转置！）
    GrayImage b5(272, 96);   fill_checker(b5, 16, 180, 40);
    GrayImage big(1280, 720); fill_big(big);
    GrayImage s64(64, 64);   fill_checker(s64, 8, 200, 60);
    GrayImage odd(33, 17);   fill_rand(odd, 101u);
    GrayImage s127(127, 63); fill_rand(s127, 202u);
    GrayImage s5(5, 3);      fill_rand(s5, 303u);
    GrayImage s2(2, 2);      fill_checker(s2, 1, 120, 40);
    GrayImage s3x1(3, 1);    fill_rand(s3x1, 404u);    // H/2=0 → 0 输出
    GrayImage s1x4(1, 4);    fill_rand(s1x4, 505u);    // W/2=0 → 0 输出

    export_down(out_dir, "board5x8", b5);
    export_down(out_dir, "big",      big);
    export_down(out_dir, "s64",      s64);
    export_down(out_dir, "odd33x17", odd);
    export_down(out_dir, "s127x63",  s127);
    export_down(out_dir, "s5x3",     s5);
    export_down(out_dir, "s2x2",     s2);
    export_down(out_dir, "s3x1",     s3x1);
    export_down(out_dir, "s1x4",     s1x4);

    // ---- gray 场景 ----
    export_gray_synth(out_dir, "board5x8", b5, 3);
    export_gray_synth(out_dir, "big",      big, 3);
    export_gray_synth(out_dir, "copy",     b5, 1);     // C=1 直接复制
    export_gray_synth(out_dir, "s2x2",     s2, 3);
    export_gray_rand(out_dir, "rand33x17", 33, 17, 777u);
    export_gray_rand(out_dir, "rand127x63", 127, 63, 888u);
    export_gray_rand(out_dir, "rand1x1",   1, 1, 999u);
    export_gray_rand(out_dir, "rand5x3",   5, 3, 111u);

    // ---- big 场景原始灰度（M6.2 全链复用；detect_chessboard 输入）----
    std::vector<uint8_t> bigb((size_t)big.w * big.h);
    for (int y = 0; y < big.h; ++y)
        for (int x = 0; x < big.w; ++x) bigb[(size_t)y * big.w + x] = big.get(x, y);
    write_all(out_dir + "/m6_big_gray.bin", bigb.data(), bigb.size());
    std::printf("[big_gray] 1280x720 bytes=%zu\n", bigb.size());

    // ---- M6.2 全链向量：big（金字塔路径）+ board5x8（native 路径）----
    export_chain(out_dir, "big", big, 5, 8);
    export_chain(out_dir, "board5x8", b5, 5, 8);

    // ---- M7.3 响应图权威向量（= RTL response_store_max RAM 存的 resp 流）----
    // big：detect_chessboard_ref 走金字塔路径（1280×720 >960 → 缩半），native
    //   检测发生在 L1 半图 640×360 → 先在原图上做 down4 缩半，再在半图上算响应。
    {
        GrayImage half(big.w / 2, big.h / 2);
        for (int y = 0; y < half.h; ++y)
            for (int x = 0; x < half.w; ++x)
                half.set(x, y, uint8_t((int(big.get(2 * x, 2 * y)) + big.get(2 * x + 1, 2 * y) +
                                        big.get(2 * x, 2 * y + 1) + big.get(2 * x + 1, 2 * y + 1) + 2) /
                                       4));
        dump_resp_map(out_dir, "big", half);            // 640×360 → 921608 B
    }
    // board5x8：native 检测在 L0 原图 272×96 → 直接在原图上算响应。
    dump_resp_map(out_dir, "board5x8", b5);             // 272×96 → 104456 B

    std::printf("[export_m6] DONE\n");
    return 0;
}
