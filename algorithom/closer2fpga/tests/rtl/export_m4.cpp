// export_m4.cpp — M4 网格排序（organize_grid）对拍向量导出
//------------------------------------------------------------------------------
// 场景：board5x8 — 5×8 角点网格合成棋盘（背景黑，保证 inner ≥ 40 且
//   organize_grid 成功）。全链：gray → shi_tomasi → merge5 → merge3 →
//   nearest/ring → inner → organize_grid（确定性排序变体）→ 40 点。
//
// 确定性排序规则（与 RTL index_sort 对齐，C++ 参考在此有明示差异，
//   见 VERILOG_DESIGN_PLAN §5.4：std::sort 不约定相等 key 次序）：
//   cmp(a,b) = (key_a < key_b) || (key_a == key_b && idx_a < idx_b)
//   该全序使排序结果唯一且稳定，任何稳定排序均与此一致。
//
// 导出（小端，坐标/浮点均为 fp32 位模式，标量 u32）：
//   m4_board5x8_gray.bin   W*H 字节灰度（TB 供 M3 链路复用可省略）
//   m4_board5x8_inner.bin  u32 N + N×{x,y}          （organize_grid 输入）
//   m4_board5x8_grid.bin   u32 ok + u32 N + N×{x,y} （最终行列序角点）
//   m4_sort_<case>.bin     u32 n + n×key + n×idx     （index_sort 单元）
//   m4_cost_<case>.bin     u32 n + n×{x,y} + u32 cost_bits（grid_validate 单元）
//------------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <random>

#include "../../closer2fpga/algo/shi_tomasi.h"
#include "../../closer2fpga/kernels/color.h"
#include "../../closer2fpga/algo/chessboard/internal.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

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

// --- M3 链路的逐行复刻（与 export_m3.cpp 相同）---
float dist_(float ax, float ay, float bx, float by) { return std::hypot(ax - bx, ay - by); }
void merge_dup_(std::vector<Point2f>& pts, float radius) {
    std::vector<bool> used(pts.size(), false);
    std::vector<Point2f> out;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (used[i]) continue;
        Point2f sum = pts[i]; int n = 1;
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
bool ring_detail_(const GrayImage& img, Point2f p, float radius) {
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

// --- 确定性排序变体 organize_grid（位级对拍权威）---
// 与 ordering.cpp::organize_grid 逐行一致，仅排序比较器加 (key 相等 → 原索引升序)
// 全序 tie-break；median 用全排序取 [size/2]（nth_element 对 7 元素值相同）。
// 与 validation.cpp::grid_cost 一致，仅 log 用自定义 log_ref（见下）。
float median_sorted(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// 自定义 fp32 log（对拍权威，RTL fp32_log 复刻同序）：
//   log(x) = 2·atanh((x-1)/(x+1)) = 2·z·(1 + z²/3 + z⁴/5 + …)（Horner 嵌套序）
//   值域 |z| ≤ 0.29（x∈[0.55,1.8]），8 项截断误差 ≪ fp32 ulp。
//   与 libm logf 有 1ulp 级差异（实测 logf≠(float)log(double)），因此以本函数
//   为 RTL 位级权威，不声称与 libm 位级一致（见 M4_REPORT 记录）。
float log_ref(float x) {
    const float c13 = 1.0f / 3.0f, c15 = 1.0f / 5.0f, c17 = 1.0f / 7.0f;
    const float c19 = 1.0f / 9.0f, c111 = 1.0f / 11.0f, c113 = 1.0f / 13.0f;
    const float c115 = 1.0f / 15.0f;
    float z = (x - 1.0f) / (x + 1.0f);        // fp32 同序
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
        std::sort(gaps.begin(), gaps.end());   // 位置升序（互异，无歧义）
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

void emit_pts(const std::string& path, const std::vector<Point2f>& pts) {
    std::vector<uint32_t> w;
    w.push_back((uint32_t)pts.size());
    for (auto& p : pts) { w.push_back(fb(p.x)); w.push_back(fb(p.y)); }
    write_vec(path, w);
}

// --- 场景：board5x8 ---
// 全图棋盘 6×17 格（格 16px）→ 内部格交点 = 5 行 × 16 列 = 80 个角点，
//   全部四象限（上下/左右仍有格，图像边缘角点 ring 越界被淘汰）。
//   organize_grid 目标 5×8：行分割取 4 个行间隙（5 行 → 选全部，无歧义），
//   每行 16 点窗口取连续 8 列 → 40 点。
void export_scene(const std::string& scene, int rows, int cols, int cell,
                  int rows_big, int cols_big,
                  const std::string& out_dir) {
    int W = cols_big * cell;
    int H = rows_big * cell;
    GrayImage gray(W, H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            int g = ((x / cell) + (y / cell)) & 1 ? 180 : 40;
            gray.set(x, y, g);
        }
    std::vector<uint8_t> grayb((size_t)W * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) grayb[(size_t)y * W + x] = gray.get(x, y);
    write_all(out_dir + "/m4_" + scene + "_gray.bin", grayb.data(), grayb.size());

    // M3 链路（无 subpixel 变体）
    std::vector<Point2f> cand;
    shi_tomasi_detect(gray, cand, 0.08f, 3);
    std::printf("[%s] shi_tomasi: %zu\n", scene.c_str(), cand.size());
    merge_dup_(cand, 5.0f);
    merge_dup_(cand, 3.0f);
    std::vector<Point2f> inner;
    for (size_t i = 0; i < cand.size(); ++i) {
        float spacing = nearest_(cand, i);
        float radius = std::clamp(spacing * 0.22f, 4.0f, 18.0f);
        bool ok0 = ring_detail_(gray, cand[i], radius);
        bool okA = ok0 && ring_detail_(gray, cand[i], radius * 0.75f);
        bool okB = ok0 && !okA && ring_detail_(gray, cand[i], radius * 1.25f);
        if (ok0 && (okA || okB))
            inner.push_back(cand[i]);
    }
    std::printf("[%s] after merge/ring: %zu\n", scene.c_str(), inner.size());
    emit_pts(out_dir + "/m4_" + scene + "_inner.bin", inner);

    // organize_grid（确定性变体）
    std::vector<Point2f> best;
    bool ok = (int)inner.size() >= rows * cols && organize_grid_det(inner, rows, cols, best);
    std::printf("[%s] organize_grid: %s, pts=%zu\n", scene.c_str(), ok ? "OK" : "FAIL",
                best.size());
    std::vector<uint32_t> gw;
    gw.push_back(ok ? 1u : 0u);
    gw.push_back((uint32_t)best.size());
    for (auto& p : best) { gw.push_back(fb(p.x)); gw.push_back(fb(p.y)); }
    write_vec(out_dir + "/m4_" + scene + "_grid.bin", gw);
}

// --- fp32_log 单元向量（log_ref 位级）---
void export_log_cases(const std::string& out_dir) {
    std::vector<uint32_t> w;
    w.push_back(0);   // 计数占位
    // 值域 [0.55, 1.8] 密集采样 + 边界 + 特殊值
    int cnt = 0;
    auto push1 = [&](float x) {
        w.push_back(fb(x));
        w.push_back(fb(log_ref(x)));
        ++cnt;
    };
    for (int i = 0; i < 4000; ++i) {
        float x = 0.55f + (1.8f - 0.55f) * i / 3999.0f;
        push1(x);
    }
    push1(1.0f); push1(0.55f); push1(1.8f); push1(0.99999994f); push1(1.0000001f);
    push1(2.0f); push1(0.5f); push1(4.0f); push1(0.1f); push1(10.0f);
    w[0] = (uint32_t)cnt;
    write_vec(out_dir + "/m4_log.bin", w);
    std::printf("[log] %d cases\n", cnt);
}

// --- fp32 位模式 → 单调无符号 key（f2o，符号感知的数值序）---
//   RTL index_sort 用无符号比较；organize_grid 的 u/v 投影可为负，
//   调用侧把 key 做 f2o 映射后无符号比较 = 有符号 float 数值序（位级单调）。
uint32_t f2o(uint32_t v) { return (v & 0x80000000u) ? ~v : (v | 0x80000000u); }

// --- index_sort 单元向量 ---
void export_sort_cases(const std::string& out_dir) {
    std::mt19937 rng(20260925u);
    std::vector<int> sizes = {8, 16, 40};
    for (int n : sizes) {
        // 3 组：纯随机 / 含少量相等 key / 全相等（退化）
        for (int cs = 0; cs < 3; ++cs) {
            std::vector<float> key(n);
            for (int i = 0; i < n; ++i) {
                if (cs == 1 && i > 0 && i % 3 == 0)
                    key[i] = key[i - 1];                    // 相等 key
                else if (cs == 2)
                    key[i] = 1.0f;                           // 全相等
                else
                    key[i] = (rng() % 100000) / 1000.0f;     // 0..99.999 正 float
            }
            // 排序比较：float 有符号数值序（含相等 tie-break 原索引升序）
            std::vector<int> idx(n);
            std::iota(idx.begin(), idx.end(), 0);
            std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
                return (key[a] < key[b]) || (key[a] == key[b] && a < b);
            });
            std::vector<uint32_t> w;
            w.push_back((uint32_t)n);
            for (auto k : key) w.push_back(f2o(fb(k)));   // 存 f2o 后位模式（与 RTL 无符号比较对齐）
            for (auto i : idx) w.push_back((uint32_t)i);
            char name[64];
            std::snprintf(name, sizeof(name), "/m4_sort_%d_%d.bin", n, cs);
            write_vec(out_dir + name, w);
        }
    }
}

// --- grid_validate 单元向量 ---
void export_cost_cases(const std::string& out_dir) {
    std::mt19937 rng(42u);
    for (int cs = 0; cs < 6; ++cs) {
        int rows = 5, cols = 8, n = rows * cols;
        std::vector<Point2f> pts(n);
        if (cs == 0) {
            // 理想矩形网格（合法）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {c * 18.0f + 10, r * 18.0f + 10};
        } else if (cs == 1) {
            // 轻微扰动（合法）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {c * 18.0f + 10 + (rng() % 40) / 100.0f,
                                         r * 18.0f + 10 + (rng() % 40) / 100.0f};
        } else if (cs == 2) {
            // 边长过短（非法：l1<4）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {c * 2.0f + 10, r * 18.0f + 10};
        } else if (cs == 3) {
            // 共线（非法：cross 符号/长度）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {(c * 18.0f + 10) * (r + 1), r * 18.0f + 10};
        } else if (cs == 4) {
            // 折线行（非法：cosine<0.90）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {c * 18.0f + 10 + (r % 2 ? 8.0f : 0.0f),
                                         r * 18.0f + 10};
        } else {
            // 随机点（大概率非法）
            for (int r = 0; r < rows; ++r)
                for (int c = 0; c < cols; ++c)
                    pts[r * cols + c] = {10 + (rng() % 800) / 10.0f, 10 + (rng() % 800) / 10.0f};
        }
        float cost = cost_ref(pts, rows, cols);
        std::vector<uint32_t> w;
        w.push_back((uint32_t)n);
        for (auto& p : pts) { w.push_back(fb(p.x)); w.push_back(fb(p.y)); }
        w.push_back(fb(cost));
        char name[64];
        std::snprintf(name, sizeof(name), "/m4_cost_%d.bin", cs);
        write_vec(out_dir + name, w);
        std::printf("[cost %d] %08x (%.4f)\n", cs, fb(cost), cost);
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string out_dir = argc >= 2 ? argv[1] : ".";
    // 全图 6×17 格 → 内部角点 5×16=80；organize_grid 选 5×8
    export_scene("board5x8", 5, 8, 16, 6, 17, out_dir);
    export_sort_cases(out_dir);
    export_cost_cases(out_dir);
    export_log_cases(out_dir);
    std::printf("[export_m4] DONE\n");
    return 0;
}
