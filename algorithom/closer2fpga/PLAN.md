# closer2fpga 实现规划

## 目标

用纯 C++（无 OpenCV）重写 `../modification/main.cpp`，覆盖完整的两阶段流程：
- **CALIB** — FPGA 上采样 + 角点检测 + LM 标定 → 得到畸变参数
- **RUN** — FPGA 上用参数实时 remap 矫正视频

每个模块明确标注：**PL（可编程逻辑，硬化）还是 PS（CPU/软核，跑 C++）**。

---

## 架构：PL + PS 异构分工

```
         适合 PL 硬化                        适合 PS 软核
       ┌────────────┐                      ┌────────────┐
       │ 固定公式    │                      │ 迭代/灵活  │
       │ 无状态      │                      │ 有状态      │
       │ 大数据量    │                      │ 小数据量    │
       │ 实时要求高  │                      │ 实时要求低  │
       └────────────┘                      └────────────┘
       grayscale, threshold,               subpixel,
       integral_projection,                calibrate (LM),
       shi_tomasi, remap_bilinear          build_remap_table
```

**FPGA 内完整闭环，不用外部 PC**：
- CALIB 阶段：摄像头 → PL 灰度化 → PL 角点检测 → PS 亚像素 → PS LM → 参数锁存 → PS 生成 remap 表
- RUN 阶段  ：摄像头 → PL 灰度化 → PL remap → 实时输出（PS 闲置）

---

## 文件结构

```
closer2fpga/closer2fpga/
├── main.cpp                    顶层入口，状态机驱动 CALIB → RUN
│
├── common/
│   ├── types.h                 全局类型（Point2f, CameraParams 等）
│   ├── image.h                 Image<T> 容器，行优先连续存储
│   ├── matrix.h / .cpp         小型矩阵运算（multiply, SVD 伪逆）
│   └── io.h / .cpp             PPM/PGM 文件读写（测试用，纯软件）
│
└── algo/
    ├── grayscale.h/cpp        ← PL 硬化目标
    ├── threshold.h/cpp        ← PL 硬化目标（binarize 部分）
    ├── chessboard_detect.h/cpp ← PL 硬化目标（积分投影 + Shi-Tomasi）
    ├── subpixel.h/cpp         （PS 运行）
    ├── calibrate.h/cpp        （PS 运行，Zhang 初始化 + LM 迭代）
    └── undistort.h/cpp        拆两部分：build_remap_table (PS) + remap_bilinear (PL 硬化目标)
```

---

## 关键模块接口

### `common/types.h`

```cpp
using f32 = float;
using f64 = double;

struct Point2f  { f32 x, y; };
struct Point2i  { int x, y; };
struct Point3f  { f32 x, y, z; };

// CALIB 产物 → 锁存为寄存器 → RUN 输入
struct CameraParams {
    f32 fx, fy;         // 焦距
    f32 cx, cy;         // 主点
    f32 k1, k2, k3;     // 径向畸变
    f32 p1, p2;         // 切向畸变
    bool valid;
};
```

### `common/image.h`

```cpp
template <typename T>
class Image {
public:
    int w, h;
    T* data;              // 行优先、连续、固定大小

    T& at(int x, int y);
    T  get(int x, int y) const;   // 不做边界检查
    void set(int x, int y, T v);
};

using GrayImage = Image<uint8_t>;
using RgbImage  = Image<uint8_t>;
using FloatMap  = Image<f32>;
```

### `common/matrix.h`（PS 端 LM 和 Zhang 用）

```cpp
class DMat {
    int r, c;
    f64* data;
};

namespace mat {
    DMat multiply(const DMat& A, const DMat& B);
    DMat transpose(const DMat& A);
    DMat pseudo_inverse(const DMat& A);   // SVD 实现
}
```

---

### `algo/grayscale` — PL

```cpp
void rgb_to_gray(const RgbImage& in, GrayImage& out);
```

公式：`Y = 0.299*R + 0.587*G + 0.114*B`

### `algo/threshold` — PL

```cpp
f32 otsu_threshold(const GrayImage& in);                    // PS 端算一次阈值
void binarize(const GrayImage& in, GrayImage& out, f32 thresh);  // PL 每帧跑
```

### `algo/chessboard_detect` — PL（加速）+ PS（逻辑）

```cpp
// PL 硬化部分
ProjectionResult integral_projection(const GrayImage& img);          // 水平/垂直积分投影 → 棋盘边界
void shi_tomasi_response(const GrayImage& img, GrayImage& response);  // 3×3 窗口响应图

// PS 逻辑部分：响应图 top-N → 聚类排序 → pattern_w×pattern_h 网格验证
DetectResult find_chessboard(const GrayImage& img, int pattern_w, int pattern_h);
```

### `algo/subpixel` — PS

```cpp
void refine_subpixel(const GrayImage& img, std::vector<Point2f>& corners,
                     int half_win = 5, int max_iter = 30, f32 eps = 0.001f);
```

### `algo/calibrate` — PS

```cpp
struct CalibrateInput {
    int pattern_w, pattern_h;
    f32 square_size;
    std::vector<std::vector<Point2f>> image_points;
};

struct CalibrateResult {
    CameraParams params;
    f64 rms;
};

CalibrateResult calibrate(const CalibrateInput& in);
```

**内部重投影公式**（PL 端 undistort 的反函数从这里推导，三处必须一致）：

```
x_d = X/Z, y_d = Y/Z
r2  = x_d^2 + y_d^2
x_r = x_d*(1 + k1*r2 + k2*r2^2 + k3*r2^3) + 2*p1*x_d*y_d + p2*(r2 + 2*x_d^2)
y_r = y_d*(1 + k1*r2 + k2*r2^2 + k3*r2^3) + p1*(r2 + 2*y_d^2) + 2*p2*x_d*y_d
u   = fx * x_r + cx
v   = fy * y_r + cy
```

### `algo/undistort` — 拆两部分

```cpp
// 部分 A：build_remap_table（PS 端，CALIB 阶段参数锁存后算一次）
struct RemapTable {
    int w, h;
    FloatMap map_x;    // 目标像素 (x,y) → 源图浮点坐标
    FloatMap map_y;
};

RemapTable build_remap_table(int w, int h, const CameraParams& cam);

// 内部：对每个目标像素反解畸变（固定点法，5 次迭代收敛）
// nx = (x-cx)/fx, ny = (y-cy)/fy
// for i in range(5):
//     r2 = nx^2 + ny^2
//     radial = 1 + k1*r2 + k2*r2^2 + k3*r2^3
//     xt = 2*p1*nx*ny + p2*(r2 + 2*nx^2)
//     yt = p1*(r2 + 2*ny^2) + 2*p2*nx*ny
//     nx = (nx - xt) / radial, ny = (ny - yt) / radial
// map_x[x][y] = fx*nx + cx,  map_y[x][y] = fy*ny + cy

// 部分 B：remap_bilinear（PL 硬化目标，RUN 阶段每帧实时跑）
void remap_bilinear(const GrayImage& src, const RemapTable& table, GrayImage& dst);

// 内部：逐像素查表 → 取 4 邻域 → 双线性插值
//  PL 实现重点：用 line buffer 缓存前一行 + 当前行已读像素形成 2×2 滑窗，
//  每 4 周期输入 → 每 1 周期输出 1 个矫正像素
```

---

## main.cpp 顶层流程

```cpp
int main() {
    // === CALIB ===
    for (int i = 0; i < N; ++i) {
        auto raw = load_ppm("chess_*.ppm");
        GrayImage gray(raw.w, raw.h);
        rgb_to_gray(raw, gray);
        // PL: integral_projection, shi_tomasi_response
        // PS: find_chessboard, refine_subpixel
    }
    auto cam = calibrate({...}).params;
    auto table = build_remap_table(w, h, cam);
    // 参数锁存，remap table 写入 BRAM

    // === RUN ===
    while (true) {
        auto frame = capture_frame();
        GrayImage gray(frame.w, frame.h);
        rgb_to_gray(frame, gray);                      // PL
        GrayImage corrected(frame.w, frame.h);
        remap_bilinear(gray, table, corrected);        // PL 核心
        save_pgm("out.pgm", corrected);
    }
}
```

---

## 开发步骤

### 阶段 A：C++ 完整链路（和 OpenCV 数值对齐）

| 步骤 | 内容 | 验证标准 |
|------|------|---------|
| **0** | `types.h` + `image.h` + `io.cpp` | 能 load/save PPM，打印像素值 |
| **1** | `grayscale.cpp` | 输出和 OpenCV `cvtColor` 像素值一致 |
| **2** | `undistort.cpp` 的 `build_remap_table` + `remap_bilinear`，**先用 OpenCV 硬编码一组假参数** | 输出和 OpenCV `cv::undistort` 像素级一致（误差 < 2） |
| **3** | `chessboard_detect.cpp`（PL + PS 部分全写 C++） | 角点坐标和 `findChessboardCornersSB` 差 < 1.0 像素 |
| **4** | `subpixel.cpp` | 和 `cornerSubPix` 差 < 0.1 像素 |
| **5** | `matrix.h` 完整实现 | `multiply` + `pseudo_inverse` 和 `cv::SVD` 数值一致 |
| **6** | `calibrate.cpp`（Zhang + LM） | CameraParams 和 `calibrateCamera` 差 < 0.1%，rms < 2 |
| **7** | Step 6 真实参数跑 Step 2 | **完整闭环**，矫正图和 OpenCV 版无肉眼差别 |

### 阶段 B：移植 Verilog

| 步骤 | 内容 |
|------|------|
| **HDL 0** | `grayscale.v` — 3 乘法 + 2 加法，纯流水线 |
| **HDL 1** | `integral_projection.v` + `shi_tomasi.v` |
| **HDL 2** | **`remap_bilinear.v`** — line buffer + 插值流水线，最核心 |
| **HDL 3**（可选） | `build_remap_table` 也移到 PL，进一步加速 CALIB |

---

## 最终硬化清单

| Verilog module | 来源 | 说明 |
|---------------|------|------|
| `grayscale.v` | `algo/grayscale.cpp` | RGB→Gray，每时钟 1 像素 |
| `integral_projection.v` | `algo/chessboard_detect.cpp` | 积分投影找棋盘边界 |
| `shi_tomasi.v` | `algo/chessboard_detect.cpp` | 3×3 窗口角点响应 |
| `binarize.v` | `algo/threshold.cpp` | 阈值比较器 |
| **`remap_bilinear.v`** | `algo/undistort.cpp` | 查表 + line buffer + 双线性插值 |