# M5 说明文档：亚像素精定位与网格精定位（SUBPIXEL / GRID_REFINE）

- 负责人：苏晨（corner 分支）
- 状态：已完成（待提交）
- 对应规划：VERILOG_DESIGN_PLAN 第 5.4 / 5.5 / 6 / 13.2 节
- 用途：面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M5 把 `detect_native` 的亚像素段全部落地：(1) merge5 与 merge3 之间的 `refine_subpixel(gray, cand, 7)`；(2) organize_grid 之后的 `refine_grid`（最短边半窗 → 亚像素 → 网格再验证）。输出与 C++ 参考（`subpixel.cpp` + `validation.cpp::refine_grid`，organize_grid 用确定性变体、log 用自定义 log_ref）**逐位一致**。

| 交付 | 验收结果 |
|---|---|
| `fp64_add.v` / `fp64_div.v`（补齐 FP64 四则） | 603 例定向集 ALL PASSED（±0/±1/±Inf/NaN/subnormal/2^53 tie/舍入边界）；被 tensor/accum/subpixel 集成位级复用验证 |
| `bilinear_core.v`（fp32 双线性插值） | 510/510 位级一致 |
| `tensor_solve.v`（fp64 2×2 求解 + 可靠性判定） | 407/407 位级一致（含 trace/det 失败边界） |
| `subpixel_accum.v`（五项 fp64 加权梯度累加） | 8 窗口（r=2..15）位级一致 |
| `subpixel_ctrl.sv`（点/40 轮/窗口三级循环 + 失败回滚） | 3 场景（r=7/2/15）×23 点逐点位级一致，覆盖收敛/病态/越界/NaN 路径 |
| `candidate_filter_ctrl.sv`（S_SUBPX 接入） | board5x8 全链 merge5/subpixel/merge3/inner 各 80/80 位级一致 |
| `grid_refine_ctrl.sv`（最短边半窗 + 再验证） | refine_grid 40/40 + valid=1 位级一致 |

全链场景 board5x8（全图 6×17 格，96×272）：shi_tomasi 320 → merge5 80 → **subpixel(7) 80** → merge3 80 → inner 80 → organize 40 → **refine 40**，与 `export_m5.cpp::export_fullchain` 逐点位级一致。

**不在 M5 范围**：金字塔多尺度与 DDR 对接（M6）、标定（M6/M7）、remap（后续）。`subpixel_ctrl` 同时服务初始候选与最终 40 点（grid_refine 复用）。

---

## 2. 流水线总览

```
candidate_filter_ctrl：
  S_CAP → S_MERGE5 ──▶ S_SUBPX ──▶ S_MERGE3 → S_NEAR → S_RING → S_DONE
                       │ subpixel_ctrl(gray, cand, half_win=7)
                       │ 输出流写回 store A（失败点保留原值）
                       └ gray 读口 2 选 1 / store 读口 4 选 1（阶段互斥）

grid_refine_ctrl：
  S_MINSTEP（40 点横/纵边最短间距 fp32_hypot）
  → S_HALFWIN（half_win = clamp(int(min_step·0.15f), 2, 10)）
  → S_SUBPX（subpixel_ctrl 调用，40 点精定位，写回中间 RAM）
  → S_VALIDATE（grid_validate 再验证，cost<1e30f → valid）
  → S_OUT / S_DONE（valid=1 出 40 点流；valid=0 清空）
```

亚像素精度链（RTL 与 C++ 逐位一致的关键）：

```
sample：floor 取整 + 小数部分（fp32 域）→ bilinear_core（fp32，top→bottom→lerp 顺序）
gx/gy ：sample(sx±1,sy)/(sx,sy±1) 差分（fp32_sub，float 精度）→ f32_to_f64 提升（精确）
w     ：exp(-(x²+y²)/r²)，double —— 离线 ROM（r=2..15 全表，RTL 不实现 FP64 exp）
累加  ：outer_product {w·gx², w·gx·gy, w·gy²}（左结合 fp64_mul）→ a/b/c/bx/by 逐次累加
       bx += xx·x + xy·y（先乘后加再累加；窗口序 y 外 x 内）
solve ：det=a·c-b·b；trace=a+c；trace<1e-8 ∥ det≤(1e-5·trace)·trace → 失败
       dx=(c·bx-b·by)/det；dy=(a·by-b·bx)/det（全部 fp64，RNE）
next  ：f64_to_f32(p+dx/dy)；isfinite && hypot(next-original)≤r 才接受
收敛  ：dx²+dy²<1e-6（fp64）→ reliable；40 轮未收敛 → 保留 original
```

---

## 3. 模块详解

### 3.1 FP64 基础件（rtl/arithmetic/，M5 新增两个）

| 模块 | 原理 | 关键决策 |
|---|---|---|
| `fp64_add.v` | IEEE-754 binary64 加法 | 完整 RNE（±0/subnormal/NaN/Inf/次正规结果正确舍入）；**M3 曾用"fp64_sub 翻 b 符号"冒充加法**，亚像素每窗口 4 次累加必须真实加法 |
| `fp64_div.v` | 恢复余数长除（54 步）+ RNE | 与 fp32_div 同风格；subnormal 输入/结果全支持 |

验证：定向 603 例（add 320 + div 283）ALL PASSED；全量 121441 例向量已生成（`test_fp64.bin`），因 fp64_div 54 层组合链在 ModelSim 约 0.6s/周期、全量需数小时，**留待安静窗口跑**（遗留事项，模块已被下层集成位级验证）。

### 3.2 `bilinear_core.v`（rtl/kernels/）

逐位复刻 `kernels::bilinear`：`top=(1-dx)·p00+dx·p10`；`bottom=(1-dx)·p01+dx·p11`；`out=(1-dy)·top+dy·bottom`。每次运算 fp32 逐次舍入，不得融合/重排。2 拍流水 + 弹性握手。

### 3.3 `tensor_solve.v`（rtl/kernels/）

逐位复刻 `kernels::solve_tensor`（double）：det/trace/可靠性判定（`(1e-5·trace)·trace` 左结合两次 fp64_mul）/除法。常量 1e-5、1e-8 位模式来自 `m5_const.txt`（g++ 打印）。ok=0 时输出任意（TB 只比 ok）。

### 3.4 `subpixel_accum.v`（rtl/detect/）

五项 fp64 累加核：逐样本流水（3×fp64_mul + 2×fp64_add，共享 w·gx/w·gy），start 清零并锁存 n_win，收齐后一次出 {a,b,c,bx,by}。累加顺序（y 外循环 x 内循环）与 m5_acc.bin / C++ 完全一致。

### 3.5 `subpixel_ctrl.sv`（rtl/detect/）

M5 核心编排，三级循环（点 → 40 轮 → 窗口）+ patch 预装：

- **patch 预装**：每迭代按当前 floor(p) 将 `[floor(p.x)-r-1, floor(p.x)+r+2]×[floor(p.y)-r-1, floor(p.y)+r+2]`（2r+4 行×列，r=15 时 34×34）经 gray 读口装入 patch RAM（reg 数组，组合读）。**地址用图像行距 W 跳行，不是 patch 行距**（易踩坑）。首版每迭代重装（文档方案），未做 patch 复用优化。
- **窗口**：每位置 sx=cur_x+(w_x-r)（fp32），4 次 bilinear（sx±1/sy±1）+ 2 次 fp32_sub 差分 + f32_to_f64 提升 + w ROM 取权 → accum 喂一组。
- **solve/更新**：窗口收齐 → tensor_solve → 失败回滚出点；ok → next=f64_to_f32(p+dx/dy) → isfinite + fp32_hypot(next-original)≤r 检查 → 收敛（dx²+dy²<1e-6）置 reliable 出点 / 未收敛迭代继续（40 轮上限）。
- **回滚**：任何失败路径输出 original（reliable=0），点仍保留在点集（C++ 语义）。
- 非有限原始点直接跳过；越界（p<r+1 或 p≥W-r-2）立即失败。

### 3.6 `candidate_filter_ctrl.sv` 接入（S_SUBPX）

占位改为 SPX_CLR/SPX_RUN 子状态机：进入拍发 spx_start（half_win=7、n_in=merge5 点数）+ store 写侧回卷；SPX_RUN 收 subpixel 输出流写回 store A（全部写回，reliable 不筛）；done 后进 S_MERGE3。**相位轮换调整**：CAP/SUBPX/NEAR/RING=0（写 A 读 B）、MERGE5/MERGE3=1（写 B 读 A），保证 subpixel 读 merge5 结果、merge3 读 subpixel 结果、NEAR/RING 读 merge3 结果。store 读口 4 选 1、gray 读口 2 选 1（阶段互斥）。

### 3.7 `grid_refine_ctrl.sv`（rtl/detect/）

逐位复刻 `refine_grid`：S_MINSTEP 先装 40 点入中间 RAM，按 67 条横/纵边逐边 fp32_hypot 取 min（初值 FLT_MAX）；S_HALFWIN `fp32_mul(min_step, 0x3E19999A)` 截断 clamp(2,10)；S_SUBPX 调 subpixel_ctrl（40 点）写回；S_VALIDATE 调 grid_validate（cost<1e30f 无符号比较判有效）；valid=1 出 40 点流、valid=0 清空（无输出）。中间 RAM 读口 3 选 1、写口 2 选 1。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_fp64_adddiv.sv` | fp64_add/div 定向 603 例（±0/subnormal/NaN/Inf/2^53 tie/商≈0.5,1 边界/溢出/相消/随机） | ALL PASSED (320+283) |
| `tb_bilinear.sv` | bilinear 510 例（随机 + dx/dy 边界） | ALL PASSED (510/510) |
| `tb_tensor.sv` | tensor_solve 407 例（随机 + trace/det 失败边界） | ALL PASSED (407/407) |
| `tb_acc.sv` | subpixel_accum 8 窗口（r=2,2,3,4,7,7,10,15） | ALL PASSED (8/8) |
| `tb_subpixel.sv` | subpixel_ctrl 3 场景（s7/s2/s15）×23 点（收敛/病态/越界/NaN） | ALL PASSED |
| `tb_filter.sv` | candidate_filter_ctrl 全链（带 subpixel）：merge5↔spx_in、subpixel↔spx_out、merge3、inner | ALL PASSED (80/80×4) |
| `tb_order.sv` | grid_order_ctrl（organize 新 inner） | ALL PASSED (40/40) |
| `tb_refine.sv` | grid_refine_ctrl（grid 40 点 → refined 40 点 + valid） | ALL PASSED (40/40, valid=1) |

向量工具（tests/rtl/，g++ 编译）：`export_m5.cpp`（权威：bilinear/solve_tensor/refine_subpixel_traj/refine_grid_ref/export_fullchain，导出全部向量 + 高斯权重 ROM + m5_const.txt）、`gen_fp64_vectors.cpp`（fp64 121441 例全量向量）。

产物（tests/build/vectors/，可重新生成）：`m5_bilinear/tensor/acc/const`、`gaussian_weights.{mem,bin}`、`m5_s{2,7,15}_{gray,pts,out,traj}`、`m5_board5x8_{gray,cand,spx_in,spx_out,merge3,inner,grid,refined}`。

复现：g++ 编译两个工具生成向量 → ModelSim vlog + vsim（单实例，授权码并发受限——**多个 vsim 并行会 license 争用失败**，务必串行）。

---

## 5. 过程中发现并修复的问题

1. **fp64_div 尾数取位**：`!ageb` 分支（商∈[0.5,1)）用 q[53:2] 应为 q[52:1]（guard/round 错位）。
2. **fp64_div subnormal 输入**：Ea 指数拼接符号扩展错误。
3. **fp64_add 进位常量**：2^52 两处位数笔误。
4. **patch 灰度地址**：误用 patch 行距而非图像行距 W。
5. **acc 输入未赋值**：acc_gx_r/acc_gy_r 恒 0 → 求解全零。
6. **gray 读口错位**：rd_en/addr 为寄存器输出，预装整体错位 1 拍（patch[k]=gray[f(k-1)]）。
7. **tensor_solve done 保持高电平**：两次 start 间 done 不落，误读上一轮结果（加 solve_started 标志）。
8. **TB 收流时序**：out 流先于 done 流完，需在 done 前并行收集。
9. **tb_filter W/H 反置**：棋盘 272 宽 96 高，TB 初版反置导致 gray/subpixel 全错。
10. **shi_tomasi.cpp 聚合初始化**：`Point2f((f32)x,(f32)y)` 在 C++17 非法（无构造函数）→ 改 `Point2f{...}`（语义不变，修所有工具的编译）。
11. **测试向量缺原始候选**：export_m5 原只导出 merge5 后点，补 `m5_board5x8_cand.bin`（shi_tomasi 320 点）供 S_CAP→merge5 回填。

**项目级教训（license）**：ModelSim 授权码并发实例数有限。本次四个子代理并行跑 vsim 导致 license/进程争用（误判为"license 损坏"，实测环境正常）。**后续所有 vsim 必须串行（检查无 vsimk 进程再启动）**。

---

## 6. 遗留事项与接口契约（交接重点）

- **fp64 全量对拍**：121441 例向量已生成（`test_fp64.bin`），ModelSim 全量约需数小时，留安静窗口跑；模块正确性已被 tensor_solve/subpixel_accum/subpixel_ctrl 集成位级验证覆盖。
- **subpixel_ctrl 性能优化（未做）**：patch 每迭代重装（首版方案），窗口像素无复用；后续可做"p 移动 <1px 时 patch 复用/增量更新"，但**任何累加/求解顺序改动须重新对拍**（位级一致优先）。
- **M6 接入**：subpixel_ctrl 的 gray 读口目前接 TB 内存/DDR 模型，真实系统接 DDR（M6 对接时 gray 读口仲裁与多尺度金字塔复用）；金字塔每层恢复调用 grid_refine_ctrl。
- **half_win 语义**：初始候选固定 7（candidates.cpp:85）；refine_grid 半窗 `clamp(int(min_step·0.15f),2,10)`（board5x8 场景 min_step≈16px → half_win=2）。
- **输出契约**：candidate_filter_ctrl 现在输出**带亚像素**的 inner 流（坐标已精定位）；grid_refine_ctrl 输出最终 40 点（valid=1）供标定使用。
- **与原始 C++ 的排序契约不变**：organize_grid 仍为确定性变体（M4 报告 §6）。
