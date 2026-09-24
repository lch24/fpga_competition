# M2 说明文档：Shi-Tomasi 检测前端与两遍扫描（交接 / 原理理解）

- 负责人：苏晨（corner 分支）
- 状态：已完成，提交 `65b8ef8`
- 对应规划：VERILOG_DESIGN_PLAN 第 5.1 / 5.2 / 10 / 11 / 13.2 节
- 用途：本文件面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M2 把 Shi-Tomasi 检测的**像素级流水前端**和**两遍扫描控制**全部落地，输出与 C++ 参考实现**逐位一致**。

| 交付 | 验收结果 |
|---|---|
| 前端数据流（gray→window→sobel→tensor→窗口和） | 3 张 1280×720 真实图，gray/ix/iy/sum 四级 921600 像素全部位级一致（ALL PASSED） |
| 浮点算术库（fp32×4 / fp64×4） | min_eigen 4034 笔（真实3000+随机1000+定向34）位级一致（ALL PASSED） |
| min_eigen_core（混合精度复刻） | 同上 |
| detect 三模块（store / nms / ctrl） | 行为 TB（5 场景）+ 32×24 棋盘全链对拍（resp/rmax/candidates 全一致） |

**不在 M2 范围**：候选后处理（MERGE/亚像素/RING，M3）、网格排序（M4）、金字塔多尺度（M6）、DDR 对接（M6）。

---

## 2. 流水线总览

```
BGR888 字节流（光栅序）
   │ bgr_to_gray（整数精确）
   ▼
window3x3(CLAMP)  ── 3×3 灰度窗
   │ sobel_core（gx/gy，s16 精确整数）
   ▼
tensor_core（xx/xy/yy，s32）
   │ tensor_window_sum(ZERO)（9 项窗和，s32）
   ▼
s32_to_f32 ×3（整数→fp32 精确）
   │ min_eigen_core（混合精度：fp32 trace/det + fp64 sqrt/sub）
   ▼
┌────────────────────────────────────────────┐
│ shi_tomasi_ctrl 两遍扫描                     │
│  PASS1: response_store_max 顺序存 resp + rmax│
│  PASS2: 回读 resp → window3x3(ZERO) →        │
│         nms_candidates(3×3 NMS) → 候选坐标流   │
└────────────────────────────────────────────┘
```

两个边界语义不同的窗口生成器并存（规划 5.2 明确要求，不许混用）：
- **Sobel 窗**：`BORDER_CLAMP=1`，边缘复制（对应 C++ `std::max/min` 钳位）；
- **张量窗 / PASS2 响应窗**：`BORDER_CLAMP=0`，图外贡献为零（对应 C++ `safe_get` 返回 0）。

---

## 3. 模块详解

### 3.1 前端（rtl/kernels/，M2 前半交付，含在 `ea2cce1`）

| 模块 | 原理 | 备注 |
|---|---|---|
| `bgr_to_gray.v` | `(299R+587G+114B+500)/1000` 整数除法 | 输入 BGR 顺序，权重对应 R/G/B |
| `window3x3.v` | 两行 read-first RAM 按行奇偶轮换 + 列移位寄存器 | **本模块流控架构是全项目的通用样板，见 §4.1** |
| `sobel_core.v` | 组合梯度，1 拍寄存 | s16，与 f32 位模式一一对应（值<2^24） |
| `tensor_core.v` | gx²/gx·gy/gy² | s32 |
| `tensor_window_sum.v` | 内嵌 window3x3(CH=3,DW=32,ZERO) + 9 项加法树 | 求和结果仍是**整数 s32**，不是 fp32！ |

### 3.2 浮点算术库（rtl/arithmetic/，M2 后半）

| 模块 | 说明 |
|---|---|
| `fp32_mul` / `fp32_add` / `fp32_sub` | RNE；加减法用 **G=52 守卫小数位定点窗**对齐，保证本值域内整数加减精确 |
| `fp64_mul` / `fp64_sub` | RNE 双精度（本域乘积<2^50 精确） |
| `fp64_sqrt` | 整数逐位二分 isqrt（55 位）→ RNE 舍入到 52 位尾数 |
| `f64_to_f32` | RNE 转换 |
| `s32_to_f32` | 有符号整数→fp32，|n|<2^24 时**精确**（集成时补的桥接件） |

所有 fp 单元均按"内部无反压直出 + 输出 sync_fifo 水位冻结"的弹性模式实现（§4.1）。

### 3.3 min_eigen_core（rtl/kernels/min_eigen_core.v）

逐位复刻 `kernels::min_eigenvalue`，运算语义**必须逐级一致**（这是本模块成败关键）：

```cpp
trace = fp32(RNE(a+c));
det   = fp32(RNE(RNE(a×c) − RNE(b×b)));        // 先乘后减，每步都舍入
discr = double(trace)·trace − 4.0·det;          // fp64；本域精确，无新舍入
s     = fp64 RNE sqrt( max(0.0, discr) );
out   = fp32(RNE(trace − s)) × 0.5f;            // 先转 f32 再 ×0.5（不是 trace/2）
```

要点：
- **不是**全整数/全 fp64——det 是 fp32 运算（有舍入），必须在 RTL 里同样用 fp32 做，否则末位不一致。
- 内部结构：输入三分叉 lockstep（全局 all_ready 门控，防 oversample）+ 组合 join 汇合 + `stream_fork2` 拆分 trace 双路。
- 延迟 ~20 拍（含各 FIFO 抖动与组合 isqrt），任意下游反压不丢。

### 3.4 detect 三模块（rtl/detect/，B 代理交付）

| 模块 | 行为 |
|---|---|
| `response_store_max.sv` | 双口 RAM（PASS1 写 / PASS2 registered-read）；rmax 逐拍 fp32 位比较更新；**pass1_done 收满后电平保持**（见 §6 坑3） |
| `nms_candidates.sv` | 3×3 窗内判 `v>0 && v>=thr && 8 邻居 v>=邻居`（相等不抑制，严格 `>`）；内部 FIFO(深16) 承压 |
| `shi_tomasi_ctrl.sv` | 状态机编排：IDLE→PASS1→P2INIT→P2RUN→WAIT；`mem_addr = r_addr+1`（registered 读提前 1 拍）；P2RUN 内 window 背压冻结读地址 |

PASS2 的内缩边界（对应 C++ 循环）：中心 `(x,y)∈[2,W-3]×[2,H-3]`，由 nms 按窗口坐标判定。

---

## 4. 三个关键设计决策

### 4.1 弹性流水（window3x3 定型，全项目复用）

**背景**：移位式流水 + 外部任意 ready 波形，若只冻结数据不移位会丢数据；若无条件推进元数据会错位（详见 §6 坑2）。

**定型方案**（所有 fp 单元、window3x3 均采用）：
```
生成核内部"无反压直出"：
  所有流水级推进使能统一源自 scan_fire 的延迟脉冲（fire_d/fire_d2）
  —— fire 一停，全流水整体冻结，不存在"数据冻结而元数据推进"的错位。
下游背压由输出侧 sync_fifo（深8）吸收：
  fire 冻结条件 = FIFO 水位 ≥ 6（在途窗口 ≤ 2）。
```
推论：**带延迟/背压的模块之间一律用 FIFO 桥接**，不要把弹性输出直接接"恒 ready"的消费者（§6 坑4 正是此坑）。

### 4.2 fp32 位比较技巧（B 代理修正版）

IEEE fp32 比较等价于无符号比较序位变换（NaN 不在本域）：

```verilog
function automatic [31:0] f2o(input [31:0] v);
    f2o = v[31] ? ~v : (v | 32'h8000_0000);   // 标准版：负→取反，正→置符号位
endfunction
```

> 注意：早期我给的 `{~a[31], a[30:0]}` 有符号变换对**负值顺序错误**（同号内反向、负号整体后置），B 代理在实现中发现并改为上式。**所有 fp32 比较（rmax、NMS）统一用这个修正版。**

### 4.3 两遍扫描（帧屏障）

阈值依赖全图 rmax → 天然两遍：PASS1 边算边存响应边求 rmax；PASS2 回读 RAM 重扫做阈值+NMS。这是 C++ 语义的硬性要求，**不能**单遍流实现。

---

## 5. 验证方法与结果

### 5.1 三层验证栈

| 层 | 工具 | 覆盖 |
|---|---|---|
| fp 单元/组合链 | `sim/tb_arith.sv` + `test0_eigen.bin`（4034 笔） | min_eigen 全路径位级 |
| ctrl 行为 | `sim/tb_detect.sv`（5 场景） | 全0/负resp/角点/阈值边界/连续帧 |
| 全链对拍 | `sim/tb_resp.sv` + `export_small.cpp`（32×24 棋盘） | BGR→candidates 端到端位级 |

### 5.2 关键结果

```
eigen:  ALL ARITHMETIC TESTS PASSED (4034/4034)
ctrl:   ALL DETECT TESTS PASSED
small:  resp 768/768 | rmax 0x49a95600==EXP | cand 140/140 | ALL RESP/DETECT TESTS PASSED
front (test0 1280x720 回归): gray/sobel/tensor/sum 921600/921600 ALL PASSED
```

### 5.3 向量生成链

- 真实图：`run_export_gcc.cmd`（v3，含 `test0_eigen.bin`）
- 小图：`tests/rtl/export_small.cpp` 编译运行 → `tests/build/vectors/small_*`（32×24 棋盘，rmax≈1.39e6，140 候选）

---

## 6. 调试记录（交接价值最高的一节）

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 窗口字段映射颠倒，gx/gy 全变号；**sum 却全对**（平方不变、xy 双负抵消） | window 拼接序高位=左上，sobel 按低位=左上解析 → 窗口 180° 旋转 | 统一拼接/解析约定 |
| 2 | **每行丢 1 个窗口**，仅下游有背压时触发 | 列移位寄存器用 fire 脉冲冻结，但坐标延迟链无条件更新 → 反压解除后坐标跳拍 | 定型为 §4.1 弹性流水 |
| 3 | PASS1 永远收不满死锁 | `pass1_done` 单拍脉冲在 ctrl 的 posedge 采样竞态下被错过（读到更新前旧值，下一拍已回 0） | **pass1_done 改为电平保持** `(wcnt>=PIXELS)` |
| 4 | 集成全链卡死 | ctrl 拿 store 恒 1 的 in_ready 当真实握手传播给弹性输出的 min_eigen → 静默丢像素 | me 与 ctrl 之间插 sync_fifo 桥接；`resp_rdy = PASS1 ? me.in_ready : 0` |

**另两个环境坑**：
- ctrl 例化时**忘记传 IMG_W/IMG_H 参数**（用了默认 1280×720），喂小图永远收不满——集成 TB 必须显式覆盖所有参数。
- ModelSim `-novopt` 下 1280×720 全链仿真约 3 小时不可行 → 全链验证定为**小图/合成图**，大图只做前端分段对拍（回归已过）。

---

## 7. 复现命令

```
# 1) 小图全链对拍（32×24，~1 秒）
Set-Location closer2fpga\sim
$env:PATH = "D:\MentorGraphics\64_10.6e\win64;" + $env:PATH
vlog -sv <M2全部rtl文件> tb_resp.sv
vsim -c -suppress 3829 -do "run -all; quit -f" tb_resp

# 2) fp 单元对拍（4034 笔）
vsim -c -suppress 3829 -do "run -all; quit -f" tb_arith

# 3) 前端大图回归（test0）
vsim -c -suppress 3829 -do "run -all; quit -f" tb_front
```

完整文件列表见 §3，vlog 时按 common→arithmetic→kernels→detect→TB 顺序即可。

---

## 8. 遗留事项（M3 衔接）

1. `response_store_max` 的 `wcnt/pass1_done` 只在 rst_n 复位——**连续两帧复用需在 start 时清零**（当前 TB 单帧）。M6 顶层编排要处理。
2. `vision_defs.vh` 接口仍未冻结（阻塞 M6 顶层，不阻塞 M3-M5）。
3. M3 起步：`candidate_store.v`（A/B 双列表）→ `candidate_merge.v` → `nearest_spacing.v` → `ring_check.v`，对拍对象需在 C++ 侧新增候选级向量导出。
