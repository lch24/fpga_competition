# M4 说明文档：网格排序（INDEX_SORT / FP32_LOG / GRID_VALIDATE / GRID_ORDER_CTRL）

- 负责人：苏晨（corner 分支）
- 状态：已完成
- 对应规划：VERILOG_DESIGN_PLAN 第 5.4 / 5.5 / 13.2 节
- 用途：面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M4 把 `detect_native` 的网格排序段（散点 → rows×cols 行列序角点）落地：输入是 M3 的 inner 点流，输出 40 点行列序坐标，与 C++ 参考实现（`ordering.cpp::organize_grid` 的**确定性排序变体**）**逐位一致**。

| 交付 | 验收结果 |
|---|---|
| `index_sort.sv`（双 RAM 迭代归并排序核） | 9/9 单元向量位级一致（n=8/16/40 × 随机/含相等/全相等） |
| `fp32_log.v`（自定义 fp32 log，log_ref 位级） | 4010/4010 位级一致（4000 密集 + 10 边界/特殊） |
| `grid_validate.sv`（网格代价判定，cost_ref 复刻） | 6/6 位级一致（理想/扰动/短边/共线/折线/随机） |
| `grid_order_ctrl.sv`（90 方向编排主控） | board5x8 **全链**（inner 80 点 → grid 40 点）位级一致（40/40） |

场景 board5x8：全图 6×17 格棋盘（格 16px，灰度 96×272）→ M3 链路（无 subpixel 变体）→ 内部角点 **80**（5 行×16 列）→ organize_grid → **40 点**；实测 best 角度 ak=44（degree=-2）、cost=0、origin=0、status=01/`grid_ok=1`，与 `m4_board5x8_grid.bin` 逐点位级一致。

**不在 M4 范围**：subpixel 精定位核（M5，届时 organize_grid 输入点集改变，需带 subpixel 变体重跑全链）、`grid_refine_ctrl` 编排（§5.5，复用本阶段 grid_validate + 后续 subpixel）、金字塔多尺度（M6）、DDR 对接（M6）。

---

## 2. 流水线总览

```
inner 点流（fp32 坐标，来自 M3 candidate_filter_ctrl）
   │
grid_order_ctrl：90 方向角度循环（cos/sin ROM，degree = -90 + 2k）
   │  A_PROJ   ：u =  x·co + y·si；v = -x·si + y·co → f2o 单调 key
   │  A_VSORT  ：index_sort 按 v 全序排序 → order[]
   │  A_GAP    ：gap = v(order[i+1]) - v(order[i])；同值按位置倒序 tie-break
   │  A_GSORT  ：升序取末 rows-1 个（= 最大间隙）→ 恢复位置
   │  A_GPSORT ：位置升序 → 行边界 gaps_arr[]（末尾补 N-1）
   │  A_ROW    ：逐行：段内按 u 排序（index_sort）→ 连续 COLS 窗口枚举
   │               （中位间距 median、spacing<4 跳过、score=Σ((step-sp)/sp)² 取最小）
   │               → 行点拷贝 grid[r*COLS+c]
   │  A_COST   ：grid_validate → cost（1e30f = 无效）
   │  A_BEST   ：严格更小才覆盖 best（40 点）
   │  A_ORIGIN ：4 端角 x+y 最小定原点，按原点翻转行/列重排
   ▼
40 点行列序输出流（out_x/out_y + out_total + out_grid_ok）
```

时间线：`S_IDLE → S_CAP（收 inner 点）→ S_ANGLE（90 角度循环）→ S_DONE（输出）`；status=01 成功、10 点不足/失败。

---

## 3. 模块详解

### 3.1 `index_sort.sv`：双 RAM 迭代归并（rtl/detect/）

organize_grid 中所有"按投影排序"共用同一个排序引擎。语义（**不得改动**，与 `export_m4.cpp::export_sort_cases` 位级一致）：

- 全序 `less(a,b) = (key_a < key_b) || (key_a == key_b && idx_a < idx_b)`；
- key 为 32bit 位模式，比较采用**位模式无符号序**；相等 key 平局按原索引升序（归并稳定）；
- 输出排序后的**原索引**序列（0..n-1），主控据索引回查真实坐标/投影值。

结构三态：`LOAD`（从外部 key/idx RAM 1 拍延迟读，写内部 RAM A）→ `MERGE`（run_len 从 1 倍增，每轮读两有序段比较写另一 RAM，轮末翻转 src_sel；n 不必为 2 的幂，段对末尾裁剪，剩单段直接拷贝）→ `OUT`（顺序输出 idx 流，out_ready 背压）。

时序（关键）：`rd_en/rd_addr` 是**输出**（直连外部 RAM 读口）；RAM 1 拍延迟 → 数据"请求后第 2 拍"组合可见，所有吸收/写入走两级流水；每拍至多一个读请求在途，吞吐 1 元素/3 拍。

### 3.2 f2o 单调映射（调用侧约定）

RTL 排序用无符号比较，而 u/v 投影可为**负** float。调用侧把 key 做 f2o 映射后无符号比较即等价于 float 数值序：

```
f2o(v) = v[31] ? ~v : (v | 0x80000000)      // 位级单调
```

主控所有送排序的 key（v、u、gap、7 距离中位数）均先 f2o 再入 RAM。

### 3.3 `fp32_log.v`：自定义 log（位级权威 = log_ref）（rtl/detect/）

**核心决策**：实测 libm `logf ≠ (float)log(double)`（200 万样本中约 8538 例 1 ulp 差），无法以"标准库等价"当权威 → 自定义 `log_ref`（atanh 级数 Horner 嵌套）为位级权威，RTL 复刻同序运算：

```
z  = (x-1)/(x+1)                  // x∈[0.55,1.8] → |z|≤0.29，8 项截断误差 ≪ 1ulp
z2 = z*z
p  = 1/15 + z2*(1/13 + z2*(1/11 + z2*(1/9 + z2*(1/7 + z2*(1/5 + z2*(1/3 + z2*1))))))
log = 2*(z*p)                     // 先 z*p 再 ×2，严格按 C++ 运算顺序
```

实现：弹性握手级联（同 fp32_hypot 风格），z/z2 锁存寄存器（被多次消费），**单飞行**（一个输入走完整链前不收新输入，避免数据竞争）；接受拍 → out_valid 约 40 拍。系数位模式由 g++ 同机打印（`c13=3eaaaaab`、`c15=3e4ccccd`、`c17=3e124925`、`c19=3de38e39`、`c111=3dba2e8c`、`c113=3d9d89d9`、`c115=3d888889`，文件头有全表，勿自算）。

### 3.4 `grid_validate.sv`：网格代价判定（cost_ref 复刻）（rtl/detect/）

逐位复刻 `validation.cpp::grid_cost`（仅 log 换 log_ref）。对每格点（顺序 r*cols+c）：

- **双轴**（axis=0 步 1、axis=1 步 cols；`pos+2>=count` 跳过）：`l1/l2=hypot(dx,dy)` 两条边 → `l1<4 || l2<4 || l2/l1<0.55 || l2/l1>1.8` 无效；`cosine=(dx1*dx2+dy1*dy2)/(l1*l2) < 0.90` 无效；`change=log_ref(l2/l1)`；`cost += (1-cosine) + change²`；
- **单格**（r+1<rows && c+1<cols）：q[4]={p, +1, +cols+1, +cols} 逐 k：`cross=(b.x-a.x)*(d.y-b.y)-(b.y-a.y)*(d.x-b.x)`、`lengths=dist(a,b)*dist(b,d)`；`lengths<16 || |cross|<lengths*0.2` 无效；首格定转向符号 `sign`，`cross*sign<=0` 无效；
- 任一检查失败**立即**输出 1e30f（0x7149f2ca）。

实现：顺序扫描状态机 + 算术模块复用（4×fp32_sub、2×fp32_hypot、3×fp32_mul、2×fp32_add、2×fp32_div、1×fp32_log），每阶段"驱动+等待 out_valid"；网格点由外部 RAM 提供（1 拍读延迟）。`l2/l1` 的除法结果一份三用（ratio 检查、cosine 分母、log 输入），保证位级一致。

### 3.5 `grid_order_ctrl.sv`：90 方向编排主控（rtl/detect/）

两块状态机：块 A（顶层 `S_IDLE/S_CAP/S_ANGLE/S_DONE`）+ 块 B（角度子机），流程见 §2；另有行子机（R_RSORT/R_WIN/R_GCPY/R_NEXT）与窗口子机（W_STEP/W_MED/W_SCORE/W_SKIP）。

关键点：

- **gap tie-break**：`gapi=(N-2)-i`（同值位置小者排末尾 → 升序取尾 rows-1 个 = "同值优先位置小者"），与 C++ 稳定语义一致；
- **中位数**：7 个距离用 index_sort n=7 取第 4 个（全排序取 [size/2]，与 nth_element 值相同）；
- **行窗口**：段 `[begin,end)` 内枚举连续 COLS 窗口，`spacing<4` 跳过；score 累加后取严格更小者；
- **best 更新**：`cost < best_cost`（严格更小），与 C++ 相同；
- **原点规范化**：4 端角（0 / COLS-1 / (ROWS-1)*COLS / ROWS*COLS-1）取 x+y 最小者，按原点翻转行/列重排输出。

存储：pts/keyv/idxv/vv/order/gapk/gapi/keyu/idxu/grid/best 用 `dual_port_ram`（1 拍延迟读）；gaps_arr/gap_pos/win_order/wstep_r/wmed_key/best_buf 用寄存器数组（0 延迟，喂 index_sort 时按 1 拍语义适配）。

时序关键（复述文件头）：

- dual_port_ram 1 拍：请求拍 → 下一拍数据组合可见；
- fp32 算术 2 拍：fire 拍接受 → 2 拍后 out_valid；
- index_sort 的 rd_en/rd_addr 为其输出，直接驱动 key/idx RAM 读口（主控不介入）；
- grid_validate 同样输出 rd_en/rd_addr，经 gv_busy 多路选通 u_grid 读口（A_COST 期间归 gv，其余归主控）；
- 输出 out_x/out_y 为组合（best_buf[oc_idx2]），out_valid 为 reg。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_sort.sv` | index_sort：n=8/16/40 × {随机/含相等/全相等} 共 9 例 | ALL PASSED (9/9) |
| `tb_log.sv` | fp32_log：4000 密集 + 10 边界/特殊 = 4010 例 | ALL PASSED (4010/4010) |
| `tb_cost.sv` | grid_validate：理想/扰动/短边/共线/折线/随机 6 例 | ALL PASSED (6/6) |
| `tb_order.sv` | grid_order_ctrl 全链：inner 80 点 → 40 点位级（含原点规范化） | ALL PASSED (40/40) |

向量导出工具（`tests/rtl/`，g++ 编译）：`export_m4.cpp`（log_ref / cost_ref / organize_grid_det 确定性变体，导出全部向量）、`gen_grid_angles.cpp`（90 方向 cos/sin ROM：行 2k=cos、2k+1=sin，degree=-90+2k，同机 libm 位模式）。

产物（`tests/build/vectors/`，不入库，可用上述工具重新生成）：`m4_board5x8_{gray,inner,grid}.bin`、`m4_sort_{8,16,40}_{0,1,2}.bin`、`m4_cost_{0..5}.bin`、`m4_log.bin`、`grid_cos_sin.mem`（180 行）。

复现：先运行两个导出工具生成向量 → ModelSim（vlog 编译对应 RTL+TB → `vsim -c tb_xxx`）。全链 tb_order 仿真约 7 分钟（90 角度 × 40 点 × cost 链）。

---

## 5. 场景设计与踩坑记录

**场景迭代（为什么用全图棋盘）**：

1. 5×8 棋盘（局部）→ inner 仅 18：图像边缘角点 ring 检查越界被淘汰，凑不满 40；
2. 加大棋盘（11×13）→ inner 99 但 organize_grid FAIL：均匀网格下行间隙大小近似（存在多个真实行界 + 行内小间隙），"取最大 4 个"不能唯一确定真实 5 行 → 行分割错位；
3. 最终 **全图 6×17 格（16px）** → inner 恰好 5 行×16 列=80：目标 5 行只需 4 个行界，行间间隙（≈16px）远大于行内间隙（≈噪声）→ 分割无歧义；每行 16 点窗口枚举自然选出连续 8 列 → 40 点。

**过程中发现并修复的问题**：

1. **logf ≠ (float)log(double)**（8538/2M 1ulp）→ 放弃"libm 等价"，自定义 log_ref 作位级权威（§3.3）；
2. **排序语义不一致**（子代理发现）：初版单元向量生成器有 `key[-1]` 越界 UB，且 C++ float 比较与 RTL 无符号 key 语义冲突 → 生成器加 `i>0` 保护 + 统一 f2o 映射，9/9 重验通过；
3. **grid_order_ctrl 首版骨架问题**（子代理修复）：写地址组合直连导致 posedge 采样已递增值（需寄存）；A_GSORT 多收 1 个 gap（应只处理 ROWS-1 个）；scp 位宽截断；RAM 读实为 2 拍（请求→转移→吸收）；reg 数组 0 延迟读与 index_sort 1 拍期望错配；vsim 时间刻度 1ps 下看门狗需 `#2e12`；
4. **export_m4 编译**：`grid_cost` 命名空间问题（改本地 cost_ref 复刻）+ 补 `subpixel.cpp` 链接。

---

## 6. 遗留事项与接口契约（交接重点）

- **M5 subpixel 接入后必须重跑 M4 全链**：organize_grid 输入点集将变为精定位坐标，需用带 subpixel 的 C++ 变体重新导出向量对拍；
- **`grid_refine_ctrl`（§5.5）未做**：求最短边 → `clamp(int(min_step*0.15),2,10)` 半窗调 subpixel → 再 grid_validate；本阶段已提供可复用的 `grid_validate`，编排属 M5/M6；
- **排序契约声明**：与原始 C++ `std::sort`（未约定相等 key 次序）在退化平局下**不保证**逐位一致；RTL 与确定性变体（全序 tie-break）逐位一致（§5.4 已声明）；
- **参数**：`ROWS=5, COLS=8, N_ADDR_W=8`（256 点容量）、`GAP_ADDR_W=8`；`ROM_FILE` 为参数（默认 `../tests/build/vectors/grid_cos_sin.mem`），综合时需换 ROM/IP 或内联；
- **性能**：全链瓶颈在 90 角度 × A_COST（每角度每点双轴 log）；未做角度早停/剪枝（保持与参考逐位一致优先）。