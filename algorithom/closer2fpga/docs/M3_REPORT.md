# M3 说明文档：候选后处理（MERGE / NEAREST / RING）

- 负责人：苏晨（corner 分支）
- 状态：已完成（待提交）
- 对应规划：VERILOG_DESIGN_PLAN 第 5.3 / 13.2 节
- 用途：面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M3 把 `detect_native` 的候选段（merge 去重 → nearest 间距 → ring 圆环筛选）全部落地，输出与 C++ 参考实现（`candidates.cpp`，无 subpixel 变体）**逐位一致**。

| 交付 | 验收结果 |
|---|---|
| fp32 基础件（`fp32_div` / `fp32_sqrt` / `fp32_hypot`） | 全量 70021 笔（hypot 25007 / div 25007 / sqrt 20007）位级一致（ALL PASSED） |
| `candidate_store`（候选 A/B 双缓冲） + `candidate_merge`（O(N²) 去重） | small(140→22→22) / texture(722→258→258) 两轮 merge 全点位级一致 |
| `nearest_spacing`（最近邻 + radius=clamp(0.22·spacing,4,18)） | 两场景 spacing/radius 全部位级一致 |
| `ring_check`（32 采样圆环判定，单半径）+ `lround_pkg` | 两场景每点 7 字段中间量位级一致（644 例 lround 自检全过） |
| `candidate_filter_ctrl`（阶段编排主控） | 两场景**全链**（NMS 候选→merge5→subpixel 直通→merge3→nearest→ring→inner）位级一致（ALL PASSED） |

场景对照：small（32×24）`140 → 22 → 22 → inner 0`；texture（128×96）`722 → 258 → 258 → inner 35`，与 `export_m3` 的 C++ 参考逐点一致。

**不在 M3 范围**：subpixel 精定位核（M5，本阶段以直通占位）、网格排序 organize_grid（M4）、金字塔多尺度（M6）、DDR 对接（M6）。

---

## 2. 流水线总览

```
NMS 候选流（11bit 整数坐标，来自 shi_tomasi_ctrl）
   │ s32_to_f32 ×2（整数→fp32 精确）
   ▼
candidate_store（A/B 双缓冲，64bit/地址 {y,x}）
   │ ┌─ MERGE(r=5)：读 A 写 B（phase 轮换）
   │ ├─ SUBPIXEL：直通占位（数据原地，M5 在此接入）
   │ ├─ MERGE(r=3)：读 B 写 A
   │ ├─ NEAREST：读 A → 逐点 {spacing,radius} → radius RAM
   │ └─ RING：逐点 i，按 pass0(r)&&(passA(0.75r)||passB(1.25r)) 短路
   ▼
inner 流（fp32 坐标）→ M4 organize_grid
```

主控阶段序：`S_CAP → S_MERGE5 → S_SUBPX → S_MERGE3 → S_NEAR → S_RING → S_DONE`。候选 <40 或 >12000 直接失败（status 10/11），与 `detect_native` 的范围检查一致。

---

## 3. 模块详解

### 3.1 fp32 基础件（rtl/arithmetic/，M3 新增三个）

| 模块 | 原理 | 关键决策 |
|---|---|---|
| `fp32_div.v` | 恢复余数除法 | N 归一化（N<B）+ 28 次循环；`ageb` 分支 mant/g/r；RNE=`g && (r||sticky||mant_lsb)`。C++ 45 万样本先验证再落 RTL |
| `fp32_sqrt.v` | 捷径：fp32→fp64 提升 → fp64_sqrt → f64_to_f32 | 实测 `sqrtf == (float)sqrt((double)x)`（40 万样本 0 差），提升精确（尾数补 29 位零、exp+896） |
| `fp32_hypot.v` | **必须走 double 路径** | `std::hypot(float,float)` 内部就是 double 精确计算再转 fp32，与 fp32 sqrt 链差 1 ulp。故实现 = f32→f64 提升 → fp64_mul(a,a)/mul(b,b) → fp64_add（用 fp64_sub 翻 b 符号）→ fp64_sqrt → f64_to_f32 |

### 3.2 候选存储与去重

**`candidate_store.sv`**：A/B 两个 64bit/地址同步读双口 RAM（`{y,x}` fp32）。`phase` 选择写/读对象（0=写A读B，1=写B读A）。流式写口自增指针 + `count` 输出；点随机读口 1 拍延迟整点读出。`clr` 清两侧写指针/count。

**`candidate_merge.sv`**（逐位复刻 `merge_duplicates`）：**固定锚点**算法，不是连通域聚类、也不是到动态均值判距离——改变语义会改变后续点集。外循环 i 找未使用点，保存锚点 `points[i]`；内循环 j=i+1…N-1，`!used[j] && dist<radius` 则置 used、`sum += pts[j]`（**按 j 增序链式 fp32_add**，保证与 C++ 舍入顺序位级一致）；输出 `fp32_div(sum, s32_to_f32(n))`。距离 = `fp32_hypot(fp32_sub(ax,bx), fp32_sub(ay,by))`。used 位图 BRAM（每轮 start 清零）。读口 REQ→LATCH→FEED 三段对齐，距离结果按 j 增序保序返回（in_flight 在途计数）。

### 3.3 最近邻与圆环

**`nearest_spacing.sv`**：逐点 i 扫描 j=0..N-1（j≠i），`result = std::min(result, dist)`，初值 FLT_MAX(0x7f7fffff)；`radius = min(max(mul(spacing,0x3e6147ae),0x40800000),0x41900000)`。min 链组合比较（std::min = (b<a)?b:a，非负 → 无符号位比较）。

**`ring_check.sv`**（单半径调用，逐位复刻 `alternating_ring`/`ring_detail_`）：
- 边界：`p.x < r+1 || p.x >= w-r-1` 等（fp32 比较）→ 失败记录全 0；
- 32 采样：`lround(p.x + radius·cos(2πk/32))`，cos/sin 取 `ring_cos_sin.mem` ROM（g++ libm 位模式），坐标按 **lround（round half away from zero）** 取整；
- `smooth[k]=(v[k-1]+2v[k]+v[k+1])/4`：分子 ≤1020，/4 在 float 域精确（整数域算分子 → 精确 fp32 位）；
- lo/hi = smooth min/max；`hi-lo<20` → 失败；
- `thr=(hi+lo)·0.5f`；transition 判 `(s[k]>thr)!=(s[k-1]>thr)`；
- `opp_err = Σ|s[k]-s[k+16]|`（链式 fp32_add 增序，与 C++ 逐次舍入一致）；`limit = 32·(hi-lo)·0.28f`（并行链）；
- 判定 A：`ntrans==4 && !(opp_err>limit)` → sector_ok；判定 B：4 段长度 ∈[3,13] → pass。

**`lround_pkg.sv`**：fp32→int32 round-half-away-from-zero，整数域精确（提取 exp/mant，`intp=m24>>shift`、`frac≥2^(shift-1)` 进位），644 随机+边界向量验证。

### 3.4 主控 `candidate_filter_ctrl.sv`

- **存储复用**：候选存储单读口由 merge / nearest / RING 三方向时复用（3 选 1 多路，阶段互斥）。
- **写侧回卷**：`clr(start)` 清两侧；S_SUBPX（merge5→merge3 之间）再清一次写侧，使 merge3 结果从 A 地址 0 起写（否则残留 NMS 候选旧数据）。
- **RING 编排**：逐点子状态机 RDPT→WPT→MUL→MULW→CALL→WAIT→END；`r0` 从 radius RAM 读回，0.75/1.25 倍半径用一条 fp32_mul 分时；短路语义与 C++ 一致（`ok0 && (okA||okB)`，okA 过则不再测 1.25r）。
- **对拍探针**：每点输出 d0（基本半径一次调用的中间量）+ 综合 pass，与 `m3_*_ring.bin` 每点 7 字段逐一对应；pass 点另出 inner 流。

### 3.5 灰度读口时序（易踩坑，务必保持）

`ring_check` 的 gray 读口期望 **`rd_en=1` 的下一拍出数据**（与 `dual_port_ram` 一致）。因为 ring_check 的 `rd_en/rd_addr` 是 clocked 输出，实际数据在"发出读请求后第 2 拍边沿"可用——TB 手动实现必须严格按 `always @(posedge clk) if (rd_en) q <= mem[addr]` 模板，**多寄存一拍会整体错位**（采样灰度差一个像素，smooth/ntrans/opp_err 全乱）。集成 TB 第一版就踩了这个坑（见 §5）。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_fp32.sv` | fp32_div/sqrt/hypot 全量 70021 笔（VEC 可选 small 671 笔） | ALL PASSED |
| `tb_merge.sv` | candidate_store+merge：merge5/merge3 两轮，small/texture | ALL PASSED |
| `tb_ring.sv` | nearest_spacing+ring_check：nearest/ring，small/texture；lround 644 例 | ALL PASSED |
| `tb_filter.sv` | candidate_filter_ctrl 全链：merge5/merge3（内部 RAM 层次引用）、radius、ring 探针、inner | ALL PASSED |

向量导出工具（`tests/rtl/`，g++ 编译）：`export_m3.cpp`（无 subpixel 变体逐行复刻，输出 `m3_<scene>_{in,merge5,merge3,nearest,ring,inner,gray}.bin`）、`gen_fp32_vectors.cpp`、`ring_tables.cpp`、`gen_lround_vectors.cpp`。

---

## 5. 过程中发现并修复的问题

1. **tb_fp32 加载越界**：`for(i=0;i<N*4;++i)` 少填最后一个 word（`word[4N]` 是末组 exp）→ `div #206 exp=0xXXXXXXXX`。改 `i<=N*4` 后全过（非模块 bug）。
2. **fp32_hypot 1 ulp 差**：`hypotf` 内部走 double，与 fp32 sqrt 链差 1 ulp → 重写为 fp64 链（见 §3.1）。
3. **集成 TB 灰度读口多寄存一拍**：整体错位 1 像素 → 按 dual_port_ram 模板重写（§3.5）。
4. **store 写指针不回卷**：merge3 结果写进 A 的地址 140 起（残留 NMS 数据）→ S_SUBPX 加 clr。
5. **RG_END 停留导致重复 probe**：finish 拍 rg_sub 未推进 → 多余一个探针 → 改为总是推进。
6. **候选/探针系列 bug**（子代理阶段解决）：读口时序三段式、res_count 跨轮累计、clr 只清当前侧等。

---

## 6. 遗留事项与接口契约（交接重点）

- **subpixel 在 M5**：`candidate_filter_ctrl` 的 `S_SUBPX` 是直通占位（数据原地）。M5 接入后阶段序变为 `MERGE(5) → SUBPIXEL(7) → MERGE(3)`，届时需用**带 subpixel 的 C++ 变体**重新全链对拍。
- **M4 输入**：inner 流（fp32 坐标）+ inner_total 即 M4 organize_grid 的输入。
- **候选上限**：`MAX_CAND=12000`、`MIN_CAND=40`（参数化）。超限/不足时 status 11/10。
- **未提交内容**：本报告所涉全部 RTL/TB/向量导出（git 未跟踪列表见 `git status`）。M2 遗留项（response_store_max 帧复用清零、vision_defs.vh 冻结）仍在 M6 处理。
