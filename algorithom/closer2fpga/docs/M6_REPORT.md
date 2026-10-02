# M6 说明文档：金字塔多尺度与帧级检测调度（PYRAMID / DETECT_CTRL）

- 负责人：苏晨（corner 分支）
- 状态：已完成（待提交）
- 对应规划：VERILOG_DESIGN_PLAN 第 5.1 / 5.2 / 13.2 节
- 用途：面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M6 把 `detect_chessboard` 的多尺度部分落地：(1) 金字塔缩图基础件 `downsample2x`；(2) BGR→灰度扫描 `gray_scan`；(3) 层灰度生成 `pyramid_ctrl`（有限深度层描述符栈替代递归）；(4) 单图多尺度检测调度 `detect_ctrl`（层槽位链 + 2p+0.5 恢复编排）。输出与 C++ 权威（`export_m6.cpp` 的 `detect_chessboard_ref` 确定性变体，逐行复刻 chessboard.cpp 递归）**逐位一致**。

| 交付 | 验收结果 |
|---|---|
| `downsample2x.v`（两行缓存 + 四像素和除 4，整数域） | 9 场景 ALL PASSED（含 odd 尺寸/0 输出边界） |
| `gray_scan.v`（BGR→Gray8，(299R+587G+114B+500)/1000；C=1 直接复制） | 8 场景 ALL PASSED（含随机 BGR 权重校验 + C=1） |
| `pyramid_ctrl.sv`（层描述符栈、逐层缩图、确定性基址） | 3 场景 ALL PASSED（big/small/2000×1200 三层） |
| `detect_ctrl.sv`（DEPTH 参数化层槽位链 + 恢复编排 + thr 链） | big 金字塔路径 + board5x8 native 路径均 40/40 位级一致 |
| `grid_order_ctrl.sv`（A_ORIGIN 缺陷修复，快照读源） | M4 tb_order 回归 40/40 位级一致（无回归） |
| **集成端到端**（pyramid_ctrl → detect_ctrl） | big：L1 生成 230400 字节 0 错 → detect 40/40；board5x8 40/40；err=0 |

全链场景：
- **big（金字塔路径，1280×720）**：L0 1280×720 → pyramid 生成 L1 640×360 → detect@L1 native 40 点 → 2p+0.5 映射 → grid_refine@L0 → 40 点，与 `m6_chain_big.bin` 逐位一致。
- **board5x8（native 路径，272×96）**：max≤960 无金字塔 → detect@L0 native 40 点，与 `m6_chain_board5x8.bin`（= M5 `m5_board5x8_refined.bin`，逐字节相同）一致。

**不在 M6 范围**：DDR 读写对接（M6.3/M7）、标定（后续）、remap（后续）。`detect_ctrl` 消费的层灰度由 `pyramid_ctrl`（或外部）按确定性基址预生成。

---

## 2. 架构总览

```
pyramid_ctrl（层灰度生成）：
  L0(外部预载 base0) → 逐层 downsample2x → L1..LD，基址 base_d = base_{d-1}+W_{d-1}*H_{d-1}
  停止条件（对最新层）：W≥32 && H≥32 && max>960，否则停
  level 描述符 {base,W,H} 输出

detect_ctrl（单图检测调度，DEPTH 参数化，generate 层槽位链）：
  槽位 d（W_d=W0>>d, H_d=H0>>d）：
    gray_reader → window3x3 → sobel → tensor → tensor_window_sum → s32×3 → min_eigen
      → sync_fifo → shi_tomasi_ctrl（thr 由 detect_ctrl 共享 rmax 链计算）
      → candidate_filter_ctrl → grid_order_ctrl → corner RAM_d → grid_refine_ctrl
  C++ 递归语义（chessboard.cpp）：
    先最深层 native；逐层向上：child_valid[d+1] ? map 2p+0.5 + refine(d)
      失败则 native(d)；最终 child_valid[0] → 40 点输出 / 无角点 status=10
```

每层"native" = 完整链（shi_tomasi→merge5→subpixel7→merge3→ring→organize→refine_grid），
与 C++ `detect_native` 结尾自带 refine_grid 一致；"恢复" = 仅 grid_refine_ctrl（M5 交付）。

---

## 3. 模块详解

### 3.1 `downsample2x.v`（rtl/detect/，M6.1）

逐位复刻 `chessboard.cpp` 缩图公式：`half(x,y)=(a+b+c+d+2)>>2`（整数 floor），输出
floor(W/2)×floor(H/2)，奇数末行/末列消费但丢弃。单行缓存（reg 数组，registered 读 1 拍），
偶数行写缓存、奇数行与缓存按列对合并；每输出需 a、b 两次缓存读，用"偶→奇→气泡"3 拍
节拍解决单读口冲突；输出挂起/气泡时 in_ready=0（反压不丢数）。done 电平保持。

### 3.2 `gray_scan.v`（rtl/detect/，M6.1）

复刻 `kernels::bgr_to_gray`：`gray=(299R+587G+114B+500)/1000`，字节序 B,G,R。3 字节收集器
（前 2 字节恒收，第 3 字节喂 `bgr_to_gray` 核，核 in_ready=out_ready 组合直通）；
C=1 时 1 字节/像素 1 拍流水复制。done = 字节消费完 && 流水排空。

### 3.3 `pyramid_ctrl.sv`（rtl/detect/，M6.2）

有限深度（MAX_DEPTH）层描述符栈替代递归：S_IDLE→S_LAYER（判定缩图条件）→S_FEED
（registered 读流水喂 downsample2x，捕获输出写 base_next）。关键实现：
- 读流 1-deep 缓冲 + `rd_pend` 在途标志**保持到转移完成**（气泡拍 rd_addr/rd_data 保持，
  数据排队不丢——downsample2x 气泡反压下的经典丢数坑）；
- downsample2x done 为电平，打 2 拍检测上升沿推进；
- 写口组合输出（out_acc 拍 1 拍完成），TB 每 8 拍停 1 拍覆盖 out_ready 反驱。

### 3.4 `detect_ctrl.sv`（rtl/detect/，M6.2 核心）

DEPTH 参数化 generate 层槽位链 + 恢复编排 + thr 共享链 + corner RAM 仲裁：
- **thr**：对喂给 shi_tomasi_ctrl 的 resp 流逐拍 max（位变换无符号比较，与
  response_store_max 同算法），Pass1 收满后 fp32_mul(rmax, 0x3DA3D70A) 闩存；
- **corner RAM**（每层 CORNER_N×{x,y} fp32）：写口 3 选 1（grid_order out / grid_refine
  out / map 写入）、读口 3 选 1（refine pt_rd / map 读取 / S_OUT），阶段互斥；
- **gray_rd 汇总**：活动层 + 活动阶段 2 级选通，`gray_rd_addr = base_d + 槽位相对地址`；
- **基址**：base_d = cfg_base0 + Σ_{k<d} W_k*H_k（确定性，层灰度须预生成）。

### 3.5 `grid_order_ctrl.sv`（A_ORIGIN 缺陷修复，M6.2 触发的既有 bug）

原 A_ORIGIN 原点规范化用 `best_buf` 就地镜像，c 翻转（origin=1/3）时读源被自身 dst
覆盖，网格被破坏成回文 → refine 判无效。M4/M5 测试场景 origin=0（无需翻转）从未触发。
修复：新增 `buf_orig` 快照（等价 C++ 的 `copy` 副本），读源改快照、写目标不变。
origin=0 路径行为完全不变；M4 tb_order 回归 40/40。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_downsample.sv` | 9 场景（board5x8/big/s64/odd33x17/s127x63/s5x3/s2x2/s3x1/s1x4，含 0 输出边界） | ALL PASSED |
| `tb_gray.sv` | 8 场景（board5x8/big/copy/s2x2/rand×4，含随机 BGR 权重 + C=1） | ALL PASSED |
| `tb_pyramid.sv` | 3 场景（big L1 位级、small 无缩图、2000×1200 三层） | ALL PASSED |
| `tb_detect_full.sv` | detect_ctrl 两场景（big DEPTH=2 金字塔路径 / board5x8 DEPTH=1 native） | ALL PASSED (40/40×2) |
| `tb_detect_pyramid.sv` | **集成**：pyramid→detect 端到端 big + board5x8 | ALL PASSED (err=0) |
| `tb_order.sv`（回归） | grid_order_ctrl A_ORIGIN 修复后 M4 单元 | ALL PASSED (40/40) |

向量工具（tests/rtl/，g++ 编译）：`export_m6.cpp`（权威：downsample/bgr_to_gray 单元向量 +
`detect_chessboard_ref` 全链 + big 场景灰度）。产物（tests/build/vectors/，可重新生成）：
`m6_down_*.bin`、`m6_gray_*.bin`、`m6_big_gray.bin`、`m6_chain_{big,board5x8}.bin`。

复现：g++ 编译 export_m6 生成向量 → ModelSim vlog + vsim（单实例，授权码并发受限——**多个
vsim 并行会 license 争用失败**，务必串行：启动前 `Get-Process vsimk` 确认无实例）。

---

## 5. 过程中发现并修复的问题

1. **big 场景候选超限**：全幅 16px 棋盘在 1280×720 下 shi_tomasi 13904 候选 >12000 上限
   （约 4 响应/角点簇，正是金字塔要压制的多重响应）→ 场景改为平坦背景 + 居中 17×6 格棋盘。
2. **ring 半径=min(4) 撞半格边界**：16px 格缩半后 8px 格，ring 半径 clamp(8×0.22)=4 = 半格，
   采样落格边界被平均，4 象限交替被破坏（inner 54→28<40）→ 格改 24px（半分辨率 12px 格，
   半径 4 深入格内 2px）。
3. **organize_grid 要求点集天然恰 rows 行**：算法只保留 rows-1 个最大行间隙切 rows 组，
   6 行点集第 5/6 行被合并 → 同 x 交错 → 网格垃圾 cost=1e30。17×6 格 → 内角点 16×5=80
   （5 自然行，同 board5x8）→ 切分干净。
4. **权威向量转置 bug（主控）**：`b5(96,272)` 与 M5 约定（棋盘 272 宽×96 高）互为转置 →
   改 `b5(272,96)`；`m6_chain_board5x8.bin` 现与 `m5_board5x8_refined.bin` 逐字节一致。
5. **grid_order_ctrl A_ORIGIN 就地镜像覆盖**（D 发现并修复，见 §3.5）。
6. **pyramid_ctrl 端口位宽悬空**：`cur_w[10:0]` 直连 downsample2x `cfg_w[11:0]`（MAX_W=2560
   →ADDR_W=12），bit11 悬空 Z 导致行尾比较恒假 → 死锁；显式零扩展修复。
7. **读流气泡丢数**：rd_addr 在数据排队期间被更新冲掉（实测丢 ~20%）→ `rd_pend` 在途标志
   保持到转移完成。
8. **集成 TB 等待电平的 SV 陷阱（主控）**：任务入参 `input logic d` 为值拷贝，循环内 `d`
   冻结为初值 0，永远等不到 done → 改内联"前一负沿值判沿"循环（防漏沿/防冻结）。

**项目级教训（license）**：ModelSim 授权码并发实例数有限，所有 vsim 必须串行
（检查无 vsimk 进程再启动；license 失败等待重试）。本阶段 6 个子代理 + 主控全程遵守。

---

## 6. 遗留事项与接口契约（交接重点）

- **M6.3 DDR 对接（未做）**：detect_ctrl 的 gray_rd 目前接外部 RAM 模型（TB 内存/双口 RAM），
  真实系统接 DDR 服务层（M1 `ddr_memory_model` / §3.2 事务接口）；`gray_scan` 字节流输入
  接 DDR 读口；响应图写 DDR（`response_store_max` 现为内部 RAM）；candidate_store 容量
  12000 已满足 §5.2 上限。M6.3 过大可留 M7。
- **多实例 vs 单链复用**：detect_ctrl 按 DEPTH 用 generate 例化每层槽位链（尺寸固定），
  是"位级一致优先"的务实选择；真实硬件可后续优化为"单链 + 运行时尺寸配置"复用，
  但任何改动须重新对拍。
- **层灰度所有权**：detect_ctrl 只消费不生成；pyramid_ctrl 生成后由外部（TB/DDR 布局）
  保证落位 base_d。基址公式 `base_d = base0 + Σ_{k<d} W_k*H_k` 为两模块的冻结契约。
- **thr 计算位置**：detect_ctrl 对 resp 流并行求 rmax（与 response_store_max 同算法），
  `fp32_mul(rmax, 0x3DA3D70A)` 后闩存进 shi_tomasi_ctrl.thr，须先于 Pass2 首窗口就绪。
- **grid_order_ctrl 修复的回归范围**：origin=0 路径行为不变（M4 tb_order 40/40 回归）；
  新覆盖 origin∈{1,2,3} 的翻转路径（big/board5x8 的 A_ORIGIN 后 origin 均经检测生效）。
- **big 场景复用**：`m6_big_gray.bin`（1280×720）是 M6.2 全链权威输入；M6.3 DDR 对接
  时可直接作为 DDR 预载内容复用。
