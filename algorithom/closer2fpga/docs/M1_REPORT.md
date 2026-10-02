# M1 工作报告：基础设施（目录骨架 / 公共 RTL / DDR 仿真模型 / 向量导出）

- 负责人：苏晨（corner 分支）
- 状态：已完成，提交 `2b227e9`
- 对应规划：VERILOG_DESIGN_PLAN 第 12 节建议开发顺序的第一阶段；第 11 节验证方法中"DDR 服务"与"对拍向量"两行

---

## 1. 范围与验收标准

M1 的目标是让角点检测子系统的开发**不依赖任何人**就能开始：

| 交付项 | 验收标准 | 结果 |
|---|---|---|
| 目录骨架 `rtl/ sim/ tests/rtl/` | 与第 13 节文件清单对应 | 完成 |
| RTL 公共模块（reset_sync / dual_port_ram / sync_fifo） | vlog 编译零错误零警告 | 通过 |
| DDR 服务行为模型 `ddr_memory_model.sv` | 按第 3.2 节接口契约实现五通道 | 完成 |
| 模型自测平台 `tb_memory.sv` | 读改写一致性等 11 项全部通过 | ALL TESTS PASSED |
| 对拍向量导出工具 | 3 张测试图导出成功，NMS 复刻与官方实现逐点一致 | 1193 个候选点，自校验 OK |

不在 M1 范围内：任何检测算法 RTL（M2 起）、真实 DDR 服务层（强文韬）、`vision_defs.vh`（待三人共同冻结）。

---

## 2. 目录结构

```
closer2fpga/
├── rtl/
│   └── common/              # 可综合公共模块（M1 只建了 common，M2 起按 13.2 扩展）
│       ├── reset_sync.v
│       ├── dual_port_ram.v
│       └── sync_fifo.v
├── sim/                     # 仿真专用（不可综合）
│   ├── ddr_memory_model.sv
│   └── tb_memory.sv
└── tests/
    └── rtl/                 # 向量导出工具链
        ├── export_vectors.cpp
        ├── jpg_to_bgr.ps1
        ├── run_export_gcc.cmd     # MSYS2 g++ 构建（本机）
        └── run_export.cmd         # MSVC 构建，找不到 cl 自动回退 g++
```

生成物全部落在 `tests/build/`（已被 .gitignore 忽略）：`tests/build/raw/`（裸 BGR）、`tests/build/vectors/`（对拍向量）。

---

## 3. RTL 公共模块

三个模块都是纯 Verilog-2001，每个文件头部注释即接口契约（时序语义、复位行为、推断意图），此处只列要点。

### 3.1 reset_sync.v

异步复位、同步释放。`arst_n` 拉低立即生效；释放后经 2 级同步器（消亚稳态）在 2 拍后拉高 `rst_n`。每个时钟域各例化一个，跨域复位禁止直连。

### 3.2 dual_port_ram.v

- 参数：`DATA_WIDTH`、`ADDR_WIDTH`（深度固定 2^ADDR_WIDTH，仅支持 2 的幂）
- 读写同钟；**读延迟 1 拍**；`rd_en=0` 时输出保持
- 同址同拍读写为 **read-first**（读旧值）——BRAM 推断的标准模板，调用方不得假设能读到当拍新值
- 复位只清读数据寄存器，存储阵列不复位（BRAM 物理特性）

### 3.3 sync_fifo.v

- 参数：`DATA_WIDTH`、`ADDR_WIDTH`（容量 2^ADDR_WIDTH）
- 读写同钟；`in_ready=!full`、`out_valid=!empty`，**valid&&!ready 时载荷保持**（团队契约）
- `out_data` 组合读（零延迟）；按 distributed/LUT RAM 推断，建议 ADDR_WIDTH≤6；更深需求出现时另建 BRAM 版本，不得静默替换

---

## 4. DDR 服务行为模型（sim/ddr_memory_model.sv）

### 4.1 契约实现（对应 README 第 4 节）

- 五通道：读请求 / 读返回 / 写请求 / 写数据 / 写完成，全部 valid-ready 握手
- 32 位数据宽度，地址/长度单位为**字节**，支持任意合法字节起点
- 字节在 data 中**低位在前**；尾拍 `keep` 标记有效字节
- 每客户端一笔读 + 一笔写在途（读写两通道在模型内可并行）
- `tag` 原样回带；请求 `len==0` 属协议违规（被丢弃并计数）
- 存储为稀疏字节关联数组，未写地址读出 0x00；**内容跨复位保留**（与真实 DDR 一致，算法侧不得假设复位后内存状态）

### 4.2 附加能力（用于暴露 RTL 客户端的健壮性问题）

| 能力 | 参数/手段 | 说明 |
|---|---|---|
| 随机延迟 | `LATENCY_MIN/MAX`、`JITTER_EN`、`SEED` | LFSR 伪随机，种子固定则完全可复现 |
| 随机背压 | `BACKPRESSURE_EN` | 服务端 ready 按伪随机拉低（约 1/8 概率） |
| 错误注入 | TB 层次调用 `inject_error(min,max)` / `clear_error_injection()` | 命中范围的读返回 error=1；写的完成 error=1 且**不写入内存** |
| 协议检查 | `PROTOCOL_CHECKS`，输出 `proto_violations` | 统计 len==0、keep 与长度不符、LAST 位置错误、无请求时的写数据；**写类违规置粘滞标志，随完成通道返回 error=1**，让客户端能感知自身错误；模型永不挂死 |

### 4.3 与真实服务层的关系

本模型模拟"服务层对算法侧暴露的逻辑口"，不模拟仲裁、256bit 拆装、DDR PHY 时序。强文韬的服务层就绪后，算法侧只需把本模型换成真实模块重跑同一套 testbench——接口即契约。

---

## 5. 模型自测平台（sim/tb_memory.sv）

### 5.1 测试项

| # | 测试 | # | 测试 |
|---|---|---|---|
| T1 | 4 字节对齐 256B 读写一致性 | T7 | 写错误注入：error=1 且内存不被破坏 |
| T2 | 非对齐起点 addr=5 len=7 | T8 | tag 匹配（内嵌所有事务） |
| T3 | len=5 尾拍 keep=0001 | T9 | 停顿时载荷逐拍保持（全局监视器） |
| T4 | 部分覆盖写不破坏周围数据 | T10 | 故意 early-LAST：违规+1、error=1、不死锁 |
| T5 | 1920B 大块（模拟一行 BGR，480 拍背靠背） | T11 | 违规后恢复正常服务 |
| T6 | 读错误注入 | | |

全程开启随机延迟+随机背压（服务端与客户端 ready 均随机）。通过标准：`fails==0` 且违规计数恰好 1（T10 注入的那个）。

### 5.2 握手采样方法论（对后续所有 TB 适用）

激励在 **negedge** 用阻塞赋值驱动；握手检测用 **posedge 采样寄存器**（fire 标志）。采样寄存器的 RHS 在 posedge 活动区求值，与模型判定逻辑看到同一组（更新前）信号值，因此 fire ⇔ 模型已接受。

> 反例（M1 踩过的坑）：在 negedge 读组合 `ready`——因为模型的背压 gate 在 posedge 更新，negedge 看到的 ready 与模型下一 posedge 实际采样的 ready 可能不同，产生"幻象握手"：TB 以为请求已被接受而模型没有，随后通道永久错位、违规风暴。

### 5.3 复现命令

```
cd closer2fpga/sim
vlog -sv ddr_memory_model.sv tb_memory.sv
vsim -c -suppress 3829 -do "run -all; quit -f" tb_memory
```

（3829 是关联数组读默认值的良性警告。ModelSim 10.6e 验证通过。）

---

## 6. 对拍向量导出工具链

### 6.1 设计决策：无 OpenCV 依赖

原计划用 OpenCV 读图，但工具链环境不统一（vcpkg/MSVC 在部分机器不可用；MSYS2 官方镜像慢）。改为两步：

1. `jpg_to_bgr.ps1`：.NET System.Drawing 解码 JPEG → 裸 BGR 字节（Windows 零依赖）
2. `export_vectors.cpp`：读裸 BGR，**纯数学计算**，g++ 或 cl 均可直接编译

附带收益：RTL 对拍的输入（`_bgr.bin`）与将来写入 DDR 模型的字节**同一份文件**，链路自洽。

### 6.2 数据路径与自校验

导出的中间量与 FPGA 硬件路径逐步对应：

```
BGR888 → bgr_to_gray(整数精确) → sobel_xy → Ix/Iy(f32)
→ shi_tomasi_response(win=3) → resp(f32) → rmax(帧屏障)
→ thr = rmax*0.08 → 3×3 NMS → 候选点
```

参数取自 `candidates.cpp:80`（`shi_tomasi_detect(gray, candidates, 0.08f, 3)`），与主流程一致。工具内本地复刻一份 rmax/thr/NMS 逻辑与官方 `shi_tomasi_detect` 输出**逐点对账**，不一致即报错退出——导出向量的正确性有自动保证。

### 6.3 向量文件格式（format_version=1，RTL TB 依赖此契约）

每张图 7 个文件（`testN` ∈ {0,1,2}），均为**小端裸数据**：

| 文件 | 内容 |
|---|---|
| `testN_bgr.bin` | W×H×3 字节，BGR 逐像素、行主序、无行填充 |
| `testN_gray.bin` | W×H 字节 |
| `testN_ix.bin` / `testN_iy.bin` | W×H 个 IEEE754 f32 |
| `testN_resp.bin` | W×H 个 IEEE754 f32 |
| `testN_candidates.bin` | u32 数量 N，随后 N 组 {i32 x, i32 y}（扫描序：先 y 后 x） |
| `testN_manifest.txt` | 尺寸/参数/rmax/thr（含 f32 位模式十六进制，RTL 比对浮点建议用位级比较） |

**格式变更必须递增 format_version 并更新本节。**

### 6.4 导出结果（2026-09-23）

| 图 | 尺寸 | rmax | thr | 候选点数 |
|---|---|---|---|---|
| test0 | 1280×720 | 322492.344 (0x489d778b) | 25799.387 (0x46c98ec6) | 420 |
| test1 | 1280×720 | 301168.281 (0x48930e09) | 24093.463 (0x46bc3aed) | 373 |
| test2 | 1280×720 | 404265.875 (0x48c5653c) | 32341.270 (0x46fcaa8a) | 400 |

候选点量级（数百）符合预期，验证了 0.08 阈值 + NMS 在真实图上的筛选力度，也确认了 candidate_store 需要支撑到千级容量（第 9.2 节 RAM 规划的依据成立）。

### 6.5 复现命令

```
tests\rtl\run_export_gcc.cmd     # 或 run_export.cmd（自动探测编译器）
```

一键完成：解码 JPEG → 编译 → 导出 → 自校验。生成物在 `tests/build/vectors/`。

---

## 7. 经验记录（对队友同样适用）

1. **幻象握手**（见 5.2）：TB 读组合 ready 必须与 DUT 采样时刻对齐，激励 negedge / 检测 posedge 是一种可靠范式。
2. **X 态复位漏洞**：模型中错误注入标志未复位初始化，X 值经 `!err` 条件**静默抑制了全部写通路**（表象：内存 size=0、读写数据全零）。教训：TB 可配置状态一律复位初始化；写使能相关条件中的 X 会以最隐蔽的方式失效。
3. **ModelSim 10.6e 兼容性**：关联数组元素不支持非阻塞赋值（用阻塞）；动态数组无 `push_back`（用队列）；`localparam time` 与部分 SV 语法受限。写 TB 时注意。
4. **cmd 编码**：.cmd 脚本含中文注释会因代码页（GBK）解析失败，脚本一律纯 ASCII。

---

## 8. 遗留事项（非 M1 缺陷，需团队跟进）

- `vision_defs.vh`：字段位宽、命令码、错误码仍未冻结（README 规划由三人共同确认）——**阻塞 M6 的顶层编排，不阻塞 M2-M5**，建议 M2 期间并行推进
- 真实 DDR 服务层（强文韬）就绪后：算法侧 TB 把 `ddr_memory_model` 换成真实模块重跑，接口即契约
- 大深度 FIFO（BRAM 版）按需再建，出现需求时单独验证替换

---

## 9. 下一步（M2：像素流水前端）

按第 5.1/5.2 节规划：`bgr_to_gray.v` → `window3x3.v` → `sobel_core.v` → `tensor_core.v` → `tensor_window_sum.v` → `min_eigen_core.v` → `response_store_max.v` + `nms_candidates.v`，由 `shi_tomasi_ctrl.v` 编排两遍扫描。

首个对拍对象就是本次导出的 `_gray.bin`（bgr_to_gray 整数精确比对），随后 `_ix/_iy/_resp/_candidates` 逐级对拍，64×64 合成小图先行、1280×720 全图跟进。
