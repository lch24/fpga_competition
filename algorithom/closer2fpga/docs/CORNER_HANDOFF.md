# 角点检测子系统（corner 分支）交付交接说明

> 交付人：苏晨（corner 分支）
> 版本：v1.0，2026-10-01
> 用途：向队友（刘承昊：相机参数算法；强文韬：采集、DDR 与校正）交接
> 本模块已完成的功能、输入/输出契约及对接接口。模块级接口以各 `.v/.sv` 文件头部注释为权威。

---

## 1. 我完成的工作总览（M1–M8）

在 `algorithom/closer2fpga/` 下独立完成了**棋盘角点检测全链路 RTL**，与 C++ 参考实现（`algo/chessboard`、`algo/shi_tomasi`、`algo/subpixel` 等）逐位对拍一致。里程碑如下：

| 里程碑 | 交付内容 | 验证结论 |
|---|---|---|
| M1 | 公共库（sync_fifo/dual_port_ram/reset_sync）、DDR 行为模型（ddr_memory_model，11 项自检）、向量导出工具链 | 全部通过 |
| M2 | 浮点算术库（fp32/fp64 加减乘除、sqrt、hypot、log）、min_eigen 张量核、两遍检测控制 | eigen 4034 例、小图全链位级一致 |
| M3 | 候选后处理：fp32 除/根/范数、merge5/merge3、nearest、ring | 70021 例、small/texture 全链一致 |
| M4 | 网格排序：index_sort 归并核、fp32 log_ref、grid_validate、90° 网格排序 | 单元 9/9、log 4010/4010、board5x8 80→40 全链一致 |
| M5 | 亚像素精化：fp64 加减除、bilinear_core、tensor_solve、subpixel 累加与控制 | 510/407/8/603 例、board5x8 320→40 全链一致 |
| M6 | 金字塔多尺度（downsample2x/pyramid_ctrl）+ 帧级 detect_ctrl | pyramid 3 场景、big(pyramid)/board5x8(native) 40/40 |
| M7 | DDR 桥：raster_dma、byte_packer、ddr_port_adapter、gray_fetch（DDR→片上灰度） | 31/31+单元、4 例、集成 40/40 err=0 |
| M7.3 | 响应图写 DDR：resp_ddr_writer + detect_ctrl ST_DUMP 帧级导出 | 4 例、读回 921600B/104448B err=0 |
| M8 | **DDR 总线整合**：ddr_port_arbiter（2 读 2 写 round-robin）、frame_task_ctrl（GF→PYR→DET→RESP）、corner_detect_ddr_top 联调顶层 | tb_arbiter 7 例、tb_frame_top 4 帧 40/40、err=0 proto=0 |
| M8.1 | 连续帧 bug 根治：grid_order S_DONE 再武装、filter 标志清零 | tb_frame_b5x2 双帧 + tb_frame_top 4 帧全过 |

**当前状态**：RTL 完成并通过仿真验证，接口契约已冻结，等待联调。

---

## 2. 模块在系统中的位置与功能

### 2.1 系统链路（README.md 三人分工）

```
 摄像头 → DDR ──(强文韬：采集/DDR 服务/校正)──→ 显示
                │
                ├─(我) 角点检测：DDR 灰度 → 40 有序角点 + 响应图
                │
                ├─(刘承昊) 相机参数算法：40 点 → 九参数标定
                │
                └─(强文韬) 参数存储/建表/插值/校正写回
```

### 2.2 我的模块功能

**输入**：DDR 中的 L0 灰度图（Gray8，逐行光栅，32 位字节地址 + 行跨度）
**处理**：灰度取入片上 RAM →（可选）金字塔缩图 → Shi-Tomasi 角点响应 → NMS 候选 → merge/nearest/ring 筛选 → 网格排序 → 亚像素精化 → 有序 40 点
**输出**：
- 40 个有序角点流（`out_x/out_y` FP32，原图分辨率，行列序 `row*8+col`）
- 成功/失败状态（`status`、`out_grid_ok`）
- 响应图（可选，写回 DDR，供调试/下游）

### 2.3 顶层：`corner_detect_ddr_top.v`

`rtl/detect/corner_detect_ddr_top.v` 是唯一对外交付顶层，把 M7 各 DMA 客户端收敛为**单组 DDR 端口 + 帧级一键接口**。内部结构：

```
corner_detect_ddr_top
 ├─ frame_task_ctrl   帧级状态机（GF→PYR→DET→RESP，连续帧复用）
 ├─ ddr_port_arbiter  读写独立 round-robin 仲裁（2 读 2 写，在途单笔）
 │   读 0=gray_fetch  读 1=ext_rd（外部读）
 │   写 0=resp_ddr_writer  写 1=ext_wr（外部写）
 ├─ gray_fetch        DDR 灰度 → 片上 gray RAM
 ├─ pyramid_ctrl      缩图（cfg_pyr_en=1 时）
 ├─ detect_ctrl       检测全链 + 40 点输出 + 响应图导出
 ├─ resp_ddr_writer   响应字流 → DDR
 └─ gray RAM          片上灰度（reg 数组，registered 读 1 拍）
```

---

## 3. 对外接口契约（对接点）

### 3.1 帧命令与状态（上层 → 我）

| 信号 | 方向 | 语义 |
|---|---|---|
| `process_frame` | 输入 | busy=0 时单拍脉冲，启动一帧（SV 关键字 `process` 为 ModelSim 保留字，端口名为 `process_frame`，契约语义即 process） |
| `busy` | 输出 | 电平保持，帧处理期间为 1 |
| `done` | 输出 | 电平保持，帧完成拉高；下次 process 清零 |
| `status[1:0]` | 输出 | `01`=成功（有/无角点均正常）、`10`=子模块错误 |

### 3.2 帧级配置（process 前稳定，帧期间不变）

| 信号 | 语义 |
|---|---|
| `cfg_gray_base[31:0]` | DDR 灰度区首字节地址 |
| `cfg_gray_stride[31:0]` | DDR 灰度行跨度（字节） |
| `cfg_gray_w/h[15:0]` | 灰度图宽/高 |
| `cfg_ram_base` | 片上 gray RAM 基址（内部用，联调可不关心） |
| `cfg_resp_base[31:0]` | 响应图写 DDR 首字节地址 |
| `cfg_resp_dump_en` | 1=帧级导出响应图到 DDR |
| `cfg_pyr_en` | 1=金字塔路径（DEPTH=2）、0=native 路径（DEPTH=1） |

### 3.3 40 点角点流（我 → 刘承昊标定输入）★核心对接

标准 valid/ready 逐点握手：

| 信号 | 方向 | 语义 |
|---|---|---|
| `out_valid` | 输出 | 1 拍有效表示 out_x/out_y 有效 |
| `out_ready` | 输入 | 1 表示接收方就绪 |
| `out_x[31:0]` | 输出 | 角点 x（IEEE754 FP32，原图像素） |
| `out_y[31:0]` | 输出 | 角点 y（IEEE754 FP32，原图像素） |
| `out_total[15:0]` | 输出 | 总点数=40，帧内保持 |
| `out_grid_ok` | 输出 | 1=网格验证通过（帧内保持） |

**时序（ST_OUT/O_EMIT）**：`out_valid=1` 且数据就绪 → 等 `out_ready`；`valid&&ready` 同拍接受该点，下一拍出下一点；出满 40 点帧结束（`cfg_resp_dump_en=1` 时先导出响应图再 done）。`out_grid_ok=0` 的帧角点**不可用于标定**，应作废该视图。

> 与 README.md 2.2 节约定一致：`point_index = row*8 + col`（0…39），x/y 已恢复原图分辨率。

### 3.4 DDR 端口（我 ↔ 强文韬 DDR 服务层）★核心对接

顶层对外是**单组 `m_*` DDR 端口**，方向同 `ddr_memory_model`。**强文韬的真实 DDR 控制器就绪后直接替换模型即可，客户端侧接口不变**。协议：

| 通道 | 载荷（valid/ready） |
|---|---|
| 读请求 | `addr[31:0], len_bytes[31:0], tag[15:0]` |
| 读返回 | `data[31:0], keep[3:0], tag[15:0], last, error` |
| 写请求 | `addr[31:0], len_bytes[31:0], tag[15:0]` |
| 写数据 | `data[31:0], keep[3:0], last`（请求后发送，不交织） |
| 写完成 | `tag[15:0], error` |

- 32 位字节地址/长度；非尾事务 keep=4'b1111；首字节在 `data[7:0]` 低位对齐。
- 每客户端首版最多 1 读 1 写在途；独立客户端口由仲裁器处理（读 0=gray_fetch、读 1=ext_rd、写 0=resp_ddr_writer、写 1=ext_wr）。
- `ext_rd_*`（外部读）：下游读回响应图用；`ext_wr_*`（外部写）：上游预载灰度用（强文韬摄像头→灰度通路可走此口）。

### 3.5 DDR 地址区划（联调前必须确认）

corner 侧仿真验证已使用的地址（1280×720 主场景）：

| 区 | 基址 | 大小 |
|---|---|---|
| 灰度输入 | `0x0000_1000`（stride=1280） | 921,600 B（0xE1000） |
| 响应图 | `0x0030_0000`（可帧级重锁存） | 640×360×4 = 921,600 B |

**联调提醒**：强文韬的 remap 需访问的灰度/原图区、映射表区、校正输出区，基址**必须避开**灰度区（`0x1000+0xE1000`）与响应图区（`0x300000+0xE1000`），建议正式区划以顶层分配为准。

---

## 4. 验证情况（交付即证据）

| 验证项 | 结果 |
|---|---|
| 浮点/单元回归（M2–M5） | 70021+4010+510 等全部 PASS，与 C++ 位级一致 |
| tb_frame_top（M8） | big 帧 A/B/C + board5x8 帧 D，各 40/40，响应读回 921600B×2+104448B err=0，协议违规 0 |
| tb_frame_b5x2（M8.1） | 连续双帧 40/40，验证连续帧复用 |
| 向量一致性 | m6_* 权威向量 SHA256 未变，resp_tap 证明 RTL 响应计算 100% 正确 |
| 仿真串行纪律 | vsim 单 license，仿真须串行执行 |

复现方式（ModelSim 10.6e）：
```bash
vlib work
vlog -sv -work work rtl/.../*.v rtl/.../*.sv sim/*.sv sim/ddr_memory_model.sv
vsim -c tb_frame_top -do "run -all; quit -f"
```

---

## 5. 交接要点与待办

1. **给刘承昊**：40 点流接口（§3.3）已冻结，可直接按此开发标定输入；`out_grid_ok=0` 帧作废。
2. **给强文韬**：`m_*` DDR 端口（§3.4）是真实控制器替换点；`ext_wr_*` 可接收摄像头灰度；响应图区地址已占用，remap 区划需避开（§3.5）。
3. **已知限制**：
   - 板型参数硬约束：`ROWS=5/COLS=8`（6×9 方格板内角点）；换板需改参数并重新导出向量。
   - 整板必须完整入画且内角点 ≥40，否则安全失败不输出错误网格。
   - fp32 弹性模块罕见背压死锁（M8.1 发现）：当前用帧级软复位兜底，根治需波形级深挖（见 M8_REPORT §5/§6）。

## 6. 参考文件

- 顶层：`rtl/detect/corner_detect_ddr_top.v`、`rtl/detect/frame_task_ctrl.v`
- 检测链：`rtl/detect/detect_ctrl.sv`、`candidate_filter_ctrl.sv`、`grid_order_ctrl.sv`、`subpixel_ctrl.sv` 等
- DDR：`rtl/mem/ddr_port_arbiter.v`、`gray_fetch.v`、`resp_ddr_writer.v`、`byte_packer.v`、`raster_dma.v`
- 模型：`sim/ddr_memory_model.sv`
- 验证：`sim/tb_frame_top.sv`、`sim/tb_frame_b5x2.sv`、`sim/tb_arbiter.sv`
- 报告：`docs/M1_REPORT.md` … `docs/M8_REPORT.md`、`docs/RTL_GUIDE.md`、`docs/VERILOG_DESIGN_PLAN.md`
