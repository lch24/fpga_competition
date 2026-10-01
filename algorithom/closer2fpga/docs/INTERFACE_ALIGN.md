# 角点检测 ↔ 标定/去畸变 联调接口对齐说明

> 本文件由角点检测组（corner 分支，M8 完成）提供给标定/去畸变组（undistort 分支），
> 供联调及队友 agent 按说明 + 已有代码推进工作。版本 v0.1，2026-10-01。

## 1. 联调背景

- **corner 分支**：已完成 M1–M8 的角点检测 RTL，交付 `corner_detect_ddr_top.v`
  （帧级一键处理：DDR 灰度输入 → 金字塔/检测 → 40 角点流 + 响应图写回 DDR）。
- **undistort 分支**：已交付 `camera_param_store.v`（九参数 shadow/active 双 bank
  存储，TB 已验证 PASS）、`biaoding.v`（标定顶层骨架，角点输入接口待定义）、
  `check_fixed_point.py`（Brown 逆映射定点化模型）。

联调目标是打通：**角点检测 40 点流 → 标定（biaoding）→ 九参数 → camera_param_store
→ remap（逆映射 + 插值）→ 输出**。本文件聚焦前三段的接口契约与 DDR 地址区划。

## 2. 帧命令与状态接口（corner_detect_ddr_top 顶层）

| 信号 | 方向 | 语义 |
|---|---|---|
| `process_frame` | 输入 | busy=0 时单拍脉冲，启动一帧（注意：SV 关键字 `process` 为 ModelSim 保留字，端口名为 `process_frame`，契约语义即 process） |
| `busy` | 输出 | 电平保持，帧处理期间为 1 |
| `done` | 输出 | 电平保持，帧完成拉高；下次 process 清零 |
| `status[1:0]` | 输出 | `01`=成功（有角点/无角点均正常）、`10`=子模块错误 |
| `cfg_gray_base[31:0]` | 输入 | DDR 灰度区首字节地址 |
| `cfg_gray_stride[31:0]` | 输入 | DDR 灰度行跨度（字节） |
| `cfg_gray_w/h[15:0]` | 输入 | 灰度图宽/高 |
| `cfg_ram_base` | 输入 | 片上 gray RAM 基址（内部用，联调不需关心） |
| `cfg_resp_base[31:0]` | 输入 | 响应图写 DDR 首字节地址 |
| `cfg_resp_dump_en` | 输入 | 1=帧级导出响应图到 DDR（默认联调置 1） |
| `cfg_pyr_en` | 输入 | 1=金字塔缩图路径（DEPTH=2），0=native 路径（DEPTH=1） |

帧期间所有 cfg 必须保持稳定；帧级软复位保证连续帧复用。

## 3. 40 点角点流接口（corner → biaoding 标定输入）

`corner_detect_ddr_top` 的角点输出是标准 valid/ready 逐点握手流，**这是标定模块
（biaoding.v）当前"暂略"的输入接口，请按此定义补齐**：

| 信号 | 方向 | 语义 |
|---|---|---|
| `out_valid` | corner 输出 | 1 拍有效表示当前 out_x/out_y 有效 |
| `out_ready` | biaoding 输出 | 1 表示标定侧就绪可接收 |
| `out_x[31:0]` | corner 输出 | 角点 x（fp32 位模式，行列序） |
| `out_y[31:0]` | corner 输出 | 角点 y（fp32 位模式，行列序） |
| `out_total[15:0]` | corner 输出 | 总点数（40），帧内保持 |
| `out_grid_ok` | corner 输出 | 1=网格验证通过（帧完成时锁存，帧内保持） |

**时序（ST_OUT 阶段 O_EMIT 状态）**：
1. `out_valid=1` 且 out_x/out_y 就绪 → 等待 `out_ready`。
2. `out_valid=1 && out_ready=1` 同拍 → 该点被接受，下一拍出下一点（或结束）。
3. 输出 `out_total`（=40）个点后帧结束（若 `cfg_resp_dump_en=1` 则进入 ST_DUMP
   导出响应图，之后才 `done`）。
4. `out_grid_ok=0` 时角点流**不可用于标定**（标定应忽略该帧，等下一帧）。

**biaoding.v 需新增的输入声明**（参考形式）：

```systemverilog
// 40 点角点流（corner 检测输出 → 标定输入）
input  wire               cp_valid,     // 角点有效
output wire               cp_ready,     // 标定可接收
input  wire [31:0]        cp_x,         // fp32 x
input  wire [31:0]        cp_y,         // fp32 y
input  wire [15:0]        cp_total,     // =40
input  wire               cp_grid_ok,   // 帧网格有效
```

建议在 biaoding.v 顶层新增以上端口并先回环直通（收集 40 点存 RAM），后续再接标定算法。

## 4. DDR 地址区划（冲突评估 + 约定）

### 4.1 现状（corner 侧仿真验证已使用的地址）

| 场景 | 区 | 基址 | 大小 |
|---|---|---|---|
| big（1280×720, DEPTH=2） | 灰度输入 | `0x0000_1000`（stride=1280） | 921,600 B（0xE1000） |
| big | 响应图 | `0x0030_0000`（帧A）/ `0x0031_0000`（帧B） | 640×360×4 = 921,600 B |
| board5x8（272×96, DEPTH=1） | 灰度输入 | `0x0020_0000`（stride=272） | 26,112 B |
| board5x8 | 响应图 | `0x0040_0000` | 272×96×4 = 104,448 B |

### 4.2 冲突风险评估

- **当前无冲突**：undistort 侧 `camera_param_store` 纯寄存器存储、不访问 DDR；
  `biaoding.v` 为骨架；remap 的 DDR 访问尚未实现。
- **潜在冲突（联调期）**：未来 remap 需要读源图/灰度做插值、写映射表和校正输出，
  若与 corner 的灰度区、响应图区重叠会互相踩数据。**必须先约定区划再实现 remap**。

### 4.3 建议区划（1280×720 主场景，DDR 容量远大于所需）

| 起始地址 | 大小 | 用途 | 所有者 |
|---|---|---|---|
| `0x0000_1000` | 0xE1000 (921,600 B) | L0 灰度输入（gray_fetch 读） | corner 读 / 上游写 |
| `0x0020_0000` | 0x19800 (104,448 B) | board5x8 灰度（测试场景可复用） | corner 读 |
| `0x0030_0000` | 0xE1000 (921,600 B) | 响应图写区（ST_DUMP 导出） | corner 写 |
| `0x0040_0000` | 0x40000 (262,144 B) | 标定/映射表工作区（预留） | undistort |
| `0x0080_0000` | 按需 | remap 校正输出/映射表 | undistort |

> 说明：以上为**建议草案**，最终以双方确认的地址分配表为准；任何 remap DDR 访问
> 基址必须避开 `0x0000_1000+0xE1000`（灰度区）与 `0x0030_0000+0xE1000`（响应图区）。

## 5. DDR 端口契约（对接真实控制器的替换点）

`corner_detect_ddr_top` 对外是**单组 m_* DDR 端口**（方向同 `ddr_memory_model`），
真实控制器就绪后直接替换模型即可；另有 `ext_rd_*`（外部读，下标 1）与
`ext_wr_*`（外部写，下标 1）客户端：

- 读客户端 0 = gray_fetch（内部）；1 = ext_rd（下游读回响应图）。
- 写客户端 0 = resp_ddr_writer（内部）；1 = ext_wr（上游预载灰度）。
- 协议：32 位地址、长度字节；读 `req(valid/ready/addr/len/tag) → ret(valid/keep/tag/last/error)`；
  写 `req → dat(valid/keep/last) → cplt(tag/error)`；非尾事务 keep=4'b1111。

## 6. 下一步工作清单（undistort 组）

1. **biaoding.v**：按 §3 补齐 40 点角点流输入端口，先做"收集 40 点 → RAM 暂存"，
   `cp_grid_ok=1` 且收满 40 点才触发标定流程。
2. **标定结果输出**：按已有 `INTERFACE.md` 保持 `param_valid/param_ready` 九参数交易
   （fx/fy/cx/cy Q16.16；k1..k3 Q4.28；calib_width/height/id；rms_error），
   接到 `camera_param_store` 的 s_* 输入（该模块已实现且 TB 通过，直接例化）。
3. **DDR 区划**：实现 remap 前先与 corner 组确认 §4.3 地址分配表，避免与灰度区
   （0x1000..0xE2000）和响应图区（0x300000..0x3E1000）重叠。
4. **验证**：ModelSim 仿真需串行执行（license 限制）；每步跑通后提交 undistort 分支。

## 7. 参考文件

- corner 侧：`rtl/detect/corner_detect_ddr_top.v`、`rtl/detect/frame_task_ctrl.v`、
  `rtl/detect/detect_ctrl.sv`（ST_OUT 40 点流）、`sim/tb_frame_top.sv`（联调级验证）。
- undistort 侧：`rtl/camera_param_store.v`、`tb/tb_camera_param_store.v`、
  `biaoding.v`、`INTERFACE.md`、`model/check_fixed_point.py`。
