# 板上按键操作与任务流程

板级入口为 `rtl/top/calibrated_view_top.v`。默认使用摄像头 2、1280×720 RGB565，算法运行在 40 MHz，DDR 控制域运行在 125 MHz。上电后等待操作，不再自动拍照或自动启动标定。

## 操作顺序

1. 调整棋盘位置，分别按 KEY0、KEY1、KEY2，保存三种姿态的照片。拍摄顺序不限，每张拍摄成功后立即全屏显示该照片。
2. 再次按已经拍摄成功的按钮，只显示对应原图，不重新拍摄。拍摄失败的槽位仍为空，再按该按钮可重试。
3. 三张照片均成功后，按 KEY3，依次检测三张图的角点，再计算初值、运行 LM 和检查标定结果。标定成功后保存参数，恢复显示所选原图。
4. 用 KEY0～KEY2 选定原图，按 KEY4，使用已保存参数生成映射表、校正该图并写回 DDR，成功后全屏显示校正图。再按照片按钮可返回原图，选择另一张后按 KEY4 可再次校正。
5. 重新采集一组照片需要全局复位。照片槽位的有效标记、参数和结果有效标记随复位清空；无需把 DDR 内容逐字节清零。

执行拍摄、检测、标定或去畸变期间，屏幕暂时为黑色，LED4 亮；此时其他用户按键不排队。任务结束后恢复显示。操作失败保留已有照片；去畸变失败保留标定参数并恢复所选原图。再次标定会使旧标定参数和旧校正结果失效。

按键经过两级同步和约 20 ms 消抖，长按只触发一次。多个按钮同时触发时，KEY0～KEY4 按编号从小到大优先。

## 按键与 LED

引脚依据硬件手册印刷页 46～47（PDF 第 50～51 页）：按键低有效，LED 高有效。约束已写入 `OV5640_DualView_100H/DualView_OV5640.fdc`。

| 按键 | FPGA 引脚 | 功能 |
|---|---|---|
| KEY0 | C22 | 拍摄或显示第 1 张原图 |
| KEY1 | C23 | 拍摄或显示第 2 张原图 |
| KEY2 | B22 | 拍摄或显示第 3 张原图 |
| KEY3 | A22 | 开始角点检测与标定，需要三张照片均有效 |
| KEY4 | B20 | 校正当前选中的原图，需要标定成功 |
| 硬件 RESET | 沿用板上硬件连接 | 保留为全局复位用途 |

手册确认有 5 个用户键和 1 个硬件复位键，但没有给出硬件 RESET 的普通 FPGA IO 映射，因此没有新增虚构的第六个 IO 约束。硬件 RESET 是否会重新加载配置、是否依赖 Flash 中的程序，仍需按实际板卡电路或上板实验确认；本次仿真验证的是 RTL 复位后的状态清除。

| LED | FPGA 引脚 | 亮灯含义 |
|---|---|---|
| LED0 | A20 | DDR 初始化后的心跳，持续闪烁 |
| LED1 | C18 | 第 1 张照片保存成功 |
| LED2 | C19 | 第 2 张照片保存成功 |
| LED3 | E18 | 第 3 张照片保存成功 |
| LED4 | A17 | 正在切换、拍摄或执行算法，暂不接受其他操作 |
| LED5 | A18 | 当前有有效标定参数 |
| LED6 | C17 | 最近一次去畸变成功，DDR 中有有效校正结果 |
| LED7 | B17 | 操作/算法失败，或显示通路报错 |

LED6 表示结果有效，不表示当前屏幕一定在显示校正图。LED7 的操作错误在下一次有效操作时清除；显示错误在停显、重新启显时重新统计。LED0 不代表摄像头配置一定完成，摄像头未就绪时按拍照键会被拒绝。

## DDR 中保存什么

三张原图长期保留在 DDR，片上不新增整帧缓存。每张有效数据为 1,843,200 字节，每个槽位预留 2 MiB。

| 内容 | 字节起始地址 |
|---|---|
| 原图 1 | `0x00000000` |
| 原图 2 | `0x00200000` |
| 原图 3 | `0x00400000` |
| 最新校正图 | `0x01000000` |
| X / Y 映射表 | `0x02000000` / `0x02800000` |
| 灰度金字塔 | `0x03000000` |
| 候选点工作区 | `0x03200000` |

三张图依次检测，共用一份灰度金字塔和候选点工作区。校正结果区也只有一份，再次校正会覆盖上次结果。全部有效数据约 15.4 MiB。

## 控制与代码入口

```text
key_n → board_buttons → board_flow
                          ├─ 拍照 → board_capture_hmic → 对应原图槽位
                          ├─ 显示 → board_display_hmic → HDMI FIFO → HDMI
                          └─ 命令 / 帧地址 → vision_clock_bridge
                                               ↓ 40 MHz
                                          vision_sequence
                               KEY3：灰度 → 检测 ×3 → 标定 → 保存参数
                               KEY4：已保存参数 → 建表 → 校正所选图
```

| 模块 | 职责 |
|---|---|
| `control/system/board_buttons.v` | 异步按键同步、消抖、产生一次按下脉冲 |
| `control/system/board_flow.v` | 照片有效位、选图、任务启动、错误状态、DDR 使用权交接 |
| `control/system/vision_sequence.v` | 检测/标定与去畸变两个独立命令；保存参数及有效位 |
| `memory/ddr/vision_clock_bridge.v` | 原子跨域传递启动类型、源图地址、帧地址及结果；算法请求帧后才发送描述符 |
| `video/dma/board_capture_hmic.v` | 接收并锁存本次写入基地址，完整帧写入完成后才报告成功 |
| `video/dma/board_display_hmic.v` | 全屏读取选定地址；停止时排空已发出的 DDR 读事务 |
| `top/calibrated_view_top.v` | 接线、三个时钟域、LED、显示 FIFO 的同步释放复位 |

`board_flow` 的核心状态为：空闲 → 停显排空 → 拍摄或启动算法 → 等待结果 → 空闲。标定过程中按槽位 1、2、3 提供帧描述，逐张等待释放。释放与结果分别跨域，控制器先接收释放再接收结果，失败重试不会遗留旧帧。

显示与算法仍分时使用 DDR。停止显示后，旧读请求保持到握手并收齐返回数据，`display_idle` 才允许切换使用权。HDMI FIFO 两端同时清空、各自在本时钟域同步释放复位，新画面从帧首开始，避免旧图尾部混入新图。

板级设定 `MANUAL_CONTROL=1`。算法通用入口默认保留自动串联模式，供已有独立调用者使用；这是同一套模块的编译期选择，不会在板上额外实例化一套算法。

## 仿真入口

在仓库根目录运行，工具路径由 `MODELSIM_BIN` 指定：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
```

将 `-OnlyTest` 换成以下名称可分别重跑。本次均已通过，不运行综合或布局布线。

| 测试 | 覆盖内容 |
|---|---|
| `tb_board_buttons` | 五键消抖、长按不重复、释放抖动、复位 |
| `tb_board_flow` | 乱序拍摄、显示切换、前置条件、失败重试、参数有效位、复位 |
| `tb_board_flow_cdc` | 125/40 MHz 操作交接、地址顺序、第一张检测失败后重试、独立校正命令 |
| `tb_vision_clock_bridge` | 跨域各通道随机反压、操作类型与地址完整性 |
| `tb_vision_manual` | 真实算法调度器配协议模型，检查参数保留、多次校正、失败和重新标定 |
| `tb_board_io` | 动态拍摄地址锁存、完整/不完整帧、FIFO 溢出、DDR 返回错误 |
| `tb_board_display_switch` | 整图像素、不同地址、请求及返回阶段停显、反压、错误后恢复 |
| `tb_board_video` | 两幅完整 720p HDMI 画面，切换原图/校正图及清空 FIFO，共 1,843,200 像素，无下溢 |

板级端口连接已展开检查，PDS 源码路径和 FDC 引脚唯一性已检查。数值算法未修改；没有重跑完整数值端到端回归。DDR PHY 和实际摄像头/HDMI 电气行为仍由上板验证。


## 串口调试

板级串口为 `uart_rx`（B24）、`uart_tx`（C24），设置为 **115200、8N1**。引脚定义保存在主工程 `DualView_OV5640.fdc`，PDS 使用调试模块的相对路径。工程使用 `rtl/io/uart/debug_uart.v`，不依赖独立串口实验目录。

串口运行在 27 MHz `sys_clk` 域，以配置后的上电计数器初始化，不等待 PLL、DDR 或摄像头就绪。每秒发一次心跳，UI 状态改变后也会发送合并通知。调试命令只查询，不启动拍照、不修改参数、不复位 DDR；按键操作保持不变。

### 电脑端使用

在仓库根目录执行，将 COM5 换成实际端口。电脑端依赖 `pyserial`，可用 `python -m pip install pyserial` 安装。

```powershell
# 持续显示状态并保存 JSONL 日志
python scripts/debug/uart_monitor.py COM5 --log data/debug/board.jsonl
# 查询状态、参数和 RMS
python scripts/debug/uart_monitor.py COM5 --once status
# 查询共享程序 PC、指令计数和访存握手
python scripts/debug/uart_monitor.py COM5 --once progress
# 空闲时导出本次收集到的角点及参数
python scripts/debug/uart_monitor.py COM5 --corners data/debug/corners.json
```

输出为可读 JSON。`ui_phase` 表示按钮操作阶段，`algorithm_phase` 表示灰度化、检测、标定、建表或校正阶段；`calibration_phase` 进一步区分初值、LM、检查和发布。`view` 从 0 开始。`display_corrected` 表示显示基地址选中了校正图，须结合 `display_enabled` 判断是否正在显示；`corrected` 仅表示校正结果有效。

判断停滞时结合以下数据：

- `health_bits`：bit0 到 bit7 分别为板级 PLL 锁定、算法 PLL 锁定、DDR 初始化完成、摄像头1/2配置完成、HDMI配置完成、DDR域复位释放、算法域复位释放。
- `camera_clock_stale` 与 `camera_frame_count_mod256`：区分像素时钟停止，和时钟仍有但 VSYNC 不再出现；帧计数按 256 回绕。
- `ui_dwell_cycles`：UI 状态字保持不变的周期数，按 DDR 核心时钟计数，饱和于 32 位最大值；不是毫秒数。
- `progress.shared_program_pc` 和 `instructions`：来自板上实际共用的标定执行核心；指令计数随核心重新启动而清零。它们不是检测或数学微程序的 PC。
- `progress.lm_state` 是 LM 适配器状态；`lm_completed_outer_iterations` 在适配器读回结果后才更新，不表示执行中的实时迭代进度。`last_accepted_steps` 保存最近诊断结果的有效更新次数。
- DDR 请求数、返回活动数、握手位及最后地址：可区分等待接受、等待返回和计算活动。核心域 HMIC 地址按接口字地址记录，算法域读地址为字节地址。
- `last_detect_count/view/flags`：最近检测完成的点数、视图和状态；flags bit[1:0] 是检测原状态码，bit2 是网格检查通过。
- `first_ui_error`、`sticky_display_error`、`sticky_underflow` 保存配置以来的错误；`last_error` 和 `last_error_job` 保存最近非零算法错误及所属标定任务。成功重试不会擦掉这些历史标记。

### 结构与数据有效性

| 文件 | 职责 |
|---|---|
| `rtl/io/uart/debug_uart.v` | 锁存发送字节，接收同步、起始位/停止位校验 |
| `rtl/debug/board_debug.v` | 独立上电、轮询、心跳、命令解析、封包、CRC |
| `rtl/debug/debug_snapshot.v` | 带页选择参数的请求/应答跨域快照 |
| `rtl/debug/core_debug.v` | 旁路观察 UI、DDR 事务和显示错误 |
| `rtl/debug/algorithm_debug.v` | 保存角点、参数、误差、检测结果和错误历史 |
| `scripts/debug/uart_monitor.py` | 协议解码、查询、日志与角点导出 |

源域先锁存总线并保持到应答完成，页选择参数在握手内也保持稳定，不逐位同步状态总线。工作模块的复位不清调试记录。新标定清本次参数/误差有效位和角点计数，去畸变不覆盖标定结果。查看保存结果必须结合 `params_valid`、`metrics_valid` 和任务号。

角点使用同步读 RAM，容量跟随 `PAR_TOTAL_POINTS`；默认 120 点、每点两个 FP32，共 960 字节。新标定只清有效计数，不清整块 RAM，不额外缓存图像。完整成功收集时，扁平索引为 `view * PAR_POINTS + point`；检测失败时只包含实际收到的点，不能当作完整有效棋盘。

DDR域与算法域分别采样，不保证属于同一物理时刻。每个快照都有年龄和首次采样标志，100 ms 未更新即标为 stale。源时钟停止后继续报告旧值及年龄；查询最多等待约 20 ms 后封包，发送一包约 11.3 ms。源域不回应不会堵住串口。页号不匹配时 `page_matches=false`，不能把旧页解释为新查询结果。

串口不反压功能接口。快速 UI 变化可合并为最新快照，通过 `ui_events` 观察变化次数；这不是完整事件轨迹。只有一个待处理查询槽，拥堵时增加 `rejected_queries`，电脑脚本有有界重试。接收帧错误、校验错误、不完整命令超时增加 `rx_errors`。计数器除另有说明外按位宽回绕。

掉电、未完成 FPGA 配置、基础 `sys_clk` 停止或 USB 串口链路断开时无法发送，电脑脚本超过 3 秒无心跳会提示检查链路。监测超时仅用于诊断，不自动取消 DDR 事务或强制切换控制流。

### 二进制协议

查询为 6 字节：`A5 5A op lo hi xor`，最后一字节为 `op ^ lo ^ hi`。`53` 查询状态/参数，`44` 查询执行进度，`43` 查询角点，索引为小端 `lo/hi`。不完整命令约 20 ms 后丢弃，后续帧可以重新同步。

响应固定 130 字节：`A5 5A 01 op seqLE16 78 00 payload[120] crcLE16`。序号是发送包序号；CRC16-CCITT 初值 FFFF、多项式 1021，覆盖版本至载荷末尾，不含 A5 5A 和 CRC。整数和浮点原始位模式均为小端。

| 载荷偏移 | 字节数 | 内容 |
|---|---:|---|
| 0 | 4 | 上电毫秒计数 |
| 4 | 4 | 快照过期/有效标志、硬件健康位、摄像头帧计数 |
| 8 / 12 | 各4 | DDR / 算法快照年龄，ms |
| 16 / 20 | 各4 | 接收错误 / 查询拒绝计数 |
| 24 | 32 | UI 状态、变化/访存计数、历史错误 |
| 56 | 64 | 算法快照，内部最后两字节为实际页号 |

算法页号 FFFF 为参数页，FFFE 为进度页，其余为角点索引。精确位布局见 `scripts/debug/uart_monitor.py::decode`，RTL 对应 `algorithm_debug.v` / `core_debug.v`，无需手工解释浮点数。

### 验证

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_debug
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -OnlyTest tb_debug_snapshot
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -OnlyTest tb_core_debug
```

`tb_board_debug` 按真实 UART 位时间收发，覆盖未初始化、参数/角点/进度读取、功能复位、源时钟停止与恢复、心跳、错误/不完整命令、break 和查询拥堵。其输出字节再由电脑端解码器交叉验证。`tb_debug_snapshot` 检查持续变化总线的一致性及停钟/恢复；`tb_core_debug` 检查错误保留、忙时按键和事务计数。原有标定控制回归 40 个用例也已通过。

RTL 仿真、主机解码及顶层连接检查已通过。2026-10-12 的用户综合和布线后报告已确认包含串口模块：LUT 59,023/66,600，算法 40 MHz 域余量 +0.853 ns，串口所在 sys_clk 域内部余量 +29.034 ns。

整机尚未实现时序收敛：调试健康位首级同步入口有 −0.340/−1.761 ns 违例，DDR 数据路径最差 −0.222 ns，DDR PHY 复位恢复时间最差 −4.148 ns。后续需核对同步器首级约束和厂商 DDR 复位约束，不能把这些路径统一忽略。实体串口通信仍需上板确认。
