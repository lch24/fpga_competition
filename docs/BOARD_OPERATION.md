# 板上操作与串口接口

## 操作顺序

摄像头就绪后自动预览。默认处理传感器中心 960×540 区域，显示时放大到 1280×720。

1. 按 KEY0、KEY1、KEY2 分别拍摄三张照片，顺序不限。成功后回看两秒，再返回预览。
2. 已成功的照片键再次按下只回看两秒；失败槽仍为空，可再次拍摄。重新采集整组需全局复位。
3. 三张照片齐全后按 KEY3，依次检测、求初值、LM 和结果检查。成功保存参数并点亮 LED5，结束后回到预览。
4. 按 KEY4，根据当前参数建表并进入连续校正视频；再按 KEY4 排空并停止，回到预览。
5. 校正视频中按 KEY0 切换校正前后；每次进入视频默认显示校正后。重新标定会使旧校正表失效。

拍照时先排空预览，再采集完整新帧，因此照片不是按键瞬间 HDMI 的冻结。
两秒从显示模块开始读取照片第一帧计时。拍摄、标定和建表期间黑屏，忙时不排队保存按键。
视频期间 KEY4 停止优先于 KEY0 切换，KEY1～KEY3 短按忽略；预览期间 KEY0 仍是第一张照片键。

默认 `ENABLE_DISTORT_DEMO=0`。启用该编译参数后，KEY3 短按释放开始标定，长按两秒切换人工畸变模式；
切换会清除照片、参数和校正表的有效状态。详见[人工畸变设计](DISTORT_DEMO.md)。
`REALTIME_VIDEO=0` 才是 KEY4 校正单张照片的独立模式。

## 按键与 LED

按键低有效，约 20 ms 消抖，长按只触发一次；LED 高有效。
引脚由主工程 FDC 定义，对应硬件手册印刷页 46～47。

| 按键       | FPGA 引脚        | 功能                                                |
| ---------- | ---------------- | --------------------------------------------------- |
| KEY0       | C22              | 照片阶段拍摄或显示第 1 张图；视频阶段切换校正前／后 |
| KEY1       | C23              | 拍摄或显示第 2 张原图                               |
| KEY2       | B22              | 拍摄或显示第 3 张原图                               |
| KEY3       | A22              | 开始角点检测与标定，需要三张照片均有效              |
| KEY4       | B20              | 标定成功后建表并启动实时视频；视频期间停止          |
| 硬件 RESET | 沿用板上硬件连接 | 保留为全局复位用途                                  |

硬件 RESET 沿用板卡连接。RTL 复位清除有效位与控制状态，不逐字清 DDR；
硬件按钮是否重新加载配置由板卡复位电路决定。

| LED  | FPGA 引脚 | 亮灯含义                                        |
| ---- | --------- | ----------------------------------------------- |
| LED0 | A20       | DDR 初始化后的心跳，持续闪烁                    |
| LED1 | C18       | 第 1 张照片保存成功                             |
| LED2 | C19       | 第 2 张照片保存成功                             |
| LED3 | E18       | 第 3 张照片保存成功                             |
| LED4 | A17       | 切换/拍摄/算法/视频运行中；视频时 KEY4 仍可停止 |
| LED5 | A18       | 当前有有效标定参数                              |
| LED6 | C17       | 最近一次去畸变成功，DDR 中有有效校正结果        |
| LED7 | B17       | 操作/算法失败，或显示通路报错                   |

LED6 表示结果有效，不表示当前屏幕一定在显示校正图。LED7 的操作错误在下一次有效操作时清除；显示错误在停显、重新启显时重新统计。LED0 不代表摄像头配置一定完成，摄像头未就绪时按拍照键会被拒绝。

## 控制与数据入口

`board_buttons → video_board_flow → vision_clock_bridge → vision_sequence` 是按键到算法的主路径。
`video_board_flow` 管理三照片槽、三视频原图槽和双结果槽；`vision_sequence` 管理逐图检测、标定、参数和建表。
照片在 DDR 中保留，检测逐张复用灰度和候选工作区。内存地址见[根 README](../README.md#当前-ddr-分区)，
帧交接和重映射数据通路见[实时视频](REALTIME_VIDEO.md)。

## 串口调试

TX 为 C24，RX 为 B24，**115200、8N1**。串口在独立 27 MHz 系统域运行，
每秒报告心跳，状态改变时发送合并通知；PLL、DDR 或算法未就绪时仍可查询健康状态。
查询只读取信息，不启动任务或取消 DDR 事务。

电脑端需要 `pyserial`。将示例端口替换为实际端口：

```powershell
# 持续显示状态并保存 JSONL 日志
python scripts/debug/uart_monitor.py COM5 --log data/debug/board.jsonl
# 查询状态、参数和 RMS
python scripts/debug/uart_monitor.py COM5 --once status
# 核对实际烧录版本的裁剪/处理/显示尺寸，以及显示放大器状态
python scripts/debug/uart_monitor.py COM5 --once geometry
# 查看最近采集帧的像素数、短行和跨域 token 完整性
python scripts/debug/uart_monitor.py COM5 --once capture
# 解析内参矩阵与畸变系数，并可保存成 JSON 供报告/OpenCV 对照
python scripts/debug/uart_monitor.py COM5 --once params --save data/debug/camera.json
# 查询共享程序 PC、指令计数和访存握手
python scripts/debug/uart_monitor.py COM5 --once progress
# 实时计数与上一次校正耗时；稳态下统计 10 秒真正的新帧率
python scripts/debug/uart_monitor.py COM11 --once video
python scripts/debug/uart_monitor.py COM11 --video-seconds 10
# 空闲时导出本次收集到的角点及参数
python scripts/debug/uart_monitor.py COM5 --corners data/debug/corners.json
```

输出为可读 JSON。`ui_phase=camera_preview` 表示拍照预览，`live_video` 表示 KEY4 启动的校正视频；`algorithm_phase` 表示算法阶段，`calibration_phase` 进一步区分初值、LM、检查和发布。`view` 从 0 开始。`display_corrected` 跟随已提交显示帧的校正标记，不再仅按缓冲地址判断；预览和校正前画面为 false。`corrected` 仅表示曾产生有效校正结果。

默认终端用中文显示状态变化、结果和错误，每十秒补一条心跳。
`--json` 显示完整包，`--log` 始终保存完整 JSON。

### 尺寸与采集诊断

| 字段                                             | 正常值及含义                                                               |
| ------------------------------------------------ | -------------------------------------------------------------------------- |
| `sensor_size` / `crop_origin`                | `[1280,720]` / `[160,90]`                                              |
| `processing_size` / `display_size`           | `[960,540]` / `[1280,720]`                                             |
| `expected_pixels` / `expected_capture_words` | 518,400 像素 / 32,400 个 256 位 DDR 数据字                                 |
| `rgb565_stride_bytes` / `rgb565_frame_bytes` | 1,920 / 1,036,800 字节                                                     |
| `scaler_phase`                                 | 0 填行、1 输出插值像素、2 排空末尾流水线                                   |
| `scaler_error` / `reader_error`              | 均为 false；分别对应放大器帧首尾标记异常、显示 DDR 读取异常                |
| `scaled_frames_since_display_enable`           | 送入显示桥的完整放大帧数，包含重复画面；停显复位时清零，不能当作新校正帧率 |

`capture.source_pixel_count` 和 `ddr_completed_words` 对应最近完成的采集帧，应与上表一致。`short_lines_saturated` 统计原始传感器全帧的短行，最高 63，包含裁掉的行。采集不补造缺失像素；像素总数正确还需结合帧完整性判断。`source_status`、`token_integrity_ok` 和 `corrupt_end_tokens` 区分源图像错误与跨域帧尾数据异常。结合 `--once video` 的失败帧增量、状态页的 `sticky_underflow` 判断故障位置。

`source_frame_error_bits` 的 bit0..3 对应下表，`source_frame_errors` 直接列出名称。它们随最近帧尾锁存，正常为 0 / 空数组；旧固件没有这四位诊断，读到 0 不代表已经具备新检查。

| 名称 | 含义 |
|---|---|
| `roi_short_line` | 保留区域的某行像素不足 |
| `odd_byte_count` | 保留行收到奇数字节，RGB565 配对不完整 |
| `long_line` | 保留行超过传感器配置宽度 |
| `wrong_row_count` | 原始帧的 HREF 行数与传感器高度不符 |

上述错误使当前帧不被提交为照片或预览；下一帧正常即可恢复。该检查能识别数量和边界异常，不能识别保持相同像素数的任意数据位错误。摄像头寄存器写入未收到 ACK 时会重试，`camera_configured` 保持 false。

角点与相机参数都使用裁剪坐标。`--once params` 和 `--corners` 自动查询并保存尺寸；
旧固件不支持尺寸页时，工具明确标记使用 1280×720 假设。

### 状态、进度和有效性

| 字段 | 含义 |
|---|---|
| `health_bits` | bit0..7：板级 PLL、算法 PLL、DDR 初始化、相机1配置、相机2配置、HDMI配置、DDR域复位释放、算法域复位释放 |
| `camera_clock_stale` / `camera_frame_count_mod256` | 像素时钟活性与按 256 回绕的 VSYNC 帧计数 |
| `ui_dwell_cycles` | 当前 UI 状态持续的 DDR 域周期数，32 位饱和 |
| `progress.shared_program_pc` / `instructions` | 实际共享标定程序位置与指令计数，启动阶段时重新计数 |
| `progress.lm_state` | LM 硬件适配器状态；外迭代数在结果读回后更新 |
| `last_detect_count/view/flags` | 最近检测点数、视图；flags bit[1:0] 为状态，bit2 为网格检查结果 |
| `params_valid` | 已通过检查，可正式用于去畸变 |
| `candidate_valid` / `candidate_params` | 候选已完整生成，最终失败时也可查看；不发布给校正模块 |
| `metrics_valid` / `converged` | 误差统计有效、算法收敛；结合任务号解释结果 |
| `first_ui_error` / `last_error_job` | 首次操作错误、最近算法错误所属任务 |
| `sticky_display_error` / `sticky_underflow` | 历史显示错误，不等于当前帧正在出错 |

源域通过请求应答锁存整份快照，数据和页号保持到应答；没有逐位同步多位状态总线。
DDR 和算法快照不是同一瞬间，分别携带年龄。超过 100 ms 未更新会标 stale，
`page_matches=false` 时不解释为所请求的新页。查询约 20 ms 超时后仍可报告旧值及年龄。
调试不反压功能接口；只有一个待查询槽，拥堵增加 `rejected_queries`，协议错误增加 `rx_errors`。

新标定清本次结果有效位和角点计数；去畸变保留标定诊断。
角点按同步 RAM 保存，完整索引为 `view * PAR_POINTS + point`。
检测失败只导出实际收到的点，不能将其视为完整棋盘。

### 参数与失败原因

板上发布顺序为 `fx,fy,cx,cy,k1,k2,k3,p1,p2`，焦距和主点单位为处理图像素。
`--once params` 输出 K 矩阵及 OpenCV 顺序的 `k1,k2,p1,p2,k3`，`--save` 保存 JSON。
R/t 参与标定，不参与图像去畸变。

`check_reason` 区分数值、配置、残差服务、未收敛、非正焦距、RMS 不合格及映射翻折。
`check_warnings` 中的焦距经验范围、主点位置、姿态差异和图片数量只作提示。
生产流程始终拟合完整 k3；`low_coverage` 不关闭高阶项。
算法目标、RMS 与边缘误差的区别见[标定质量](CALIBRATION_QUALITY.md)。

### 二进制协议

查询为 6 字节：`A5 5A op lo hi xor`，最后一字节为 `op ^ lo ^ hi`。`53` 查询状态/参数，`44` 查询执行进度，`43` 查询角点，索引为小端 `lo/hi`。不完整命令约 20 ms 后丢弃，后续帧可以重新同步。

响应固定 130 字节：`A5 5A 01 op seqLE16 78 00 payload[120] crcLE16`。序号是发送包序号；CRC16-CCITT 初值 FFFF、多项式 1021，覆盖版本至载荷末尾，不含 A5 5A 和 CRC。整数和浮点原始位模式均为小端。

| 载荷偏移 | 字节数 | 内容                                        |
| -------- | -----: | ------------------------------------------- |
| 0        |      4 | 上电毫秒计数                                |
| 4        |      4 | 快照过期/有效标志、硬件健康位、摄像头帧计数 |
| 8 / 12   |    各4 | DDR / 算法快照年龄，ms                      |
| 16 / 20  |    各4 | 接收错误 / 查询拒绝计数                     |
| 24       |     32 | UI 状态、变化/访存计数、历史错误            |
| 56       |     64 | 算法快照，内部最后两字节为实际页号          |

保留页号 FFFF 为参数、FFFE 为进度、FFFD 为视频、FFFC 为人工畸变、FFFB 为采集、FFFA 为尺寸与放大状态；非保留页号用于角点索引。采集/尺寸查询复用操作码 0x43，数据包长度不变。精确位布局见 `scripts/debug/uart_monitor.py::decode`，RTL 对应 `algorithm_debug.v` / `core_debug.v` 及板级顶层的诊断页选择。

算法快照字段：字节 6 bit5 为候选有效，字节 7 高四位为检查原因，字节 13 高四位为警告掩码。两个字节的低四位仍为当前/结果阶段。原因码 0=无、1=数值异常、2=配置、3=残差服务、4=未收敛、5=非正焦距、6=RMS、7=映射翻折；警告 bit0..3 依次对应焦距、主点、姿态、图片数量。

协议维护位置为 `rtl/debug` 和 `scripts/debug/uart_monitor.py`。
`python scripts/debug/uart_monitor.py --self-test` 可在无串口时检查解码；
`tb_board_debug`、`tb_debug_snapshot` 和 `tb_core_debug` 覆盖封包、快照、停钟及复位。
