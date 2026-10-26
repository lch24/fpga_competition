# FPGA 相机标定与去畸变

本项目在 FPGA 上完成摄像头采集、棋盘角点检测、相机标定、图像去畸变和 HDMI 显示。
板级入口是 [calibrated_view_top.v](rtl/top/calibrated_view_top.v)，
PDS 工程是 [DualView_OV5640.pds](OV5640_DualView_100H/DualView_OV5640.pds)，
引脚和时钟约束在同目录的 [FDC](OV5640_DualView_100H/DualView_OV5640.fdc)。

## 处理流程

```text
摄像头 1280×720 → 中心裁剪 960×540 → RGB565 写入 DDR
                                   ├→ 实时预览 / 按键保存三张照片
三张照片 → 逐图灰度与角点检测 → 初值 → 完整模型 LM → 参数检查
                                                        ↓
连续采集 → DDR 原图 → 查校正映射表 → 双线性插值 → DDR 校正图
                                                        ↓
                                显示出口放大 4/3 → HDMI 1280×720
```

裁剪起点为 `(160,90)`，宽高各保留 75%。检测、标定和校正都在 960×540
坐标系中进行；只在显示时放大。完整帧、灰度金字塔和映射表存放在 DDR，
片上保存行缓存、FIFO、角点和计算工作区。

默认 `REALTIME_VIDEO=1`：KEY0～KEY2 拍照或回看两秒，KEY3 检测与标定，
KEY4 建表后启动校正视频，再按停止；视频期间 KEY0 切换校正前后。
上电及回看结束后自动预览。按键、LED 和串口见[板上操作](docs/BOARD_OPERATION.md)。

人工畸变是编译期可选扩展，默认 `ENABLE_DISTORT_DEMO=0`。
启用后可先人为改变输入图像，再独立检测、标定和校正；
人工参数和人工映射表不进入求解器。操作见[人工畸变](docs/DISTORT_DEMO.md)。

## 代码组织与阅读入口

| 目录 | 内容 |
|---|---|
| [rtl](rtl/README.md) | 按功能划分的硬件：控制、计算、存储、图像、视频、调试和顶层连接 |
| [scripts](scripts/README.md) | 程序生成、仿真运行、工程配置、串口和软件复算工具 |
| [tb](tb/README.md) | HDL 测试与行为模型，按功能划分 |
| [data](data/README.md) | 汇编源、ROM 表、测试输入、真实角点及独立参考数据 |
| [algorithom/closer2fpga](algorithom/closer2fpga/README.md) | C++ 检测、标定与数据导出参考 |
| `OV5640_DualView_100H` | PDS 工程、约束和厂商 PLL/DDR IP |
| `build` | 可重新生成的仿真、分析输出，不提交 |

建议先读[系统架构](docs/SYSTEM_ARCHITECTURE.md)，再按任务阅读
[实时视频](docs/REALTIME_VIDEO.md)、[检测](docs/DETECTION_ENGINE.md)、
[标定架构](docs/CALIBRATION_ENGINE_ARCHITECTURE.md)和[算法质量指标](docs/CALIBRATION_QUALITY.md)。
程序入口、取指和 HOST 服务见[指令控制指南](docs/INSTRUCTION_CONTROL_GUIDE.md)。

复杂数值循环通过微程序复用计算资源；摄像头、DDR、像素流水、插值和显示保留专用电路。
初值、LM、检查共用一个标定执行器；检测精修与标定共享 FP64 后端。
这些程序随 bitstream 初始化，板上运行不依赖电脑。

## 配置与接口

标定配置集中在 [calib_config.vh](rtl/include/calib_config.vh)，默认 3 张图、5×8 内角点。
角点按视图、行优先点号存储，使用裁剪图像素坐标 FP32；`view` 区分图片，`job` 区分任务。
标定内部主要使用 FP64，联合求解内参、五项畸变及每张图的 R/t。
输出九项 FP32：`fx, fy, cx, cy, k1, k2, k3, p1, p2`；R/t 用于标定和诊断。
生产路径始终拟合 k3，覆盖度仅作提示。

请求在 `valid && ready` 时交接，背压时保持载荷。算法域为 40 MHz，DDR 控制域为
125 MHz；跨域采用 FIFO 或请求应答快照。算法 DDR 地址按字节计，native HMIC
按 32 位字计，标定工作 RAM 按 64 位字计。

## 实际板上资源数：

当前 PDS 选择的器件为 **PANGO Logos2 PG2L100H，FBG676 封装，-6 速度等级**。下表容量取自当前工程配置和 PDS 综合报告的器件上限，是整个器件的总量，包含摄像头、DDR 控制器、HDMI 等基础功能需要占用的部分，并非全部可分配给算法。

| 资源                      |                         器件总量 | 在本项目中的用途与限制                                                                                                                         |
| ------------------------- | -------------------------------: | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| LUT（查找表）             |                 **66,600** | 组合运算、选择器、地址生成、控制逻辑及部分分布式存储；当前主要瓶颈                                                                             |
| FF（寄存器）              |                **133,200** | 流水线、状态和数据暂存；不能用 FF 余量直接抵消 LUT 超限                                                                                        |
| 可作分布式 RAM 的 LUT     |                 **19,900** | 是 LUT 总量中的可用子集，不是额外增加的 LUT；使用它们存数据会占用逻辑资源                                                                      |
| DRM36K/FIFO（片上块 RAM） | **155 个 36 Kibit 等效块** | 总计**5,580 Kibit = 697.5 KiB ≈ 0.681 MiB**；用于窗口、FIFO、角点、矩阵和工作缓存                                                       |
| APM（专用算术块）         |                    **240** | 承担适合映射的乘法、乘加等运算；不等于 240 个完整浮点运算器，浮点控制、规格化等仍可能消耗 LUT/FF                                               |
| 用户 I/O                  |                    **300** | 器件用户端口上限；实际可用引脚还受板级布线、接口占用和电气约束限制，不能按封装的 676 个焊球计算                                                |
| 板外 DDR3     |     **当前配置对应 1 GiB** | DDR IP 配置为 15 位行地址、10 位列地址、3 位 Bank 地址、32 位数据宽度，容量为`2^(15+10+3) × 4` 字节；属于外部存储，不增加 FPGA 内部 LUT/DRM |

当前 960×540 RGB565 图约 0.99 MiB，Gray8 图约 506 KiB，完整帧存放 DDR。DDR 的 1 GiB 是配置容量，不是算法实际用量。最终资源占用以当前版本整板综合和布局布线报告为准，局部优化估算不作为整板已放入的结论。

## 当前 DDR 分区

以下为当前 960×540、两层灰度金字塔的**字节地址**。定义见板级顶层的 `RAW_BASE` 与算法顶层的地址参数。

| 内容             | 起始地址       | 有效数据量     |
| ---------------- | -------------- | -------------- |
| RGB565 原图 1 | `0x00000000` | 1,036,800 字节 |
| RGB565 原图 2 | `0x00200000` | 1,036,800 字节 |
| RGB565 原图 3 | `0x00400000` | 1,036,800 字节 |
| 视频原图 0/1/2 | `0x00800000` / `0x00A00000` / `0x00C00000` | 每份 1,036,800 字节 |
| 视频校正图 0/1 | `0x01000000` / `0x01200000` | 每份 1,036,800 字节 |
| 打包定点映射表 | `0x02000000` | 4,147,200 字节 |
| Gray8 灰度金字塔 | `0x03000000` | 648,000 字节 |
| 候选点工作区     | `0x03200000` | 262,144 字节   |

有效数据合计 13,351,744 字节（约 12.73 MiB）；每个 RGB565 帧槽仍预留 2 MiB，行步长为 1,920 字节。显示放大后的帧不写回 DDR。`0x02800000` 在实时模式不使用；旧照片模式仍使用分离 FP32 X/Y 表。

## 开发入口

工具位置由 `MODELSIM_BIN`、`PDS_SHELL` 或 PATH 指定。以下命令从仓库根目录运行：

```powershell
python scripts/build/check_rtl_layout.py
python scripts/build/check_test_layout.py
python scripts/calibration/engine/build_rom.py
powershell -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
python scripts/debug/uart_monitor.py COM5 --once params
```

修改程序源后重新生成 `rtl/include` 中的 ROM；正常 clone 后可直接使用已提交的生成物。
添加或删除 RTL 文件时，运行 `scripts/build/configure_pds.ps1` 同步相对路径并重新打开工程。
按修改范围选择仿真，入口见[脚本说明](scripts/README.md)。
实际资源与时序以该版本 PDS 布局布线报告为准。
