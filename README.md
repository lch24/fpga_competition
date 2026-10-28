# FPGA 相机标定与去畸变

本分支为 `demo`：在 main 的[两次 KEY3、红点查看角点和计算前 k3 覆盖检查](docs/CORNER_REVIEW.md)基础上，合入 advance 的[提高部分 1（上位机动态旋转/仿射）和 3（自适应 Gamma）](docs/ADVANCED_DYNAMIC_GAMMA.md)。三个特性默认都关闭；演示版 profile 需要 `REALTIME_VIDEO=1, ENABLE_CORNER_REVIEW=1, ENABLE_ADVANCED=1, ENABLE_DISTORT_DEMO=0`，运行 `python scripts/build/prepare_demo_project.py` 后打开生成的 `OV5640_Demo_100H/Demo_CornerReview_Advanced.pds`。红点版与人工畸变演示仍互斥。

本项目在 FPGA 上完成摄像头采集、棋盘角点检测、相机参数求解、图像校正和 HDMI 显示。默认板级 `REALTIME_VIDEO=1`：KEY0～KEY2 拍摄或查看三张标定照片，KEY3 标定，KEY4 一次建表后启动实时视频，再按 KEY4 排空并停止。规格目标是 **1280×720、RGB565、每秒 30 个新校正帧**，不是重复显示静态图片。2026-10-20 普通模式已实板测得约 30.01 个新校正帧/秒，资源已放下，但布局时序仍未收敛。详见[实时链路交接](docs/REALTIME_VIDEO.md)与[板上操作说明](docs/BOARD_OPERATION.md)。

## 从输入到输出

2026-10-19：增加基于角点边缘覆盖的保守 k3 模型选择，并提供角点编号叠图与软件复算。
2026-10-20 普通模式实板测得约 30.01 个新校正帧/秒，稳态无新增失败/跳帧，停止重启及前后切换正常；资源已放下，但 125 MHz setup WNS 仍为 −0.718 ns，不能把功能通过当作时序签核。修改、实测参数和后续任务见[标定质量与实板交接](docs/CALIBRATION_QUALITY.md)。

摄像头就绪后自动进入拍照预览。KEY0～KEY2 拍摄空槽位，成功后回看 2 秒；已有照片也回看 2 秒，然后恢复预览。人工畸变模式下预览与照片都经过人工加畸变。KEY3 标定结束、KEY4 停止校正视频后也返回预览。

```text
摄像头 → RGB565 原图写入 DDR
             ↓ 每张标定图
       灰度/金字塔 → 候选点 → 有序棋盘角点 → 角点 RAM
                                                  ↓ 收齐视图
                                           初值 → LM → 结果检查
                                                  ↓ 相机参数
连续原图 DDR（3 槽）→ 源像素缓存 → 定点双线性插值 ← 64 位映射表（只建一次）
                    ↓
             校正图写入 DDR（2 槽）→ 完整帧切换 → HDMI
```

三张标定原图分别保存在 DDR，检测依次处理它们，共用灰度金字塔和候选点工作区。视频另外使用三份原图槽位、两份校正槽位；不会覆盖标定照片。完整图像和映射表放在 DDR，片上只放块缓存、FIFO、角点和计算工作区。`REALTIME_VIDEO=0` 保留旧照片模式；默认板级不实例化旧的逐像素校正读写器。

## 当前架构

系统由专用数据通路和分层微程序控制器组成。像素扫描、采集显示、DDR、投影与插值保留专用电路；迭代和重复数值步骤通过程序复用运算器。

| 控制器 | 作用 | 代码入口 |
|---|---|---|
| 检测任务程序 | 管理金字塔层、候选遍历、方向搜索、精修调度 | `rtl/control/detection/detection_program.v` |
| 标定程序 | 初值、LM、结果检查轮流使用同一执行器和工作 RAM | `rtl/compute/service/calib_execution_service.v` |
| 精修微程序 | 求解局部张量方程、更新角点坐标、计算收敛量 | `rtl/compute/service/feature_program.v` |
| 数学微程序 | 实现复杂数学函数内部的分步运算 | `rtl/compute/float/fp_math_program.v` |

这些控制器属于不同层次。检测精修与标定通过仲裁共享 FP64 后端；检测的多个模块还共享 FP32 加减和距离服务。几何投影内部另有固定运算序列表。控制器数量不等于独立算术单元数量，也不意味着每个像素由通用 CPU 执行。

程序源在 `scripts/*` 和 `data/programs`，生成的指令表在 `rtl/include`，随 FPGA bitstream 初始化。算法目标时钟为 40 MHz，摄像头、DDR 和显示通过跨时钟接口连接。

## 阅读与代码组织

| 入口 | 内容 |
|---|---|
| [板上操作](docs/BOARD_OPERATION.md) | 按键、LED、拍照/标定/校正、串口调试和相关测试 |
| [系统架构](docs/SYSTEM_ARCHITECTURE.md) | 整体流程、时钟、存储和模块之间的数据交接 |
| [RTL 导航](rtl/README.md) | 按功能组织的目录及关键源码 |
| [指令控制指南](docs/INSTRUCTION_CONTROL_GUIDE.md) | 启动、取指、执行、HOST 服务及程序生成 |
| [检测设计](docs/DETECTION_ENGINE.md) | 检测任务程序和专用计算服务 |
| [标定设计](docs/CALIBRATION_ENGINE_ARCHITECTURE.md) | 初值、LM、检查如何共用执行器 |
| [测试平台](tb/README.md)、[脚本](scripts/README.md)、[数据](data/README.md) | 验证入口、程序源与参考数据 |
| [C++ 参考](algorithom/closer2fpga/README.md) | 算法原理、软件实现及导出 |

板级入口是 [calibrated_view_top.v](rtl/top/calibrated_view_top.v)，算法 DDR 到 DDR 入口是 [vision_ddr_top.v](rtl/top/vision_ddr_top.v)。PDS 工程为 [DualView_OV5640.pds](OV5640_DualView_100H/DualView_OV5640.pds)，引脚和时钟约束为同目录 [DualView_OV5640.fdc](OV5640_DualView_100H/DualView_OV5640.fdc)。

源码按职责分为 `rtl/control` 调度、`rtl/compute` 计算服务、`rtl/memory` 存储、`rtl/image` 图像数据通路、`rtl/video` 采集显示，`rtl/top` 负责连接。厂商 IP 位于工程目录的 `ipcore`；仿真产物统一写入 `build`。

`rtl/io/uart` 提供串口字节收发，`rtl/debug` 独立采集状态和数值记录；电脑端入口为 `scripts/debug/uart_monitor.py`，测试位于 `tb/debug`。

## 配置和数据接口

默认 3 张图、5×8 内角点，标定配置集中在 [calib_config.vh](rtl/include/calib_config.vh)。板级图像为 1280×720 RGB565，检测使用 Gray8；角点以原图像素坐标 FP32 表示，按行优先排列。`view` 区分同一任务的各张图，`job` 区分整次任务。

标定内部主要使用 FP64，输出九个 FP32 参数：`fx, fy, cx, cy, k1, k2, k3, p1, p2`，stage 2 的 k3 已计入 LM 活动参数。各图 R/t 参与 LM，但不作为图像去畸变的输入。RTL 当前使用 DLT/Jacobi 初值和单阶段 LM；C++ 的单应初值已简化为 8 元求解，两者是数值对照关系，并非逐指令相同。

请求/响应在 `valid && ready` 时交接，背压期间保持载荷。算法 DDR 接口使用字节地址；物理 DDR 总线宽度和突发由访存适配层处理。

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

整幅 720p RGB565 图约 1.76 MiB，Gray8 图为 900 KiB，所以完整帧存放 DDR。DDR 的 1 GiB 是配置容量，不是算法实际用量。最终资源占用以当前版本整板综合和布局布线报告为准，局部优化估算不作为整板已放入的结论。

2026-10-12 22:02 最新实时版：综合 60,310 LUT；布局后物理 LUT **66,024/66,600（99.14%）**，逻辑簇 **16,627/16,650（99.86%）**，FF 52,804，RAM 物理位置 105/155，APM 70/240。资源已放下但仅剩 23 个逻辑簇。125 MHz DDR/视频域 WNS **−0.949 ns**、113 个 setup 违例端点；40 MHz 算法域 +0.308 ns。资源通过不代表时序或 30 新帧/秒通过，下一位仍需上板验证。报告摘要和待办见[实时交接](docs/REALTIME_VIDEO.md#本次-pds-结果2026-10-12)。

## 当前 DDR 分区

以下为默认 1280×720、两层灰度金字塔的**字节地址**。定义见板级顶层的 `RAW_BASE` 与算法顶层的地址参数。

| 内容             | 起始地址       | 有效数据量     |
| ---------------- | -------------- | -------------- |
| RGB565 原图 1 | `0x00000000` | 1,843,200 字节 |
| RGB565 原图 2 | `0x00200000` | 1,843,200 字节 |
| RGB565 原图 3 | `0x00400000` | 1,843,200 字节 |
| 视频原图 0/1/2 | `0x00800000` / `0x00A00000` / `0x00C00000` | 每份 1,843,200 字节 |
| 视频校正图 0/1 | `0x01000000` / `0x01200000` | 每份 1,843,200 字节 |
| 打包定点映射表 | `0x02000000` | 7,372,800 字节 |
| Gray8 灰度金字塔 | `0x03000000` | 1,152,000 字节 |
| 候选点工作区     | `0x03200000` | 262,144 字节   |

有效数据合计 23,532,544 字节（约 22.44 MiB）；每个 RGB565 帧槽预留 2 MiB。`0x02800000` 在实时模式不使用；旧照片模式仍使用分离 FP32 X/Y 表。

## 使用入口

可选的[人工畸变演示](docs/DISTORT_DEMO.md)默认不编入硬件。顶层 `ENABLE_DISTORT_DEMO=1` 时，长按 KEY3 可切换“先人工加畸变、再独立标定和校正”；两遍图像处理共享同一重映射器。换摄像头后恢复为 `0` 并重新生成比特流即可移除扩展。首次人工建表的坐标计算约需 40 秒，详见该文档的操作、验证和时间说明。

工具路径通过 `MODELSIM_BIN`、`PDS_SHELL` 或 PATH 指定。以下命令从仓库根目录运行：

```powershell
python scripts/build/check_rtl_layout.py
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build/configure_pds.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
```

PDS 配置脚本同步相对源码路径、准备图像 ROM 并保存、重新打开工程核对；不启动综合。按修改范围选择数值回归，具体命令见脚本文档。


串口调试已接入板级顶层：115200 8N1，周期报告状态，可查询参数、角点和共享程序执行进度。使用方法见[板上操作](docs/BOARD_OPERATION.md#串口调试)。
