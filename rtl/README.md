# RTL 代码导航

先从 [calibrated_view_top.v](top/calibrated_view_top.v) 看板级连接，再从 [vision_ddr_top.v](top/vision_ddr_top.v) 看 DDR 到 DDR 的算法。完整数据流见[系统架构](../docs/SYSTEM_ARCHITECTURE.md)。

## 按硬件职责组织

| 目录 | 负责什么 | 阅读重点 |
|---|---|---|
| `top` | 板级和算法集成连线 | `calibrated_view_top`、`vision_ddr_top` |
| `control/system`、`control/frame` | 多帧任务、帧交接、参数发布和校正调度 | 输入帧何时稳定，何时允许覆盖 |
| `control/detection` | 检测任务取指、服务调用、循环和层切换 | `detection_program`、`detection_flow_control` |
| `control/calibration` | 收集后启动初值、LM、检查及发布结果 | `calib_top` |
| `control/calibration/init,lm,check` | 三个阶段与共享执行器的接口适配 | 搬输入、启动程序、HOST 服务、读回结果 |
| `control/calibration/engine` | 标定指令译码、寄存器、取指执行 | `calib_sequencer` |
| `compute/service` | 运算仲裁与共享执行服务 | `calib_execution_service`、`fp_calibration_pool`、`fp32_pair_add_pool`、`hypot_pool` |
| `compute/float` | FP64 基础算术、复杂函数微程序、校正算术 | `fp_operator`、`fp_math_program`、`calib_alu` |
| `compute/float/stream` | 检测流式算术和格式转换 | FP32/FP64 数据对齐与 CE |
| `compute/geometry` | 旋转、投影、Brown 模型和残差 | `residual_engine`、`geometry_engine` |
| `memory/local`、`memory/parameters` | 工作 RAM、角点、参数 | 同步读延迟、数据归属和生命周期 |
| `memory/ddr`、`memory/image` | DDR 仲裁、HMIC 适配、灰度和候选缓存 | 字节地址、突发、响应归属 |
| `image/features` | 候选处理、方向排序、亚像素和网格检查 | 指令服务适配器与专用数值循环 |
| `image/kernels` | 窗口、梯度、张量、插值 | 局部像素数据通路 |
| `image/remap` | 建表、读取邻域、插值写回 | 校正图从 DDR 到 DDR |
| `video` | 摄像头/HDMI 配置、DMA、像素和跨域通路 | 持续采集显示 |
| `common`、`clock` | FIFO、复位、时钟 | 时钟域边界 |
| `include` | 配置、接口定义、指令编码和生成的 ROM | `.vh` 的来源及数据位宽 |

## 代码与程序的分工

标定初值、差分、正规方程、阻尼求解和检查步骤在 `scripts/calibration/engine` 生成的程序中。RTL 阶段模块主要负责输入输出，矩阵计算没有另一套板级高斯/Jacobi 控制器。`pose_init` 是保留的独立姿态程序测试包装，板级初值由共享程序直接执行。

检测的任务顺序在 `scripts/image/build_detection_program.py`；单个方向的投影/排序、单点圆环检查等仍在 `image/features`。精修数值步骤由 `compute/service/feature_program.v` 执行。

`compute/geometry/geometry_engine.v` 用固定指令表完成投影，`project_point` 和 `brown_distort` 是不同接口包装。相同源码可以被不同测试使用，板级共享由顶层实例和仲裁连接决定。

## 工程和验证入口

`files.f` 是公共仿真源码清单，`scripts/build/pds_excluded_sources.txt` 标出只供独立测试或非板级组合使用的模块。厂商 IP 由 PDS 工程的 IDF 管理，位于 `OV5640_DualView_100H/ipcore`。

增删 RTL 后运行：

```powershell
python scripts/build/check_rtl_layout.py --update
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build/configure_pds.ps1
python scripts/build/check_rtl_layout.py
```

`--update` 先更新清单，然后检查 PDS，因此工程尚未同步时会报告集合差异。第二步保存并重新打开 PDS，第三步核对最终路径。

板级顶层直接维护；`vision_ddr_top`、`vision_camera_top` 的连线维护源分别是 `scripts/build/generate_wiring.js`、`generate_camera_wiring.js`。功能测试在根 `tb`，运行入口见[脚本说明](../scripts/README.md)。
