# 仿真测试平台

`demo/` 保存人工畸变扩展的长短按、流程调度、逆映射、建表、参数隔离与 UART 测试，由 `scripts/demo/run_checks.py` 执行。关闭扩展时继续使用原测试，详见[人工畸变演示](../docs/DISTORT_DEMO.md)。

这里保存 HDL 激励、断言和行为模型，运行器在 [scripts](../scripts/README.md)，参考输入在 [data](../data/README.md)。

裁剪与最终显示使用 `video/tb_camera_crop.sv`、`tb_video_scale.sv`、`tb_board_video.sv`，由 `scripts/system/run_realtime_checks.ps1 -OnlyTest <名称>` 分别运行。完整传感器裁剪测试可对 `tb_camera_crop` 覆盖参数 `W=1280,H=720`；板级显示测试直接检查两幅 1280×720 完整画面。`system/tb_realtime_remap.sv` 默认使用 960×8 条带，覆盖实际行步长与边界，避免每次都运行完整算法。采集短行、字节对齐、跨域队列和异常恢复回归使用 `scripts/video/run_camera_checks.ps1`。

`system/tb_photo_preview.sv` 覆盖正常/人工畸变预览、照片回看定时、DDR 在途任务排空、失败恢复和预览到标定/校正的交接。运行：`python scripts/demo/run_checks.py --modelsim <ModelSim目录> --only tb_photo_preview`。残差寄存器压缩的数值与复位验证使用 `scripts/compute/run_residual_engine.ps1`。

`video/tb_camera_integrity.sv` 检查裁剪内外短行、奇数字节、超长行、错误行数与下一帧恢复；`tb_camera_config.sv` 在实际 SCCB 字节应答处注入 NACK，检查重试、极性寄存器和完成条件。`image/tb_grid_order_robust.sv` 检查杂点剔除和退化网格拒绝，独立/共享运算两种模式由 `scripts/image/run_grid_order_checks.ps1` 运行。

`video/tb_async_bram_fifo.sv` 检查填满后背压、连续每拍读取、指针回绕、不同频率时钟、输出停顿和带数据复位；`ADDR_BITS=11` 对应板级 HDMI 队列深度。`tb_hdmi_bridge.sv` 检查欠载后的帧同步恢复，`tb_board_video.sv` 用完整两帧逐像素验证显示链路。三者由 `scripts/system/run_realtime_checks.ps1 -OnlyTest <名称>` 运行。摄像头带电重启和完整配置等待分别由 `tb_camera_power.sv`、`tb_camera_config.sv` 验证。

| 目录 | 与当前架构的对应关系 |
|---|---|
| `control` | 检测任务程序、层切换适配器、标定取指核心和服务接口 |
| `calibration` | 初值/LM/检查阶段适配器、标定顶层；姿态程序独立包装 |
| `compute` | 浮点后端、精修微程序、FP32 加减池、距离服务和几何模型 |
| `memory` | 角点 RAM、灰度/候选缓存、DDR 仲裁、DMA、跨时钟桥 |
| `image` | 像素核、候选输入、方向排序、精修和整图检测 |
| `remap` | 映射坐标、建表、采样校正、参数缓存 |
| `video` | 摄像头/显示接口、跨域和配置时钟 |
| `system` | DDR 闭环、板级任务和完整数值流程 |
| `debug` | 串口协议、停钟/复位、快照一致性、主机解码交叉验证 |
| `models` | DDR 行为模型与公共连接头文件 |

验证按层展开：指令测试检查调用顺序、分支、取消和背压；数值测试使用独立期望值检查结果；集成测试连接真实服务，检查数据交接和任务结束。板级 IP 端口桩用于连接与调度检查，不模拟物理 DDR/PLL 的全部行为。

矩阵算法由标定程序测试覆盖，入口见 [calibration/docs](calibration/docs/README.md)。
两个 `tb_fp32` 分别测试流式和重映射数据通路，使用各自编译库。
