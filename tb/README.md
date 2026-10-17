# 仿真测试平台

`demo/` 保存人工畸变扩展的长短按、流程调度、逆映射、建表、参数隔离与 UART 测试，由 `scripts/demo/run_checks.py` 执行。关闭扩展时继续使用原测试，详见[人工畸变演示](../docs/DISTORT_DEMO.md)。

这里保存 HDL 激励、断言和行为模型，运行器在 [scripts](../scripts/README.md)，参考输入在 [data](../data/README.md)。

`system/tb_photo_preview.sv` 覆盖正常/人工畸变预览、照片回看定时、DDR 在途任务排空、失败恢复和预览到标定/校正的交接。运行：`python scripts/demo/run_checks.py --modelsim <ModelSim目录> --only tb_photo_preview`。残差寄存器压缩的数值与复位验证使用 `scripts/compute/run_residual_engine.ps1`。

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

旧的独立高斯、Jacobi、单应、Zhang、雅可比、正规方程和阻尼控制器测试已随被替代 RTL 移除。对应算法由当前标定程序测试覆盖，入口见 [calibration/docs](calibration/docs/README.md)。两个 `tb_fp32` 分别测试不同的数据通路，使用各自编译库。

检测保留当前整数响应、圆环、候选恢复、共享排序、精修窗口、金字塔重算和整图回归。依赖旧 Shi–Tomasi 导出器且没有输入向量的里程碑 TB 已删除。
