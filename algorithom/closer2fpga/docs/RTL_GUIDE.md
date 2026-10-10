# C++ 与硬件的对应关系

| C++ 功能 | 当前硬件位置 |
|---|---|
| 灰度扫描、金字塔 | `rtl/memory/image`、`rtl/image/features` |
| 检测流程、候选与网格 | `rtl/control/detection` 的任务程序与 `rtl/image/features` 服务 |
| 亚像素求解与坐标更新 | `rtl/compute/service/feature_program.v` |
| 初值、LM、结果检查 | `scripts/calibration/engine` 生成的程序，`rtl/control/calibration` 的阶段接口 |
| 旋转、投影、残差 | `rtl/compute/geometry` |
| 矩阵步骤与复杂函数 | 共享标定执行器、浮点池和数学微程序 |
| 建表与双线性校正 | `rtl/image/remap` |

循环不一定对应一套独立 RTL：标定通过指令反复使用工作 RAM 和算术后端；图像局部计算则使用窗口、流水线和专用状态机。软件数组也不直接对应寄存器阵列，完整图像和映射表放 DDR。

取指与程序维护详见[指令控制指南](../../../docs/INSTRUCTION_CONTROL_GUIDE.md)，硬件职责和接口详见[系统架构](../../../docs/SYSTEM_ARCHITECTURE.md)。
