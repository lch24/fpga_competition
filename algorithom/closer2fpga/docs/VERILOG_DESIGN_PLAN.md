# 当前 Verilog 设计入口

RTL 已集中到仓库根目录 `rtl`，当前实现以分层微程序和共享计算服务为中心。完整设计见[系统架构](../../../docs/SYSTEM_ARCHITECTURE.md)、[RTL 导航](../../../rtl/README.md)、[检测设计](../../../docs/DETECTION_ENGINE.md)和[标定设计](../../../docs/CALIBRATION_ENGINE_ARCHITECTURE.md)。

硬件边界为 DDR 原图输入、DDR 校正图输出。板级外层连接摄像头采集和 HDMI 显示；内部依次进行灰度、检测、角点收集、初值、LM、检查、建表和插值。

C++ 描述算法，RTL 按吞吐与存储组织实现。当前 RTL 初值仍用归一化 DLT/Jacobi，C++ 已用 8 元线性求解；两者都只用一组焦距初值、固定 k3，并联合优化各图 R/t。具体数值容差由测试规定。
