# 计算数据通路与共享服务

`service` 负责共享执行器、请求仲裁、响应归属；`float` 负责算术和复杂函数；`geometry` 负责旋转、投影和整批残差。

标定执行器、残差及检测精修接入共享 FP64 池。检测另有四客户端的 FP32 成对加减池和距离池。`feature_program` 计算张量位移、更新和收敛量，`fp_math_program` 计算数学函数内部步骤。

`geometry_engine` 使用固定运算序列和工作 RAM，投影包装交入参数后等待结果。旧的独立高斯/Jacobi 数据通路已被标定程序替代，矩阵算法维护在 `scripts/calibration/engine`。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
