# C++ 标定与去畸变验证

当前验证对象是单初值、混合雅可比、按视图分块 Schur LM，k3 固定为零。

| 脚本 | 范围 |
|---|---|
| `run_recommended_tests.cmd` | 实拍和合成数据与固定基准对比，可变棋盘/视图数及非法输入 |
| `run_calibration_tests.cmd` | OpenCV 标定/映射对照，纯 C++ 标定与 remap，缓冲区接口 |
| `run_export_tests.cmd` | JSON/CSV 数值往返和导出失败处理 |

`camera.valid` 成功后才应用校正。实现流程见[当前标定算法](../docs/CALIBRATION_SIMPLIFICATION.md)，RTL 的阶段验证入口见[根测试目录](../../../tb/README.md)。
