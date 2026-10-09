# 标定与去畸变验证

当前只实现固定推荐方案，k3 始终为零。原理、边界和执行命令见 [标定实现说明](../docs/CALIBRATION_SIMPLIFICATION.md)。

- run_recommended_tests.cmd：真实角点与七组合成数据对比保存的推荐方案结果；覆盖可变棋盘、视图数和非法输入。
- run_calibration_tests.cmd：独立 OpenCV 标定/映射对照、纯 C++ 标定与 remap、缓冲区回归。
- run_export_tests.cmd：JSON/CSV 数值往返和导出失败处理。

只有 camera.valid 为真才应用校正。
