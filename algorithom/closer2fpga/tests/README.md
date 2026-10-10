# C++ 检测与标定测试

检测测试入口为 `run_tests.cmd`：读取实拍图和合成棋盘，比较角点数量、顺序、坐标及非法输入行为。当前检测为整数 Harris、固定半径圆环、方向排序和亚像素迭代，主流程见[项目 README](../README.md)。

标定测试分为固定回归 `run_recommended_tests.cmd`、独立对照及缓冲区测试 `run_calibration_tests.cmd`、导出往返 `run_export_tests.cmd`。详细参数算法见[标定说明](../docs/CALIBRATION_SIMPLIFICATION.md)。

脚本通过 `VS_ROOT`、`OPENCV_ROOT` 定位工具和 OpenCV，产物写入 `tests/build`。桌面示例的图片路径由 `main.cpp` 指定，默认棋盘为 5×8 内角点。合成图用于已知真值，OpenCV 和实拍对照用于检查工程数据上的偏差。
