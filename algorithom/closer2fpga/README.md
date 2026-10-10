# C++ 相机标定与去畸变参考

当前只有一套算法：整数 Harris 候选、固定半径圆环检查、亚像素精修；单初值标定、混合雅可比与按视图分块的 Schur LM；最后建表并双线性去畸变。

## 主流程与代码

`closer2fpga/main.cpp` 读取三张图，逐图灰度化、检测和排序角点，联合计算相机参数，导出角点与结果，再显示原图和校正图。

| 目录/文件 | 功能 |
|---|---|
| `algo/grayscale.*` | 灰度扫描 |
| `algo/chessboard.cpp` | 金字塔与原分辨率回退 |
| `algo/chessboard/candidates.cpp` | 整数 Harris、合并、固定半径圆环 |
| `algo/chessboard/ordering.cpp`、`validation.cpp` | 排列方向搜索、网格排序和检查 |
| `algo/subpixel.*` | 梯度交点迭代定位 |
| `algo/calibrate.cpp` | 单初值、单阶段标定入口 |
| `algo/calibration` | 8 元单应初值、姿态、残差、分块 LM 和检查 |
| `algo/remap_table.cpp`、`undistort.*` | 映射表及双线性取样 |
| `kernels`、`common` | 无状态运算、图像视图、矩阵和几何基础 |
| `desktop` | OpenCV 显示与 CSV/JSON 导出 |

原理见[算法总览](docs/ALGORITHM_OVERVIEW.md)，当前参数求解见[标定实现](docs/CALIBRATION_SIMPLIFICATION.md)。RTL 的实际结构见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，C++ 与 RTL 的初值及求解组织存在差异，不要求逐步执行一致。

## 运行与导出

打开 `closer2fpga.slnx` 构建运行。输入图片和导出根目录由 `main.cpp` 的 `paths`、`export_root` 指定；目前桌面示例仍保存本机绝对路径，换机器运行时在这里修改。默认 3 张图、5×8 内角点；算法接口支持由输入决定的棋盘和视图数。

成功时显示原图/校正图对比；窗口出现前已经写出 `corners.csv`、`calibration.json` 和 `COMPLETE.txt`，格式见[导出说明](docs/CALIBRATION_EXPORT.md)。算法核心不依赖 OpenCV，桌面读图/显示和部分对照测试使用 OpenCV。

## 测试

从仓库根目录运行：

```powershell
& ./algorithom/closer2fpga/tests/run_tests.cmd
& ./algorithom/closer2fpga/tests/run_recommended_tests.cmd
& ./algorithom/closer2fpga/tests/run_calibration_tests.cmd
& ./algorithom/closer2fpga/tests/run_export_tests.cmd
```

分别检查检测、固定基准、标定/校正和导出。工具通过 `VS_ROOT`、`OPENCV_ROOT` 配置，测试产物在 `tests/build`。FPGA 测试则统一在根 `tb` 和 `scripts`。
