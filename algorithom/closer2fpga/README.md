# closer2fpga

纯 C++ 相机标定与去畸变参考实现，用于指导 FPGA Verilog 模块划分。处理边界为 DDR 输入图像到 DDR 输出图像。

当前流程：读取三张图片 → 灰度化 → 内角点检测和亚像素定位 → 联合求解内参、姿态和畸变 → 生成一次映射表 → 彩色双线性去畸变。

先阅读 [算法总览：从棋盘角点到相机标定与去畸变](docs/ALGORITHM_OVERVIEW.md)，了解每一步的输入输出、几何关系和参数含义；再阅读 [Verilog 模块与时序指南](docs/RTL_GUIDE.md) 查看实现划分。

## 运行

打开 `closer2fpga.slnx`，重新生成并运行 `Debug | x64`。

- 默认输入为上级目录 `E:/fpga/algorithom/test0.jpg`、`test1.jpg`、`test2.jpg`，路径在 `closer2fpga/main.cpp` 中设置。
- 三张图使用同一分辨率、同一相机及 5 行 × 8 列内角点棋盘。
- 三个窗口同时显示左侧原图、右侧去畸变结果；关闭全部窗口退出。
- 控制台输出参数及重投影误差，标定失败时不应用校正。
- 三视图默认固定 `k3=0`，其余四个畸变系数参与估计。

OpenCV 用于桌面程序的读图、图像容器、绘制和显示。`algo/`、`common/`、`kernels/` 的计算不依赖 OpenCV；回归测试单独使用 OpenCV 作为数值参考。

## 代码结构

```text
closer2fpga/
  main.cpp                    三图流程与窗口事件循环
  desktop/display.*           绘制与显示容器（OpenCV）
  kernels/                    无状态运算核
    color.h                   BGR 灰度化
    gradient.h                Sobel、外积、2×2 张量运算
    interpolation.h           共享双线性插值
    distortion.h              共享 Brown 畸变模型
  algo/
    grayscale.*               带行跨度的灰度扫描
    chessboard.*              多尺度检测调度
    chessboard/               候选点、网格排序、网格验证
    shi_tomasi.*              梯度/窗口响应/全图阈值/NMS
    subpixel.*                亚像素迭代控制
    calibrate.*               标定入口与多初值调度
    calibration/              初始化、姿态、残差、LM、结果检查
    remap_table.cpp           坐标表生成
    undistort.*               借用缓冲区与拥有内存的 remap 接口
  common/
    image.h / image_view.h    拥有内存的图像 / 借用的行跨度视图
    types.h                   点与相机参数
    math3.*                   三维运算与旋转转换
    symmetric_eigen.*         对称特征分解与外积累加
    matrix.*                  小矩阵与高斯消元
```

运算核不访问图像内存，扫描/迭代控制留在算法层。格式由 `.clang-format` 统一；按职责拆文件，避免把所有标定或检测步骤放在一个长文件中。

## 验证

从上级仓库目录 `E:/fpga/algorithom` 执行：

```powershell
& .\closer2fpga\tests\run_tests.cmd
& .\closer2fpga\tests\run_calibration_tests.cmd
& .\closer2fpga\tests\build_demo.cmd
```

依次检查内角点、标定/去畸变及桌面程序构建。第二个脚本还编译并运行不链接 OpenCV 的核心程序及缓冲区/运算核回归。构建脚本默认使用 `E:\vs` 和其中的 vcpkg，可通过 `VS_ROOT`、`OPENCV_ROOT` 环境变量调整。

- [内角点测试记录](tests/README.md)
- [标定参数、去畸变验证和适用限制](tests/CALIBRATION.md)

测试产物位于 `tests/build`；Visual Studio 缓存和构建目录均由 `.gitignore` 排除。输入图片位于本项目之外，不属于测试输出。

## Verilog 开发参考

从 [模块、DDR 缓冲区与时序划分](docs/RTL_GUIDE.md) 开始阅读。其中逐模块区分无状态算术、窗口缓存、状态机和流水切点，并说明可共享运算单元及其吞吐取舍。

当前不固定 MCU/FPGA 分工，不实现 DDR 控制器、摄像头采集或显示链路。带行跨度的灰度化和 remap 接口已可借用调用方缓冲区；角点检测仍使用连续中间图。所有代码仍为浮点软件参考，定点位宽和逐周期验证留到后续 RTL 实现阶段。
