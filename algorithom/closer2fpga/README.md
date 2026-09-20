# closer2fpga

纯 C++ 相机标定与去畸变原型，面向后续 FPGA 的 PS/PL 分工。

当前流程：读取三张图片 → 灰度化 → 内角点检测和亚像素定位 → 联合求解内参、姿态和畸变 → 生成一次映射表 → 彩色双线性去畸变。

## 运行

打开 `closer2fpga.slnx`，重新生成并运行 `Debug | x64`。

- 默认输入为上级目录 `E:/fpga/algorithom/test0.jpg`、`test1.jpg`、`test2.jpg`，路径在 `closer2fpga/main.cpp` 中设置。
- 三张图使用同一分辨率、同一相机及 5 行 × 8 列内角点棋盘。
- 三个窗口同时显示左侧原图、右侧去畸变结果；关闭全部窗口退出。
- 控制台输出参数及重投影误差，标定失败时不应用校正。
- 三视图默认固定 `k3=0`，其余四个畸变系数参与估计。

OpenCV 用于桌面程序的读图、图像容器、绘制和显示。`algo/`、`common/` 的计算不依赖 OpenCV；回归测试单独使用 OpenCV 作为数值参考。

## 代码结构

```text
closer2fpga/
  main.cpp                  三张图的读取、流程组织及显示
  algo/
    chessboard.h/.cpp       候选点筛选、方向搜索、网格组织
    shi_tomasi.h/.cpp       Sobel 和角点响应
    subpixel.h/.cpp         迭代亚像素定位
    calibrate.h/.cpp        完整相机标定（初始化、姿态、LM）
    undistort.h/.cpp        映射表与灰度/彩色双线性插值
  common/
    types.h                坐标、相机参数与数值类型
    image.h                连续图像存储
    matrix.h/.cpp          小矩阵存储与高斯消元
```

每个算法只有一套正在使用的实现。旧版已知内参拟合器、停用的合成入口和早期导出图片已移除。

## 验证

从上级仓库目录 `E:/fpga/algorithom` 执行：

```powershell
& .\closer2fpga\tests\run_tests.cmd
& .\closer2fpga\tests\run_calibration_tests.cmd
& .\closer2fpga\tests\build_demo.cmd
```

依次检查内角点、标定/去畸变及桌面程序构建。第二个脚本还编译并运行不链接 OpenCV 的独立核心程序。构建脚本默认使用 `E:\vs` 和其中的 vcpkg，可通过 `VS_ROOT`、`OPENCV_ROOT` 环境变量调整。

- [内角点测试记录](tests/README.md)
- [标定参数、去畸变验证和适用限制](tests/CALIBRATION.md)

测试产物位于 `tests/build`；Visual Studio 缓存和构建目录均由 `.gitignore` 排除。输入图片位于本项目之外，不属于测试输出。

## 后续 FPGA 工作

当前为浮点 C++ 软件原型，尚无 RTL、定点实现或实时视频采集接口。

- PS：网格组织、亚像素定位、相机标定及映射表生成。
- PL 候选：灰度化、Sobel/角点响应、查表及双线性插值。
- 移植前需用更多实拍数据检验鲁棒性，并量化定点误差、映射表容量和访存带宽。
- 去畸变涉及非顺序源坐标访问，缓冲方案应根据实际映射范围设计，不能直接假定两行缓存足够。
