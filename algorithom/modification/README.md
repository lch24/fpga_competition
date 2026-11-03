# OpenCV 标定对照

使用 OpenCV 标准角点检测与相机标定，对照 FPGA 的角点坐标、内参、畸变系数和重投影误差。`modification/main.cpp` 处理命令行，`compare.cpp` 实现对照与合成数据自检。

在仓库根目录运行同角点参数对照：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File algorithom/modification/run.ps1 -CornerSet -Manifest data/calibration/board_20261102_170134/solver_input.json
```

脚本定位 Visual Studio 和 vcpkg OpenCV，也可通过 `-VsRoot`、`-VcpkgRoot` 指定。默认输出 `build/opencv_comparison/detection`；再次运行时先清理输出或用 `-Output` 指定新目录。脚本先执行数学自检，再处理输入。

`-CornerSet` 直接使用清单中的有序角点，比较固定 k3 与自由 k3 的求解结果；FPGA 参数排列为 `fx,fy,cx,cy,k1,k2,k3,p1,p2`，程序负责与 OpenCV 的畸变顺序转换。不带该开关时，`-Manifest` 按清单中的图像组运行检测和标定。

无清单模式沿用 `algorithom/ftest1.png`～`ftest6.png` 的六图检测入口，需自行准备完整六张图片。仓库保留的 `ftest4.png`～`ftest6.png` 只有部分画面能与已归档角点对应；同角点对照不依赖这些图片。
