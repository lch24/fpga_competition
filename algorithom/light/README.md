# 亮度敏感性实验

比较 `algorithom/an.png、an2.png、an3.png` 与 `ming.png、ming2.png、ming3.png`，并对同一张灰度图做亮度增益实验。棋盘默认为 8 列、5 行内角点。

程序同时运行 OpenCV SB 和项目 `closer2fpga` 的 C++ 角点检测器，使用同一灰度输入；两条分支都由 OpenCV 求解固定 k3=0 的标定模型。项目检测源码直接引用，没有复制。这里不模拟 RTL 或调用板上算法。

从仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File algorithom/light/run.ps1
```

脚本自动查找 Visual Studio；可用 `-VsRoot`、`-VcpkgRoot` 指定本机安装位置。依赖 vcpkg 的 `opencv4:x64-windows`。`-Columns`、`-Rows` 修改棋盘尺寸，`-InputDirectory` 修改图片目录；`-Output` 指定一个尚不存在的结果目录，默认写入 `build/light/run_时间戳/`。全部图像处理与标定均在 C++ 中完成。

结果包括：

- `report.md`：亮度、检测、标定参数、跨组重投影及同图增益对比表。
- `results.json`：完整角点、各视图误差、标定参数、估计标准差和软件计时。
- `input_sha256.json`：输入图片和参与计算的源码校验值。
- `overlays/`：原始图片的检测角点叠加结果。

`light/main.cpp` 负责命令行，`light/experiment.cpp` 负责检测、标定和统计。Visual Studio 工程使用相同源文件，选择 x64；OpenCV 路径由 `VCPKG_ROOT` 或 Visual Studio 安装目录下的 `vcpkg` 确定，也可设置工程属性 `OpenCvRoot`。调试参数已设为默认素材目录和 `build/light/visual_studio_run`，重复运行需更换输出目录。命令行脚本会先检查角点编号匹配、RMS 定义及合成相机参数恢复。

实际暗、亮照片有轻微移动，不能把两组参数差值全部归因于亮度。同图增益保持几何位置不变，可单独检验亮度与量化的影响，但不模拟低照度新增的噪声或运动模糊。
