# 当前算法标识

导出已切换为 single_seed_forward_lm_v1：单初值、单阶段、最多 60 次迭代、k3=0。rtl_algorithm_matches=false 表明旧 RTL 尚未迁移；rtl_input_ready 仅为输入格式检查。接口不再接收算法配置对象。详见 [当前标定实现](CALIBRATION_SIMPLIFICATION.md)。

# 用真实角点对比 C++ 与 FPGA

重新编译并运行桌面程序即可自动导出，无需另按保存按钮。输入仍是 `main.cpp` 中配置的三张图片，角点检测、标定和窗口显示流程保留。

## 到哪里找文件

默认目录：`E:\fpga\algorithom\closer2fpga\exports\run_<时间戳>_<序号>\`。

程序在控制台打印本次完整目录。每次运行新建子目录，不覆盖上一轮结果。导出发生在标定计算之后、生成校正图和显示窗口之前，出现窗口时文件已经写好。路径可以在 `main.cpp` 的 `export_root` 修改，不受 Visual Studio 工作目录影响。

| 文件 | 内容 |
| --- | --- |
| `corners.csv` | 本次检测出的有序角点，包含图号、点号、像素坐标和原始FP32位模式 |
| `calibration.json` | 图片路径、每图检测状态、宽高、棋盘尺寸、格长、算法配置，以及本次C++算出的参数和诊断 |
| `COMPLETE.txt` | 本目录导出完成标记，最后生成；缺少此文件时不要用于对比 |

**运行后把控制台打印的本次目录路径告诉我即可。** 后续将该目录的角点原始位模式送入 `calib_top`，再把RTL输出与同目录的C++结果对比。这里对比的是两种实现处理同一组实拍角点后的结果，不是把C++算出的参数当作FPGA迭代初值。

## 角点格式

```text
view_id,point_index,x,y,x_fp32_hex,y_fp32_hex
```

图号为0、1、2，对应 `paths[0]`、`paths[1]`、`paths[2]`；每图点号按检测器原有棋盘排序，从0到39。没有重新排序或坐标归一化，保存的就是传给 `calibrate_camera()` 的原图像素坐标。

`x/y` 是便于查看的十进制数，使用17位有效数字。`*_fp32_hex` 是8位IEEE754十六进制位模式，无 `0x` 前缀。FPGA仿真应直接读取十六进制位模式，避免十进制解析造成额外舍入。负零、非规格化数也能原样保存。异常的NaN/Inf十进制栏写为 `null`，位模式仍保留。

## 参数与诊断格式

`calibration.json` 的格式版本为 `closer2fpga.calibration.v1`。

- 顶层记录 width、height、rows、cols、square_size、每阶段迭代上限及 estimate_k3。
- `views` 记录各图原路径、分辨率、是否读取成功、是否检测成功和实际角点数量。
- `calibration_attempted` 表示是否真正调用了标定函数；检测失败导致跳过时，`result=null`。
- `rtl_input_ready` 表示本次输入和配置符合当前RTL接口：三图、每图40个有限角点、5×8棋盘、同分辨率且宽高为2..65535、正有限格长、每阶段150轮、k3不估计。**此字段不代表最终参数有效，也不代替初始化层的坐标范围/几何检查。**
- `result.camera_valid` 表示C++是否允许用本次参数校正图像；即使为false，也保留已计算出的参数及诊断。
- `result.camera` 按 `fx,fy,cx,cy,k1,k2,k3,p1,p2` 保存9个FP32参数，`camera_fp32_hex` 保存对应位模式。
- `result` 同时包含 converged、weak_geometry、accepted_steps、message、总RMS、每图RMS、最大角点误差，以及三组R/t。`accepted_steps` 是最佳候选三个阶段接受更新次数的总和。
- R按行优先存储；t已经恢复到第一个内角点为原点，并乘上square_size。默认格长1，因此平移单位是棋盘格。
- 所有FP64诊断都有对应16位十六进制位模式；非有限十进制数写为JSON `null`。
- `metrics_present` 表示报告包含每张图的RMS和姿态；不是RTL `diag_metrics_valid` 的逐位替代，仍需检查非有限值和计算失败来源。

检测不完整、分辨率不一致、标定不收敛时也会生成新目录。对比时必须使用**同一目录中的角点与参数**，并检查完成标记和状态，不能拿上一轮成功参数配本轮失败角点。

目前导出不包含每个LM中间状态或最佳seed编号，因为原公开结果接口没有这些字段。首次对比先看最终参数、RMS、R/t和状态；如有差异，再增加阶段跟踪定位。

## 本次修改的检查

完整桌面程序通过 `tests/build_demo.cmd` 编译检查。导出回归测试使用专门构造的数据检查序列化，不运行实拍图片：

```bat
tests\run_export_tests.cmd
```

测试覆盖成功、参数被拒绝、检测失败导致跳过、非有限数、目录隔离、JSON字符串转义、写入失败，以及十进制/IEEE位模式往返一致性。默认使用 `E:\vs` 的MSVC和PATH中的Node.js；实际桌面程序不依赖Node.js。
