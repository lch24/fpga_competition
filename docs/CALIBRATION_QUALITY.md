# 标定覆盖、模型选择与画质定位

## 诊断原则

重投影 RMS 衡量模型对已检测角点的拟合程度，边缘映射还受角点覆盖、姿态变化与模型适用范围影响。诊断时同时检查角点位置及顺序、参数收敛、映射有限性与局部翻折。

## 默认模型选择（AUTO_K3=1）

初始化微程序读取角点时，累计所有视图中四个边缘带的点数：

- 左：`x <= (width−1)/4`；右：`x >= 3*(width−1)/4`。
- 上：`y <= (height−1)/4`；下：`y >= 3*(height−1)/4`。
- 每个带至少 4 个角点时，允许完整模型（stage 2）。
- 任一个带不足时，**第一次 LM 就用 stage 3，固定 k3=0**，其余内参、四项畸变和各视图姿态仍联合拟合。
- 完整模型若在最终检查中翻折，保留原来的一次固定 k3 重试。
- 覆盖不足已经使用降阶模型时，失败直接上报，不无限重试。

这是保守的图像空间启发式，不是可辨识性证明；即使覆盖达标，姿态相近、
误检、错误角点顺序、镜头模型不适用等仍可能造成差结果。
它不强制拍超过三张照片、不裁剪已拟合的系数、不提供人工参数答案，
不放宽 RMS、有限性、收敛或映射检查。九参数接口和参数顺序保持不变。

若诊断需要强制复现原完整模型，可在 `calib_top` 实例设置 `AUTO_K3=0`；
只关闭覆盖预选，仍保留原有翻折重试与全部检查。正常整板不要关闭。

## 实现与验证入口

边缘计数通过共享 FP64 服务、微程序和工作 RAM 完成，模型选择由标定控制器执行。对应回归覆盖计数边界、初值与控制器交接、失败清零、重试及 UART 解码。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/calibration/run_quality_checks.ps1 -IcarusBin <含iverilog和vvp的目录>
```

## 串口如何判断

状态查询增加 `low_coverage`（算法快照字节 6 bit7），包长和 CRC 不变。

| reduced_model | low_coverage | 含义 |
|---|---|---|
| false | false | 完整模型 |
| true | false | 完整模型翻折后，固定 k3 的重试 |
| true | true | 覆盖不足，拟合前直接选择固定 k3 |

这些标志不是成功标志；仍看当前任务的 `params_valid`、`last_status`、
`check_reason` 和快照新鲜度。旧 bitstream 的保留位为 0，不能据此反推覆盖足够。

## 上板后的诊断顺序

1. 用新 bitstream，普通模式先测。三张棋盘必须完整入镜，改变倾角和位置，
   尽量让内角点分别覆盖左右及上下区域；不是三张都在中央转动。
2. 标定结束后空闲时导出角点（COM11 按实际端口替换）：

```powershell
python scripts/debug/uart_monitor.py COM11 --once status
python scripts/debug/uart_monitor.py COM11 --corners data/debug/corners.json
python scripts/debug/inspect_calibration.py data/debug/corners.json
python scripts/debug/replay_calibration.py data/debug/corners.json
```

3. 检查 `build/debug/corner_audit/report.json` 和 `view0.svg`～`view2.svg`。
   数字是每张图的角点索引，连线按行排列，黄虚线为覆盖带边界。
   正负方向一致的翻转不视为顺序错误；单点错序、退化格子及非法坐标会提示。
   几何检查通过不表示亚像素定位准确，必须对照原图。
4. 若已取得对应三张 DDR 标定原图，可叠图（只此步骤需要 Pillow）：

```powershell
python scripts/debug/inspect_calibration.py data/debug/corners.json --images raw0.png raw1.png raw2.png
```

必须是同次拍摄、同分辨率、未裁剪的原始标定帧，不能拿之后的预览帧或拍屏替代。
脚本拒绝尺寸不符的图像，不偷偷缩放坐标。现有 UART 导出的是角点，不是整帧图像。
5. 软件完整/降阶结果都不好，先查角点位置、顺序、采集覆盖和模型。
   软件同参数校正正常而 FPGA 不正常，再查九参数槽位、坐标映射、采样和时序。
   保存失败候选、原因、RMS、角点与对应图片，不能只保留一张校正后拍屏。

回归复现可以直接把 `corners.json` 换成 `data/calibration/uart_fold_corners.json`。
所有离线诊断工具只消耗电脑资源，不占 FPGA LUT。


## 拍摄与回归数据

棋盘需要完整入画，三张照片改变倾角和位置，并使内角点覆盖画面左、右、上、下区域。边缘压缩明显时，适当拉近棋盘，保留外框余量并避免强反光。检测阶段失败时先查角点数量和排列；求解阶段失败时再查收敛与映射原因。

固定回归输入位于 `data/calibration`：`uart_fold_corners.json` 覆盖翻折与降阶重试，`uart_coverage_corners.json` 覆盖集中点集，`fisheye_20261022_corners.json` 提供广角实拍角点。用相同输入复算参数，避免把不同拍摄姿态的结果混作同一算法对照。
