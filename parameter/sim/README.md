# 标定模块仿真

新增多配置仿真入口：[run_configurable.ps1](run_configurable.ps1)，覆盖不同视图数/棋盘行列数，方法与范围见 [CONFIGURATION.md](../CONFIGURATION.md)。下列历史测试数字对应默认3×5×8配置。

浮点算术模块的实现和独立测试见 [FP_OPERATOR.md](FP_OPERATOR.md)。
高斯求解模块的实现和独立测试见 [GAUSS_SOLVER.md](GAUSS_SOLVER.md)。
对称特征分解模块的实现和独立测试见 [JACOBI_EIGEN.md](JACOBI_EIGEN.md)。
投影模型层四个模块的实现、接口时序及独立测试见 [MODEL.md](MODEL.md)。
初始化层四个模块、corner_store联调及C++参考复核见 [INIT.md](INIT.md)。
LM层四个模块、真实残差核联调及原C++优化结果对比见 [LM.md](LM.md)。
最终参数检查、误差诊断、姿态恢复及C++参考对比见 [VALIDATE_RESULT.md](VALIDATE_RESULT.md)。
顶层任务调度、45组控制场景和完整真实RTL链路的运行方法见 [CALIB_TOP.md](CALIB_TOP.md)。
使用实拍图片角点的1280×720端到端对比结果见 [REAL_CALIBRATION.md](REAL_CALIBRATION.md)，全部50项数值检查通过。

实现文件：[corner_store.v](../rtl/control/corner_store.v)。测试平台：[tb_corner_store.sv](tb_corner_store.sv)，SystemVerilog编写，不需要厂商RAM仿真库。

在PowerShell中运行（可从任意目录启动）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_corner_store.ps1
```

默认使用本机 `E:\pangu\Modelsim10.1c\win64`。其他安装位置可通过脚本参数 `-ModelSimBin` 指定。脚本编译RTL与TB，运行110 us，检查完成标志、检查计数和错误计数；超时或任何检查失败均返回非零退出码。

最近一次验证：**ModelSim SE-64 10.1c，21组场景、3240次检查、0错误**。

## 实现行为

- 120×64位RAM同时存储x/y，每张图40点。RAM不整体复位；有效性由状态和计数控制。FP32只保存位模式并检查NaN/Inf，不进行浮点计算；负零和有限非规格化数可保存。
- 图号0→1→2，每图必须收到完成通知后才允许下一图。每图点号0→39，只有39号点last=1。错误点不写RAM、不增加计数，格式错误保持到clear。
- 成功通知须对应40个完整点；失败通知不等待点数补足。原始检测状态与缓存格式错误分开保存，done/usable含义不同。
- 同图角点和通知同时有效时，先接收角点，通知ready暂时拉低；发送方须保持通知，下一拍接收。已done视图的重复输入被背压，不覆盖结果。
- clear优先级高于写入、通知和读取。清空只更新控制状态；旧RAM数据不会通过合法读口暴露。
- 读地址由图号和点号构成：在E_n上升沿采样使能与地址，数据及rd_valid在该沿后更新，调用方在E_(n+1)采样；无返回ready。非法/未提交图读取不产生rd_valid，正常调用方仍须保证地址合法。
- 非法图号不截断、不握手，顶层应报BAD_CONFIG。即使某张图失败，其他图的缓存状态也保留；是否停止整次任务由calib_top决定。

## 覆盖范围

| 场景 | 检查内容 |
| --- | --- |
| 4次完整任务 | 每次3×40点，伪随机输入空拍和有限FP32数据；遍历全部120点的乱序读回逐位比对 |
| 读时序 | 请求前无组合返回，一拍数据/valid，下一拍撤销valid；读取不改变视图状态 |
| 每图完成与失败 | 点数不足、0点失败、早到成功、失败图不影响先前成功图 |
| 数据格式错误 | 错误点号、越界点号、提前/缺失last、NaN、正负Inf、多余第41点 |
| 协议边界 | 重复完成通知、done后写入、图像乱序、非法图号、最后一点与完成通知同拍 |
| 清空与复位 | 并发输入时clear优先；清空/异步复位取消读有效；禁止读取旧任务；复位后再次收集 |

检查使用期望坐标数组和显式状态断言，不依赖DUT内部RAM层次。伪随机序列固定种子，结果可重现。非法读测试用于验证防御行为，不表示正常调用者可以任意读未提交数据。

## 输出文件

运行结果在 `sim/build/`（已加入.gitignore）：

- `results.txt`：逐场景记录、失败位置及统计。
- `simulation.log`：ModelSim日志和 `CORNER_STORE_PASS` / `CORNER_STORE_FAIL`。
- `corner_store.wlf`：波形，可用ModelSim打开查看。
- `work/`：编译库。

本机ModelSim退出时仍可能打印已有的 `FileWatch(fileName)` Tcl错误；当前仿真完成、检查全通过且进程退出码为0。脚本同时检查退出码和通过标记，避免把失败仿真当作成功。

当前仅完成RTL功能仿真。RAM使用可综合的同步读模板；是否推断为目标器件的块RAM、资源占用和时序收敛仍需在厂商综合工具中确认。本页统计只对应corner_store，其他模块的验证结果见上方各专题文档。
