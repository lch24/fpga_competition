# 最终结果检查与仿真

> 配置更新：本文的3视图、40点、27项状态等具体尺寸描述默认构建；可配置范围与当前接口以 [CONFIGURATION.md](../CONFIGURATION.md) 和 calib_defs.vh 为准。

[validate_result.v](../rtl/calibration/check/validate_result.v) 已实现，保留原端口名称和位宽。它接收顶层选出的最佳 LM 状态，生成相机参数和诊断，并判断是否允许发布给图像校正模块。它不重新优化、不选择其他 seed，也不访问 DDR。

## 数据与计算流程

1. 锁存 27 项 FP64 状态、图像尺寸、棋盘格长、best_cost 和最终阶段的收敛标志。
2. 解码相机参数：`fx=exp(p0)`、`fy=exp(p1)`、`cx=p2*W`、`cy=p3*H`，连同畸变系数量化成 FP32。
3. 使用原始 FP64 状态调用真实 residual_engine，读取三图共 120 个角点，缓存 240 项残差。失败响应可提前结束，不等待补齐数据。
4. 统计角点误差、每图 RMS 和最大误差；总 RMS 使用输入 `best_cost`。
5. 将各图旋转向量转成 R，恢复平移原点和实际尺寸，计算三对棋盘法向的最大夹角。
6. 检查收敛、内参、RMS 和姿态变化；通过后遍历 `33×25=825` 个位置检查畸变映射。

相机参数从低位起依次为 `fx,fy,cx,cy,k1,k2,k3,p1,p2`，每项 FP32。注意内部状态中畸变槽是 `k1,k2,p1,p2,k3`，输出时已经重新排列。

**两种精度不能混用：** 误差统计和姿态使用原始 FP64 状态；内参范围与映射检查使用实际输出的 FP32 参数，再精确提升到 FP64 运算。这对应原项目 `report.cpp` 的行为，也能拒绝量化后越界的参数。

## 误差和姿态的定义

每个角点的误差是二维距离 `e=hypot(du,dv)`，最大误差为 `max(e)`。模块用缩放形式计算 hypot：设 `a=max(abs(du),abs(dv))`、`b=min(...)`，非零时计算 `a*sqrt(1+(b/a)^2)`，两者为零时直接输出零。

- 总 RMS：`sqrt(best_cost/120)`。
- 第 v 图 RMS：`sqrt(sum(e²)/40)`。
- 分母按角点数计算，不能误用 240 个残差分量作为总 RMS 分母。
- 与 C++ 一样，本模块信任输入 best_cost 属于该状态，不检查它与重算 cost 是否相等。

姿态输出低位开始为 view0、view1、view2。每个视图含行优先 R 的 9 项，再接 t 的 3 项，全部 FP64。平移由内部“棋盘中心、单位格长”恢复为“首内角点原点、实际格长”：

`t_report = (t_internal - 3.5*R[:,0] - 2*R[:,1]) * square_size`

`t_internal.z=exp(logtz)`。这里的 3.5 和 2 分别来自固定 8 列、5 行角点。

## 可用性条件

必须全部满足：

| 检查 | 条件 |
| --- | --- |
| 最终阶段收敛 | cmd_converged 为 1 |
| 焦距 | `0.05*W < fx,fy < 20*W`，两者均按 W 限定，与 C++ 一致 |
| 主点 | `0<=cx<W` 且 `0<=cy<H` |
| 总 RMS | 严格小于 3 像素 |
| 姿态变化 | 棋盘法向最大夹角严格大于 0.01 弧度 |
| 映射规则性 | 所有 825 个采样点满足下述条件 |

网格覆盖输出图像范围 `[0,W-1]×[0,H-1]`，按行遍历。先通过相机内参转成归一化坐标，再求 Brown 畸变映射的二维 Jacobian：

`J = [[a,b],[b,d]]`

要求 `a>0`、`d>0`、`a*d-b*b>1e-4`，且计算有限。遇到首个失败点即可结束。它是网格采样检查，不是对所有连续图像位置的数学证明。

法向来自 R 的第三列。每对法向夹角使用 `acos(clamp(abs(dot(n_i,n_j)),0,1))`，取三对中的最大值。weak_geometry 沿用“最大夹角小于 0.17 或视图数小于 5”的定义，固定三视图下始终为 1，但它不会单独否定 camera_usable。

## 接口和错误行为

命令握手锁存全部输入；响应被接收前不接受下一条命令。响应背压时所有字段保持。角点读口沿用固定一拍返回契约，模块每次只留一笔在途请求。计算过程中，上层不得修改或清空角点缓存。

| 结果 | status | camera_usable | metrics_valid |
| --- | --- | --- | --- |
| 所有检查通过 | OK | 1 | 1 |
| 诊断计算完成，但收敛、内参、RMS、姿态或映射不合格 | CALIB_INVALID | 0 | 1 |
| 状态非有限、残差/姿态/统计计算失败、FP32 转换溢出 | CALIB_INVALID | 0 | 0 |
| 宽高、格长或 cost 非法；残差流格式错误 | BAD_CONFIG | 0 | 0 |
| 角点缺少固定一拍返回 | MEM_ERROR | 0 | 0 |

格长必须正且有限，cost 必须非负且有限；cost 的负零按零处理。上游状态错误与配置错误同时存在时，配置错误优先。

相机参数只在 camera_usable 为 1 时输出，否则为零。metrics_valid 为 0 时，RMS、最大误差、姿态和 weak_geometry 全部屏蔽为零；为 1 时，即使参数不可用，诊断仍有效。最终状态字段由 rsp_valid 限定。

复位取消所有在途事务。子模块取消使用寄存器驱动的复位信号，避免多位状态译码直接驱动异步复位。数组不整体复位，每次使用前重新写入。

## 测试与运行

最终版本在 ModelSim SE-64 10.1c 中通过 **66 组向量 + 6 组额外协议/复位恢复场景，0 错误**。测试中完整可用结果的最大响应等待时间为 **408,979 周期**；未收敛等提前拒绝场景约 6.1 万周期。周期随数据和退出位置变化，这不是所有输入的最坏上界，也不是综合频率承诺。

```powershell
# 使用已保存的参考向量运行完整RTL测试
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_validate_result.ps1

# 可选：重新编译原C++报告函数并生成参考向量
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_validate_reference.ps1
```

ModelSim 路径可通过 `-ModelSimBin` 指定，MSVC 路径可通过 `-VsRoot` 指定。参考程序以 `/fp:strict /utf-8` 编译，直接调用原项目的 `finish_result()` 和 `residuals()`。

主要文件：

| 文件 | 内容 |
| --- | --- |
| [tb_validate_result.sv](tb_validate_result.sv) | 实例化真实检查模块、残差核、旋转与浮点核；一拍角点存储模型 |
| [generate_validate_reference.cpp](generate_validate_reference.cpp) | C++ 场景与参考结果生成 |
| [validate_result_vectors.txt](validate_result_vectors.txt) | 输入状态、角点、参考相机参数与诊断 |
| [validate_result_cases.txt](validate_result_cases.txt) | 逐场景名称和预期状态 |
| run_validate_result.ps1 / .do | 编译、运行、检查场景数和错误数 |
| run_validate_reference.ps1 | 编译并运行 C++ 参考程序 |

66 个向量中，48 个调用原 C++ 报告函数生成参考，18 个验证硬件接口或数值失败契约，不调用缺少相应前置保护的报告函数。另有 6 个协议与恢复场景。

覆盖正常参数、不同格长、各图不等的残差、9 项相机参数排列、RMS=3 边界、焦距上下界、量化后越界、主点边界、姿态差不足、径向/切向映射异常、映射行列式 1e-4 两侧、NaN、FP32 转换和姿态平移溢出、缺失角点。额外检查命令锁存、全部 825 点遍历、响应背压、计算/映射/响应期间复位、残差流格式错误、旋转失败和后续任务恢复。

相机参数按 FP32 位模式精确比较，状态码和标志精确比较。41 项 FP64 诊断（总 RMS、三个分图 RMS、最大误差、36 项姿态）使用 `2e-9*(1+abs(reference))` 容差，拒绝未知态和非有限值。

仿真使用合成角点及存储模型，不是完整摄像头链路，也未实例化 corner_store。尚未进行目标器件综合、资源与时序验证；calib_top 的任务/seed/阶段调度仍待实现。
