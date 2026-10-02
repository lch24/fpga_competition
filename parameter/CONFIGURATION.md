# 标定模块配置

统一修改 [rtl/common/calib_config.vh](rtl/common/calib_config.vh)，不要分别修改各模块的数组和循环。默认仍是 **3 张图、5 行 × 8 列内角点**，这是默认配置，不是算法固定尺寸。行列数指内角点数，不是棋盘方格数。

| 配置 | 默认 | 支持范围 | 用途 |
| --- | ---: | --- | --- |
| `PAR_VIEWS` | 3 | 3～16 | 一次标定收集的视图数 |
| `PAR_BOARD_ROWS` | 5 | ≥2 | 内角点行数 |
| `PAR_BOARD_COLS` | 8 | ≥2，行×列≤256 | 内角点列数 |
| `PAR_LM_MAX_ITERS` | 150 | 1～255 | 每个候选、每个 LM 阶段的外迭代上限 |
| `PAR_LM_MAX_TRIES` | 16 | 1～256 | 一轮线性化中阻尼求解的最多尝试次数 |

例如改为 5 张、7×10 棋盘，只需修改前三项为 `5、7、10`。所有 RTL、TB 和接口调用方必须使用相同配置，并重新编译/综合。也可以通过 ModelSim 的 `+define+PAR_VIEWS=5+PAR_BOARD_ROWS=7+PAR_BOARD_COLS=10` 覆盖默认值。`calib_top` 对范围外配置在展开阶段报错。

这些是**综合期配置**，决定 RAM 容量、矩阵规模和总线位宽；同一个比特流不能按任务切换它们。图像 `cmd_width/cmd_height`（2～65535）、`cmd_square_size_fp64`（有限正数）和任务号仍是运行时命令。若以后需要在同一个比特流中切换棋盘规格，应另加“最大容量＋本次有效数量”的命令接口。

## 自动推导的内容

设视图数为 V，行数 R，列数 C，P=R×C：

| 项目 | 实际尺寸/规则 |
| --- | --- |
| 角点 RAM | V×P 个二维 FP32 点 |
| 角点顺序 | `point_index=row*C+col`，每图 0～P−1；按 view 递增收集 |
| 物方坐标 | `X=col-(C-1)/2`，`Y=row-(R-1)/2`，Z=0，单位格 |
| 残差数量 | 2×V×P；索引 `2*(view*P+point)+component` |
| LM 状态 | 9+6×V 个 FP64；9 个相机状态槽＋每图 6 个外参 |
| 活动参数数 | 阶段 0：4+6×V；阶段 1：5+6×V；阶段 2：8+6×V |
| 雅可比 RAM | `(2×V×P) × (8+6×V)` 个 FP64 |
| 高斯消元 RAM | `N×(N+1)` 个 FP64，N=8+6×V |
| 单应矩阵集合 | V×9 个 FP64 |
| 结果姿态 | V×12 个 FP64，各图按 R 的9项、t的3项打包 |
| 图像误差 | V 个 RMS，总 RMS 分母为 V×P |

DLT 均值、Hartley 归一化、棋盘中心、重复视图检查、所有视图对的法向夹角、物理平移恢复均使用这些尺寸。增加视图时临时寄存器区也会后移，避免覆盖新增外参。LM 活动列映射由 [calib_lm_layout.vh](rtl/common/calib_lm_layout.vh) 统一提供。

固定的算法模型仍为 Brown 畸变、k3=0、零 skew，以及 Zhang＋4组焦距候选、3阶段 LM。配置棋盘规格不改变这些模型假设。增加点数/视图数不保证数据可标定；退化视图和不收敛结果仍会被拒绝。

## 对接规范

- 输入 `corner_view_id`、`view_rsp_view_id` 和 `corner_point_index` 仍为 8 位；`corner_last` 仅在当前图 P−1 号点为 1。
- 每图必须收到恰好 P 个有序点及一笔成功检测响应；全部 V 图成功才接受标定命令。任何一图失败均作废本次任务。
- 内部读口位宽、调试总线和诊断总线随配置变化，调用方应包含 [calib_defs.vh](rtl/common/calib_defs.vh)，不要复制默认位数。
- `dbg_view_done/format_error/usable` 各 V 位，`dbg_view_status` 为 8×V 位；计数每图占 `PAR_POINT_BITS=$clog2(P+1)` 位，包含“已收满 P 点”。view0在低位。
- `diag_view_rms_fp64` 为 64×V 位，`diag_poses_fp64` 为 768×V 位。成功输出的相机参数包始终为 9×32 位，字段顺序不变。
- 所有视图来自同一相机、同一分辨率、同一棋盘规格。检测方、标定方及任务调度方必须统一上述配置；校正方不需要 R/t，也不依赖棋盘行列数。

配置范围是本实现的接口/容量边界，不是器件资源承诺。尤其雅可比 RAM 随点数和视图数增长，高斯计算量也随活动参数数增加；尚未进行目标器件综合和时序验证。

## 仿真方法

在 `E:\fpga` 执行：

```powershell
# 独立解析针孔模型生成角点，测试缓存、残差、初始化、一次真实LM更新及结果校验
powershell -NoProfile -ExecutionPolicy Bypass -File parameter/sim/run_configurable.ps1 -Views 5 -Rows 7 -Cols 10

# 非8列、行列奇偶性相反的配置
powershell -NoProfile -ExecutionPolicy Bypass -File parameter/sim/run_configurable.ps1 -Views 4 -Rows 6 -Cols 7

# 真实顶层：最后一图检测失败、重复视图拒绝、完整五候选三阶段任务
powershell -NoProfile -ExecutionPolicy Bypass -File parameter/sim/run_configurable.ps1 -Views 4 -Rows 6 -Cols 7 -TopOnly

# 最大容量；跳过大矩阵LM，测试其余数值链路
powershell -NoProfile -ExecutionPolicy Bypass -File parameter/sim/run_configurable.ps1 -Views 16 -Rows 16 -Cols 16 -SkipLm
```

新配置测试默认仅 **1 次外迭代、3 次阻尼尝试**，目的是核对维度、数值和迭代上限，不能把这个测试上限当作实际标定的推荐设置。可用 `-MaxIterations/-MaxTries` 指定。顶层测试允许因迭代上限返回未收敛，但必须完成全部候选、输出正确诊断并通过几何误差检查。

`generate_config_vectors.js` 独立按 Rodrigues 和针孔投影公式生成参考，不使用 RTL 结果，也不伪装成原 C++ 标定结果。每个构建目录有 `fixture.json`、`simulation.log` 和 `config_results.txt` 或 `config_top_results.txt`。解析模型的零畸变数据用于隔离尺寸问题；原默认配置的畸变/异常场景由现有回归继续覆盖。

真实图片回放仍使用 `run_calib_top.ps1 -ExportDirectory <导出目录>`，现在会从 JSON 提取视图数、行列数、外迭代上限，同时配置 RTL 和 TB。它重新检查实际文件，不再以旧导出中仅面向三张5×8的 `rtl_input_ready` 标志作为唯一依据；阻尼尝试保持 C++ 的16次，k3仍需为0。原合成/控制测试向量只适用于默认配置，配置不符会明确报错。

此前 [实拍对比报告](sim/REAL_CALIBRATION.md) 是默认3×5×8配置的历史记录；不能将那次“50项通过”当作任意配置已经与实拍 C++ 全量对比的证明。

## 本次验证记录（2026-09-30）

| 配置 | 验证内容 | 结果 |
| --- | --- | --- |
| 3张2×2；迭代1、尝试1 | 缓存、DLT/Zhang/姿态、残差、26参数LM、结果校验 | 0错误；LM代价3.88048→0.000140586 |
| 4张6×7；迭代1、尝试3 | 同上，32参数LM，奇数列/偶数行 | 0错误；LM代价50.9929→0.00240811 |
| 5张7×10；迭代1、尝试3 | 同上，38参数LM，角点索引超过63，残差700项 | 0错误；LM代价105.429→0.00528532 |
| 16张16×16 | 4096点缓存、初始化、8192残差、全部姿态/RMS；跳过LM | 0错误；10,820,236周期 |
| 4张6×7，真实calib_top | 无子核模拟替换；最后视图失败、重复视图拒绝、完整五候选三阶段 | 3任务0错误；合计79,336,273周期 |

顶层完整任务最佳seed=0，接受3步，RMS约1.80939e-5像素；由于每阶段最多1轮，最终converged=0，正确返回CALIB_INVALID并保留诊断、不发布相机参数。这个测试同时验证了可配置迭代上限的语义。

默认3×5×8回归：初始化101场景＋14协议/复位场景、LM83场景、结果验证66场景＋6协议/复位场景、顶层45控制场景，均0错误。未重新运行耗时较长的默认实拍539,535,222周期整链路；历史实拍报告保持原样。

额外检查：`test_config_limits.ps1` 的9种非法配置均被RTL展开拒绝；`node parameter/sim/test_calib_real_data.js` 验证350点打包、76字段比较，以及错误数值/点序/规格拒绝（文件格式模拟测试，不是标定结果）。真实文件回放TB在5张7×10下编译、展开成功。
