# 当前标定执行架构

## 从角点到参数

`calib_top` 收齐各视图的角点和检测状态后，依次启动初值、单阶段 LM、结果检查。三个阶段通过 `calib_execution_port` 访问一个 `calib_execution_service`，共享程序 ROM、`calib_sequencer`、工作 RAM 和浮点请求接口。

```text
corner_store → init_controller → lm_controller → validate_result → 参数/诊断
                       ↘              ↓              ↙
                         calib_execution_service
                        ROM + sequencer + workspace
                                   ↓
                             FP64 共享运算池
```

箭头表示阶段顺序，三个适配器都连接同一个执行服务。当前阶段占有工作区，结束并读回结果后交给下一阶段。

## 初值程序

维护源为 `scripts/calibration/engine/build_init.py`，姿态子程序在 `data/programs/calibration/pose.asm` 和 `rotation_inverse.asm`。

程序逐图读取角点，归一化棋盘与像素坐标，累加 DLT 的 9×9 对称矩阵，用 Jacobi 旋转求最小特征向量，再反归一化得到 H。随后以图宽作为 fx/fy 初值、图像中心作为主点，从 H 恢复各图姿态，畸变初值为零。只输出一组初值。

Jacobi 搜索、旋转、排序和姿态计算都是指令循环；不再调用独立的 `homography`、`zhang`、`jacobi_eigen` 硬件。C++ 当前用 8 元线性求解代替 DLT/Jacobi，RTL 这里仍保留已经验证的数值方法。

## LM 程序

维护源为 `build_lm.py` 和 `data/programs/calibration/solve.asm`。程序计算基准残差，对活动参数做正负扰动的中心差分，累积正规方程，加入阻尼，执行带主元选择的消元和回代，得到参数增量。试探状态使代价下降时接受，否则增加阻尼重试；依据收敛条件和迭代上限退出。

共享内参、畸变与各视图 R/t 一起优化，当前 k3 固定。寄存器保存少量操作数和地址，矩阵、状态及临时量保存在工作 RAM；初值、求解和检查复用同一套浮点资源。

需要整批重投影残差时执行 HOST 指令：执行器暂停，`lm_controller` 读取工作区参数，调用共享 `residual_engine`，把返回残差和统计写回 RAM，再恢复程序。这里的 HOST 是 RTL 适配器，不是外部电脑。

## 检查程序

维护源为 `build_validate.py`。程序检查参数范围和有限性，调用残差服务计算最终误差，恢复各图姿态诊断，并检查采样映射的有效性。检查成功后，适配器输出相机参数包和完成状态；R/t 用于标定与诊断，去畸变只使用相机内参和畸变系数。

## 执行器与数值通路

标定指令宽 32 位，有 8 个 A32 地址/整数寄存器和 8 个 F64 数据寄存器。执行流程是同步取指、译码、发起访存或运算、等待返回、写回，然后进入下一条指令。工作 RAM 按 64 位字寻址。

板级 `ENABLE_KERNELS=0`：矩阵算法通过标量指令执行。独立引擎测试仍覆盖可选向量内核；这部分由综合参数裁剪，不在板级实例中。FP64 后端由仲裁池共享，其复杂函数内部由 `fp_math_program` 执行，基础算术由 `calib_alu` 等数据通路完成。

程序 ROM 由 `build_rom.py` 合并，当前有效 3704 条、深度 4096。排列为 LM 713 条、检查 1343 条、初值 1648 条；实际启动顺序由阶段入口决定。地址和长度以生成的 `calibration_program_defs.vh` 为准。

## 畸变模型与鱼眼适配

求解器的投影子程序 `project.asm` 只做**两个径向项**（`F(t)=1+k1t+k2t²`，t=r²）加切向 p1/p2；重映射核 `map_coord_core.v` 已经算**三个**径向项（多一个 k3，见 `S_K3R6`），但求解器把 k3 固定为零。两侧阶数并不一致，换鱼眼前要先对齐。

离线拟合（等距鱼眼 ρ=fθ、校正视图覆盖整个视场、1280×720）：

| 总视场 | 两阶 `1+k1t+k2t²` | 三阶 `+k3t³` | 四阶 `+k4t⁴` |
|---|---|---|---|
| 90° | 3.24 | 0.48 | 0.07 |
| 100° | 6.95 | 1.30 | 0.25 |
| 120° | 29.1 | 8.33 | 2.47 |
| 140° | 119 | 50.2 | 21.9 |

单位是原始图像里的最大像素偏差；折算到输出图像后，120° 三阶只剩 0.05 像素。脚本：`scripts/calibration/fisheye_model_fit.py`。

**结论：不需要 Kannala-Brandt，也不需要引入 atan。** 现有径向多项式就能覆盖鱼眼，代价是让求解器启用 k3（与 `map_coord_core` 同阶）、并把范围/初值放宽到 k1≈−0.33、k2≈+0.19、k3≈−0.11 这一量级；120° 以内三阶足够，更宽再考虑 k4。改动集中在微码参数布局（`project.asm`、`build_lm.py`、`build_validate.py`）和 ROM 重新生成——`map_coord_core.v` 的 k3 通路已经存在。

主要风险是**微码参数向量与 RTL 工作区/发布参数之间的布局必须一次对齐**；这类错位不会被现有 Python 位精确检查发现，只能在板上用真实照片暴露。

## 文件与维护入口

| 文件 | 职责 |
|---|---|
| `rtl/control/calibration/calib_top.v` | 任务、阶段、参数及诊断发布 |
| `init/init_controller.v`、`lm/lm_controller.v`、`check/validate_result.v` | 阶段输入输出和 HOST 服务 |
| `rtl/compute/service/calib_execution_port.v` | 共享/独立服务接口和入口 PC 重定位 |
| `rtl/compute/service/calib_execution_service.v` | 合并 ROM 和执行服务实例 |
| `rtl/compute/service/calib_datapath.v` | 执行器、工作 RAM、运算接口连接 |
| `rtl/control/calibration/engine/calib_sequencer.v` | 取指、译码、多周期执行 |
| `rtl/memory/local/calib_workspace.v` | 同步工作存储 |
| `scripts/calibration/engine` | 汇编器、程序生成、数值与协议验证 |

取指状态、指令编码、HOST 时序和调试信号见[指令控制指南](INSTRUCTION_CONTROL_GUIDE.md)。

## 验证入口与资源证据

`check_init.py`、`check_lm.py`、`check_validate.py` 验证当前共享执行服务；`check_lm_service.py` 检查真实残差服务联调。脚本与输入来源见[测试说明](../tb/calibration/docs/README.md)。

检测统一控制和共享加减的局部综合对照为：排序 3665→2705 LUT，合并 2732→1190，精修 1326→600；新增共享加法器池 572 和流程控制 204 LUT，分块合计净减约 2452。对应报告位于本地 `build/system/partition_pds_1791611493`、`1791611868`、`1791612016`。这些是局部实测，当前整板是否满足 66,600 LUT 和 40 MHz 仍以整板结果为准。
