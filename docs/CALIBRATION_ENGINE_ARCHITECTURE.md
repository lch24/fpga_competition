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

维护源为 `build_lm.py` 和 `data/programs/calibration/solve.asm`。程序计算基准残差，对活动参数做单侧（前向）差分：只调用一次 +h 扰动的残差服务，与该状态的基准残差相减；每列由两次服务降为一次。累积正规方程，加入阻尼，执行带主元选择的消元和回代，得到参数增量。试探状态使代价下降时接受，否则增加阻尼重试；收敛判据包含步长、梯度，以及“本步代价下降小于当前代价的 1e-6”这一相对阈值，避免在代价已到噪底后继续做无意义迭代。

共享内参、畸变与各视图 R/t 一起优化。寄存器保存少量操作数和地址，矩阵、状态及临时量保存在工作 RAM；初值、求解和检查复用同一套浮点资源。stage 2 的活动参数是 27 列：4 个内参、各视图 6 个外参、k1/k2/p1/p2/k3；雅可比按 bank 存储，畸变占 bank 4..8，各视图姿态轴共用 bank 9..14 但行区间互不相交。

需要整批重投影残差时执行 HOST 指令：执行器暂停，`lm_controller` 读取工作区参数，调用共享 `residual_engine`，把返回残差和统计写回 RAM，再恢复程序。这里的 HOST 是 RTL 适配器，不是外部电脑。

## 检查程序

维护源为 `build_validate.py`。程序检查参数范围和有限性，调用残差服务计算最终误差，恢复各图姿态诊断，并检查采样映射的有效性。检查成功后，适配器输出相机参数包和完成状态；R/t 用于标定与诊断，去畸变只使用相机内参和畸变系数。

## 执行器与数值通路

标定指令宽 32 位，有 8 个 A32 地址/整数寄存器和 8 个 F64 数据寄存器。执行流程是同步取指、译码、发起访存或运算、等待返回、写回，然后进入下一条指令。工作 RAM 按 64 位字寻址。

板级 `ENABLE_KERNELS=0`：矩阵算法通过标量指令执行。独立引擎测试仍覆盖可选向量内核；这部分由综合参数裁剪，不在板级实例中。FP64 后端由仲裁池共享，其复杂函数内部由 `fp_math_program` 执行，基础算术由 `calib_alu` 等数据通路完成。

程序 ROM 由 `build_rom.py` 合并，当前有效 3704 条、深度 4096。排列为 LM 713 条、检查 1343 条、初值 1648 条；实际启动顺序由阶段入口决定。地址和长度以生成的 `calibration_program_defs.vh` 为准。

## 畸变模型与鱼眼适配

两侧阶数现在一致：求解器与 `map_coord_core.v` 都算三个径向项。k3 从零初值能否收敛、以及 fx 与 k1 的相关性（单视角下严重，三视角下可分辨但仍弱）由合成鱼眼数据验证（`scripts/calibration/engine/generate_lm_fisheye.py` + `tb/calibration/tb_lm_fisheye.sv`）。

离线拟合（等距鱼眼 ρ=fθ、校正视图覆盖整个视场、1280×720）：

| 总视场 | 两阶 `1+k1t+k2t²` | 三阶 `+k3t³` | 四阶 `+k4t⁴` |
|---|---|---|---|
| 90° | 3.24 | 0.48 | 0.07 |
| 100° | 6.95 | 1.30 | 0.25 |
| 120° | 29.1 | 8.33 | 2.47 |
| 140° | 119 | 50.2 | 21.9 |

单位是原始图像里的最大像素偏差；折算到输出图像后，120° 三阶只剩 0.05 像素。脚本：`scripts/calibration/fisheye_model_fit.py`。

**结论：不需要 Kannala-Brandt，也不需要引入 atan。** 现有径向多项式就能覆盖鱼眼，做法是让求解器启用 k3（与 `map_coord_core` 同阶）。已核对：**求解器与硬件检查里没有任何 k1/k2/k3 的量级限制**——校验程序只约束焦距（`build_lm.py`：0.05～20 × 宽度）和主点（0～宽/高），`sample_coord` 只查有限性与边界，`packed_map_builder`/`map_coord_core` 也没有系数范围门。所以鱼眼量级系数（k1≈−0.33、k2≈+0.19、k3≈−0.11）不会被拒。120° 以内三阶足够，更宽再考虑 k4。

k3 已计入 stage 2 活动参数（`PAR_ACTIVE_N` 26→27），ROM 条数不变：LM 列数由 RTL 的 `columns(stage)` 提供，汇编不展开 n。微码侧只需把 `project.asm` 的 A0 向量由 8 词扩到 9 词（`fx,fy,cx,cy,k1,k2,k3,p1,p2`），RTL 侧的发布槽位本来就把 `state[8]` 放在 camera 的第 6 槽，未发生平移。

合成鱼眼数据（`generate_lm_fisheye.py`）给出了这两点的答案：k1≈−0.33、k2≈+0.19、k3≈−0.11、视场半径约 0.75 时，从"畸变为零"的初值出发，独立参考解把三阶畸变恢复到真值的 2×10⁻⁷ 以内；同一批数据若不启用 k3，代价是 647 对 1.3×10⁻⁸，说明这批数据确实能区分 k3。窄视场（针孔）fixture 则相反：k3 不可分辨，拟合会把它吸收到约 −2×10⁻⁴，因此相关断言改成"估计且量级很小"。板上真鱼眼标定仍是最后的签核项。

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

`check_init.py`、`check_lm.py`、`check_validate.py` 验证当前共享执行服务；`check_lm_service.py` 检查真实残差服务联调；`run_lm_fisheye.ps1` 用合成鱼眼角点验证三阶畸变能从零初值收敛。脚本与输入来源见[测试说明](../tb/calibration/docs/README.md)。

检测统一控制和共享加减的局部综合对照为：排序 3665→2705 LUT，合并 2732→1190，精修 1326→600；新增共享加法器池 572 和流程控制 204 LUT，分块合计净减约 2452。对应报告位于本地 `build/system/partition_pds_1791611493`、`1791611868`、`1791612016`。这些是局部实测，当前整板是否满足 66,600 LUT 和 40 MHz 仍以整板结果为准。
