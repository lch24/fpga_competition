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

共享内参、畸变与各视图 R/t 一起优化。寄存器保存少量操作数和地址，矩阵、状态及临时量保存在工作 RAM；初值、求解和检查复用同一套浮点资源。stage 2 的活动参数是 27 列：4 个内参、各视图 6 个外参、k1/k2/p1/p2/k3；雅可比按 bank 存储，畸变占 bank 4..8，各视图姿态轴共用 bank 9..14 但行区间互不相交。

需要整批重投影残差时执行 HOST 指令：执行器暂停，`lm_controller` 读取工作区参数，调用共享 `residual_engine`，把返回残差和统计写回 RAM，再恢复程序。这里的 HOST 是 RTL 适配器，不是外部电脑。

## 检查程序

维护源为 `build_validate.py`。程序检查参数范围和有限性，调用残差服务计算最终误差，恢复各图姿态诊断，并检查采样映射的有效性。检查成功后，适配器输出相机参数包和完成状态；R/t 用于标定与诊断，去畸变只使用相机内参和畸变系数。

## 执行器与数值通路

标定指令宽 32 位，有 8 个 A32 地址/整数寄存器和 8 个 F64 数据寄存器。执行流程是同步取指、译码、发起访存或运算、等待返回、写回，然后进入下一条指令。工作 RAM 按 64 位字寻址。

板级 `ENABLE_KERNELS=0`：矩阵算法通过标量指令执行。独立引擎测试仍覆盖可选向量内核；这部分由综合参数裁剪，不在板级实例中。FP64 后端由仲裁池共享，其复杂函数内部由 `fp_math_program` 执行，基础算术由 `calib_alu` 等数据通路完成。

程序 ROM 由 `build_rom.py` 合并，当前有效 4033 条、深度 4096。排列为 LM 1137 条、检查 1160 条、初值 1736 条；实际启动顺序由阶段入口决定。地址和长度以生成的 `calibration_program_defs.vh` 为准。

## 完整畸变模型与形状正则化

求解器与 `map_coord_core.v` 都使用 k1/k2/k3 三个径向项和 p1/p2 两个切向项。
当前生产流程始终用 stage 2 拟合全部参数。角点覆盖仅作为诊断，不关闭 k3；
失败也不自动降阶重试。

`build_lens.py` 在每次 HOST 返回后追加四个形状残差：两个抑制全视场内的
高阶曲率，两个避免更新接近映射退化。它们和角点残差一起参与差分、列归一化
及正规方程。试探更新需要降低总目标并保持全视场映射安全裕量；发布的 RMS
仍只包含角点误差。初值为零畸变，不使用人工镜头预设或其映射表。

每个残差缓冲和雅可比 bank 包含四项形状残差，默认工作 RAM 为 8192×64。
形状计算也使用共享浮点服务。算法公式和精度指标见
[标定质量诊断](CALIBRATION_QUALITY.md)。

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


## 程序源的分工

| 文件 | 维护内容 |
|---|---|
| `workspace.py` | 三阶段共用的标量字地址、scratch 别名和 FP64 常量 |
| `program.py` | 生成加载、存储、算术、分支、循环和 HOST 指令的公共接口 |
| `build_init.py` | 坐标归一化、单应矩阵、Jacobi、姿态初值和覆盖诊断 |
| `build_lm.py` | `initialize → jacobian → normal_equations → trial_step → accept_or_retry → finish` |
| `build_lens.py` | 完整径向/切向模型的曲率与映射安全裕量 |
| `build_validate.py` | 最终参数转换、误差统计、姿态和映射检查 |
| `assemble.py` / `build_rom.py` | 指令编码、阶段入口与合并 ROM |

上述 Python 文件位于 `scripts/calibration/engine`。函数调用组织的是**生成过程**，
板上执行的是生成的指令分支和循环，不会为每个 Python 函数增加硬件模块。
`workspace.py` 的地址顺序与 RTL 初始化宏一致；`rot/h/q` 别名用于互斥阶段。

RTL 适配器按“接口与状态 → 本地程序端口 → 工作区访问 → 共享服务 → 状态推进”排列。
`calib_u16_fp64.vh` 提供三阶段共用的整数转换函数源码，展开后仍是各调用点的组合逻辑。
独立模式保留本地 ROM/执行器用于模块测试；板级选共享模式，采用服务中的唯一 ROM/RAM。

验证入口在[脚本说明](../scripts/README.md)和[标定测试导航](../tb/calibration/docs/README.md)。
