# 初始化层实现与仿真

> 配置更新：本文的3视图、40点、27项状态等具体尺寸描述默认构建；可配置范围与当前接口以 [CONFIGURATION.md](../CONFIGURATION.md) 和 calib_defs.vh 为准。

`rtl/calibration/init` 的四个模块已实现，原有端口名称、宽度及握手规则保持不变。固定输入为3张图，每张5行×8列有序角点；不访问图像或DDR。

输入角点经 `corner_store` 保存后，初始化层产生最多5组27项FP64状态，供后续LM逐组优化。初始畸变系数全部为0；初始化成功并不代表标定已经完成。

## 1. 模块与计算流程

```text
corner_store ──角点只读端口──> init_controller
                                ├─ homography ×3 → 三张H
                                ├─ 视图差异检查
                                ├─ zhang → 内参候选K（允许失败）
                                └─ pose_init → seed输出
                                     ↑
                           Zhang K及四组固定焦距K
```

同名子模块按命令重复调用，不是为三张图各实例化一份。

| 文件 | 实现方法 | 内部保留的数据 |
| --- | --- | --- |
| [homography.v](../rtl/calibration/init/homography.v) | 坐标归一化 → DLT约束外积 → 9阶Jacobi → 反归一化 → H/H[8] | 40组FP64坐标、均值/尺度、81项矩阵 |
| [zhang.v](../rtl/calibration/init/zhang.v) | 图像归一化 → 三图各两组约束 → 6阶Jacobi → 恢复内参并检查范围 | 三张H、36项矩阵、6项曲线系数 |
| [pose_init.v](../rtl/calibration/init/pose_init.v) | K逆变换H → 尺度和深度符号修正 → Gram-Schmidt正交化 → R转旋转向量 | 三张H、当前K、27项待输出状态 |
| [init_controller.v](../rtl/calibration/init/init_controller.v) | 三图H收集 → 重复检查 → Zhang和固定焦距候选 → 逐组姿态初始化及输出 | 三张H、当前K、当前seed、已接收seed数 |

各模块用状态机串行调用自己的 `fp_operator`，一条运算完成后再进入下一步。H和Zhang各有一个 `jacobi_eigen`；姿态模块调用已实现的 `rotation(mode=1)`。RTL不使用 `real`，不依赖厂商浮点IP。

### homography：40点到H

对象坐标固定为 `X=col-3.5, Y=row-2`，单位是棋盘格。

1. 按index=0..39读取一次角点，转换到FP64并缓存。检查有限性和 `0≤x<W, 0≤y<H`，累计均值。
2. 扫描内部缓存，累计图像点到均值的距离以及对象点到原点的距离，得到 `si=√2×40/di`、`sw=√2×40/dw`。
3. 每点先构造u约束行，再构造v约束行，逐项累加9×9矩阵 `AᵀA`。
4. 把81项矩阵按行送入Jacobi核，取最小特征向量。次小特征值小于最大特征值的 `1e-10` 倍时判退化。
5. 计算 `H=T_image⁻¹×H_normalized×T_board`，再除以H[8]；成功时H[8]严格等于1。

距离和小于 `1e-6` 或反归一化后的 `|H[8]|<1e-12` 也判退化。距离用平方和开方实现；图内FP32输入和16位宽高范围使平方不会溢出。软件的 `hypot` 以及完整矩阵乘法可能有不同末位舍入，测试采用数值容差。

### zhang：三张H到内参K

先以W归一化两轴像素坐标，并将图像中心平移至原点。每张H构造 `v12` 和 `v11-v22`，累加6×6约束矩阵，取最小特征向量b。

统一b的符号，使b[0]为正，再恢复主点、lambda、焦距及辅助skew。输出只有fx、fy、cx、cy，skew只参与cx恢复。

沿用C++检查：b[0]>0、分母>1e-14、lambda>0；焦距严格位于 `(0.05W,20W)`，主点距图像中心的偏移分别严格小于W和H。不额外添加软件没有的秩判断。Zhang失败由控制器回退到固定焦距候选。

### pose_init：K和H到27项状态

每张H按列计算K⁻¹H，用前两列范数估计尺度，结合第三列深度统一符号。第一列归一化，第二列减去沿第一列的投影后归一化，第三列取叉乘。由得到的R调用稳定的四元数分支转成旋转向量。

范数小于1e-12、深度非正或算术失败时，整组初值作废。H整体变号可正常恢复正深度。

输出从低位起：

| 槽位 | 内容 |
| --- | --- |
| 0..3 | log(fx)、log(fy)、cx/W、cy/H |
| 4..8 | k1、k2、p1、p2、k3，初始均为0 |
| 9..14 | view0的rx、ry、rz、tx、ty、log(tz) |
| 15..20 | view1的相同6项 |
| 21..26 | view2的相同6项 |

### init_controller：初值枚举和失败处理

先计算三张H。与C++一致，累加H1、H2分别相对H0的前8项绝对差，小于1e-6时拒绝。因此只在三张都重复时直接拒绝，两张重复仍进入后续流程。

| seed_id | 内参来源 |
| --- | --- |
| 0 | Zhang估计成功时尝试 |
| 1 | fx=fy=0.6W |
| 2 | fx=fy=W |
| 3 | fx=fy=1.8W |
| 4 | fx=fy=3W |

固定焦距候选的主点均为 `((W-1)/2,(H-1)/2)`。每组K都调用pose_init；失败则跳过该ID，成功则输出seed。ID可以跳号，不能按接收序号推断ID。

H失败直接结束；Zhang失败继续固定候选；pose失败只跳过当前候选；所有候选都失败时返回 `PAR_CALIB_INVALID`。成功输出至少一个seed才返回 `PAR_OK`。

## 2. 接口时序和共同约定

命令只在 `cmd_valid && cmd_ready` 时接受，宽高、K、H等输入随命令锁存。命令后外部输入可以改变。当前响应被接收前不接受新命令。

`homography`成功任务恰好读取40次；完整三图初始化恰好读取120次。角点读取严格使用已有的固定一拍契约：

| 时刻 | 行为 |
| --- | --- |
| E_n之前 | homography给出rd_en、view、index |
| E_n上升沿 | corner_store采样地址 |
| E_n之后 | corner_store更新rd_valid及x/y |
| E_(n+1)上升沿 | homography采样返回，开始转换/计算 |

读路径无ready、无返回背压，不能额外插流水寄存器；预期返回拍没有valid会报 `PAR_MEM_ERROR`。计算期间父模块必须保持角点缓存不变，不得clear或写入。

`seed_valid && seed_ready` 每次传输一整组状态；背压期间valid、id、state保持。`rsp_seed_count`只统计已握手的seed。最后一个seed被接收并完成候选枚举后才出现rsp；失败不补占位seed。

响应背压期间valid及载荷保持。低有效复位取消父模块和全部子模块事务，valid清零；工作数组不清零，下次事务会先写后读。复位同步释放要求与项目其他模块一致。

| 状态 | 触发情况 |
| --- | --- |
| `PAR_OK` | 计算成功；控制器至少输出一组seed |
| `PAR_BAD_CONFIG` | W/H<2；homography的view_id不在0..2 |
| `PAR_MEM_ERROR` | 固定角点返回拍没有valid |
| `PAR_CALIB_INVALID` | 非有限输入、非法运算/除零/溢出、退化、参数范围不通过等 |

子核错误按上述调度规则传播或跳过。homography/zhang/pose_init失败时数值结果清零。允许有限的下溢和非精确舍入；不保证任意极端浮点输入都与C++逐位一致。

## 3. 运行仿真

所有TB、脚本、向量均在本目录，默认ModelSim路径是 `E:\pangu\Modelsim10.1c\win64`。

运行全部四个TB：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_init.ps1
```

也可分别运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_homography.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_zhang.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_pose_init.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_init_controller.ps1
```

可通过 `-ModelSimBin` 指定其他安装目录。各脚本使用独立 `build_<模块>` 工作库，检查超时、done、用例数、错误数和PASS标记，失败时返回错误。结果在各工作目录的 `*_results.txt`，日志在 `simulation.log`。

测试向量已保存，不需要先运行Node。如修改参考数据，可用：

```powershell
node E:\fpga\parameter\sim\generate_init_vectors.js
```

生成器独立用JavaScript数值实现软件公式，未使用RTL结果生成期望值。为避免只验证移植后的参考代码，另提供C++复核脚本，直接编译项目原有的 initialization.cpp、pose_init.cpp、math3.cpp、symmetric_eigen.cpp：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_init_reference.ps1
```

默认MSVC安装目录 `E:\vs`，可通过 `-VsRoot` 更改。这一步复核了50组可直接调用原算法的场景，0错误；另外51组硬件协议/非法输入场景不直接调用缺少这些前置校验的C++函数，由RTL TB验证。该脚本复核的是参考向量，RTL与同一向量的比较由ModelSim完成。

## 4. 本次验证结果

| TB | 数值/异常场景 | 额外复位恢复场景 | 错误数 | 测得最大响应等待周期 |
| --- | ---: | ---: | ---: | ---: |
| tb_homography.sv | 22 | 4 | 0 | 158013 |
| tb_zhang.sv | 16 | 4 | 0 | 39560 |
| tb_pose_init.sv | 54 | 3 | 0 | 5155 |
| tb_init_controller.sv | 9 | 3 | 0 | 516634 |

共101组场景、14组额外复位恢复场景。周期是本次向量测量值，控制器包含seed背压等待，不是所有输入的最坏上界或综合频率指标。

覆盖包括：透视/仿射H、随机姿态、负号及不同尺度H、常量点/共线点退化、非有限数据、图像边界、非法宽高/view、缺失首次/末次读取、重复视图、候选跳过、所有候选失败、命令锁存、响应/seed背压、计算中及待发送结果时复位、复位后重新完成任务。

控制器TB直接实例化真实corner_store及全部init子模块，按三图角点流和视图完成响应写入缓存。9组中有3组仅强制子模块完成状态，以确定性验证Zhang回退、指定pose跳过及所有pose失败；其他场景使用真实计算状态，包括自然出现的Zhang失败。强制状态只存在于TB，不在RTL中。

误差标准：每项 `|实际-参考| ≤ 容差×(1+|参考|)`；H为2e-8，K为2e-7，单独pose状态为2e-9，完整控制器状态为3e-7。异常状态、ID、数量、valid时序和背压稳定性按精确值检查，额外拒绝未知态及NaN/Inf。

目前完成的是功能仿真及初始化层联调。尚未综合评估寄存器/RAM推断、资源占用和关键路径，也未接入完整LM或顶层。每个算法模块各自复用局部运算器，目前没有全局浮点资源仲裁。
