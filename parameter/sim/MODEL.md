# model 投影模型层：实现与仿真

> 配置更新：本文的3视图、40点、27项状态等具体尺寸描述默认构建；可配置范围与当前接口以 [CONFIGURATION.md](../CONFIGURATION.md) 和 calib_defs.vh 为准。

四个模块已实现，外部端口和打包顺序保持原约定。RTL 使用位模式浮点运算器，不使用 `real`，不依赖厂商运算核；测试平台可以使用 `real` 比较数值。

## 模块分工与实现

| 文件 | 负责的计算 | 控制流程 |
| --- | --- | --- |
| [brown_distort.v](../rtl/model/brown_distort.v) | 归一化坐标的径向、切向畸变，支持 FP32/FP64 | 锁存 x/y 和五系数 → 32 条串行浮点运算 → 返回 xd/yd |
| [rotation.v](../rtl/model/rotation.v) | 旋转向量与矩阵双向转换 | 正向选小角度/Rodrigues 分支；反向选四元数分支、统一符号、转回旋转向量 |
| [project_point.v](../rtl/model/project_point.v) | 一个棋盘平面点预测成像素坐标 | R/t 变换 → 检查 Z → 两次除法 → Brown 子核 → 内参投影 |
| [residual_engine.v](../rtl/model/residual_engine.v) | 一次状态对应的 240 项残差和平方和 | 解码内参 → 每视图解码 R/tz → 读点、投影、求残差、累加 cost → 发送 du/dv |

每个模块各有一个串行复用的 `fp_operator`。子模块有自己的运算器，没有共享全局算术仲裁器。每条算术指令遵循“发请求 → 等结果 → 写工作寄存器 → 下一步”；源文件开头列出 `v[]` 寄存器用途和状态段，`calculate` task 只描述寄存器赋值，不是软件函数调用。

依赖关系：

```text
residual_engine
├── fp_operator
├── rotation
│   └── fp_operator
└── project_point
    ├── fp_operator
    └── brown_distort
        └── fp_operator
```

### Brown 畸变

五系数从低位到高位依次为 **k1、k2、k3、p1、p2**。运算顺序保持 [distortion.h](../../algorithom/closer2fpga/closer2fpga/kernels/distortion.h) 的左结合顺序，例如 `k2*r2*r2` 依次相乘，不改为 `k2*(r2*r2)`；每次乘加分别舍入。FP32 每一步均调用 FP32 运算器。

这是正向畸变核，不做反解迭代，也不包含内参、DDR 访问或插值。第三部分的图像校正模块可以按此接口复用。

### 旋转转换

计算顺序对应 [math3.cpp](../../algorithom/closer2fpga/closer2fpga/common/math3.cpp)：

- 向量转矩阵：`t² < 1e-12` 时使用 `a=1-t²/6`、`b=0.5-t²/24`；否则使用 sin/cos/sqrt/div。构造反对称矩阵 S，再计算 `R=I+a*S+b*S*S`。
- 矩阵转向量：按 trace 和最大对角元素选择四个四元数分支。统一为 qw≥0，然后用 `2*atan2(norm(qxyz),qw)/norm(qxyz)` 求比例；范数 ≤1e-12 时比例为 2，避免除零。
- 未选中的输入载荷忽略；成功时两个输出都有效，输入方向的原始位模式透传。
- 输入矩阵应是合法旋转矩阵。与参考函数一致，这里不额外检查正交性或行列式，也不自动修正输入。

### 单点投影

输入对象点是 `(X,Y,0)`，R 按行优先打包，所有字段均从低位开始。计算三行 `R*[X,Y,0]+t`，要求 **Z 严格大于 1e-5**。随后透视除法、Brown 畸变，输出 `u=fx*xd+cx`、`v=fy*yd+cy`。这里传入的 fx/fy/tz 已解码，没有 log 或相对尺度。

### 残差服务

计算对应 [residuals.cpp](../../algorithom/closer2fpga/closer2fpga/algo/calibration/residuals.cpp)。命令锁存 27 项状态和图像尺寸：

1. 检查全部状态有限、宽高 ≥2。解码 fx/fy=exp(logf)，范围为闭区间 [1e-3,1e7]；cx/cy 乘宽高。
2. 每个视图只计算一次 R 和 exp(logtz)，每条新命令重新计算。
3. 视图 0→2，每视图点号 0→39。固定 5 行×8 列，`X=列-3.5`、`Y=行-2`，无需外部对象点 RAM。
4. FP32 观测精确提升为 FP64，计算 `du=预测u-观测x`、`dv=预测v-观测y`。
5. 按 C++ 分组累加 `cost += (du*du + dv*dv)`，然后发送 du、dv。

状态中的五系数是 **k1、k2、p1、p2、k3**，与 Brown 端口不同，模块内部已重新排列。虽然当前标定流程约定 k3=0，本核仍按状态实际值计算 k3。

## 对接时序及错误行为

所有模块一次只接受一个任务。命令只在 `cmd_valid && cmd_ready` 时锁存；计算期间和响应背压期间 `cmd_ready=0`。响应有效后，状态和载荷保持到 `rsp_ready` 接收。复位取消在途计算并清除全部生产端 valid。

角点读口遵循已有 corner_store 的固定一拍时序：

```text
E_n：       corner_store 采样 point_rd_en 和图号/点号
E_n 之后：  corner_store 更新 point_rd_valid、x、y
E_(n+1)：   residual_engine 采样返回；没有返回 ready
```

一次最多一个角点读在途。预定返回拍没有 valid，直接返回 PAR_MEM_ERROR，不无限等待。上层必须保证视图已提交、任务中不清空或改写缓存，且读口路由不增加流水拍。

残差数据索引为 `2*(view*40+point)+component`，component=0/1 对应 du/dv，只有索引 239 的 last=1。数据背压时索引、载荷和 last 保持；du/dv 都被接收后才进入下一点。最后一个 dv 被接收后才发布成功响应。

| 情况 | 状态和结果 |
| --- | --- |
| 正常结束 | PAR_OK；残差模块成功前必须完成全部 240 次数据握手 |
| 宽高小于 2 | PAR_BAD_CONFIG |
| 角点预定返回拍无 valid | PAR_MEM_ERROR |
| 输入非有限、非法运算、除零、算术溢出、深度/焦距不合法 | PAR_CALIB_INVALID |
| 浮点下溢或非精确标志，结果仍有限 | 允许继续 |

前三个模块失败时结果清零。残差模块失败时 cost=+Inf，可能已输出部分数据；接收方收到失败响应后必须废弃整个残差向量，不等待补足 240 项。中间运算出现溢出就立即失败，不允许用后续计算掩盖异常。

## 独立运行测试

在 PowerShell 中分别执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_brown_distort.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_rotation.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_project_point.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File E:\fpga\parameter\sim\run_residual_engine.ps1
```

默认使用 ModelSim SE-64 10.1c，路径可用 `-ModelSimBin` 指定。每个脚本分别编译、运行自己的 TB，并检查完成标志、预期场景数、错误数和 PASS 标记；超时或检查失败返回非零退出码。

测试输入和预期值已随文件保存，日常仿真无需 Node。需要重生成时运行：

```powershell
node E:\fpga\parameter\sim\generate_model_vectors.js
```

参考生成器在软件中按 C++ 公式独立计算，不调用 RTL；FP64 使用 JavaScript Number/Math，FP32 使用 Math.fround 对每次运算舍入。该比较不是直接执行 C++ 二进制；超越函数允许小幅数值差异。

| 测试平台 | 数值/异常场景 | 额外复位恢复场景 | 核对内容 |
| --- | ---: | ---: | --- |
| tb_brown_distort.sv | 260（FP32/64 各130） | 6 | 零畸变、单系数、随机系数、NaN/Inf、溢出、非规格化数；逐位精确比较 |
| tb_rotation.sv | 155 | 3 | 双向转换、零/微小角度、π 附近、三个最大对角分支、随机角度、未选中载荷忽略、非有限与溢出 |
| tb_project_point.sv | 129 | 3 | 不同位姿/畸变、深度边界及负深度、每个输入字段非有限、溢出 |
| tb_residual_engine.sv | 49 | 4 | 三视图不同外参、完整残差/cost、不同宽高、全部状态字段非有限、焦距边界外、早期/中途读失败与观测异常、第三视图 tz 溢出 |

每条数值事务还检查输入锁存、忙时拒绝新命令、响应背压和连续任务。残差测试逐个核对地址及数据索引，并施加随机背压和视图边界/末项的长背压；复位分别覆盖外参计算、点投影和数据背压，再重新完成一次完整任务。

旋转逐元素容差为 `2e-11*(1+abs(expected))`，投影为 `2e-10*(1+abs(expected))`，残差和 cost 为 `2e-9*(1+abs(expected))`。期望失败必须返回指定状态；不能用数值容差绕过异常检查。

最近一次 ModelSim 结果：**593 组数值/异常场景、16 组额外复位恢复场景，全部 0 错误**。残差 TB 检查 348421 次；本组样例完整任务最大 39701 拍（含施加的背压）。这是样例周期统计，不是延迟上界；背压可无限延长任务。

运行记录位于 `sim/build_<模块名>/` 下的 `*_results.txt` 和 `simulation.log`，这些目录已忽略。当前验证包含四个 model 模块的组合调用；角点缓存采用契约一致的同步 RAM 模型，尚未做真实 corner_store 到完整 LM 的系统联调，也未验证综合资源、时序或上板效果。
