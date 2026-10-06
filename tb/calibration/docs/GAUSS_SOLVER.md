# gauss_solver 实现与仿真

> 配置更新：本文的3视图、40点、27项状态等具体尺寸描述默认构建；可配置范围与当前接口以 [CONFIGURATION.md](../../../parameter/CONFIGURATION.md) 和 calib_defs.vh 为准。

实现：[gauss_solver.v](../../../rtl/compute/linalg/gauss_solver.v)。测试：[tb_gauss_solver.sv](../../compute/tb_gauss_solver.sv)。

本模块解FP64线性方程A*x=b，维度1..26；覆盖LM三个阶段的22、23、26个活动参数。它不计算法方程或选择lambda，这些由normal_equation/damped_step负责。

## 接口用法

1. cmd_valid&&cmd_ready时锁存cmd_n。
2. 按行输入A的n个元素，然后输入该行b；共n*(n+1)拍。仅最后一行的b令matrix_last=1。允许任意输入空拍，握手才推进计数。
3. 输入完整后内部计算，matrix_ready=0。等待rsp_valid，根据rsp_status判断结果。
4. 成功时第i个解位于rsp_solution_fp64[64*i +:64]，未使用的高槽置0；失败时整个解向量为0。
5. rsp_ready=0时保持响应及结果；响应被接收后才接受下一任务。

cmd_n=0或>26、last位置错误返回BAD_CONFIG。非有限输入返回CALIB_INVALID，但仍先接收本矩阵剩余元素；上游不应因为发送NaN就停止剩余数据。提前last会立即结束本次输入，之后不得继续发送旧矩阵。

模块不对输入暂停设置内部超时；上游必须提供完整矩阵。复位取消加载、计算或待接收的响应，下一任务重新加载全部元素。

## 内部结构和运算顺序

控制流程：加载增广矩阵 → 逐列选主元 → 主元尺度检查 → 必要时换行 → 逐行消元 → 倒序回代 → 完成响应。

- 使用固定27槽行跨度，最大702×64位工作RAM。两路同步读、一条写口，交换两行分两拍写；RAM不整体复位。具体物理存储映射需在目标工具中确认。
- 复用一个fp_operator(FP_W=64)，顺序发送乘、减、除请求并等待结果，不假定算术单元延迟。
- 绝对值比较在确认有限的数值上去掉符号位后进行，无需调用浮点比较器。
- 主元为当前列剩余行的最大绝对值，相等时保留较早的行。
- 候选主元行尺度仅扫描当前列至n-1，不含右端项b。row_scale<1e-30或max_pivot<row_scale*1e-14时失败。
- 换行后及回代时另检查abs(pivot)<1e-30；绝对值小于1e-30的消元元素跳过该行。交换和消元包含右端项b。
- 按参考源码顺序执行factor=value/pivot及entry=entry-factor*pivot_entry；乘法与减法分开舍入。回代从小列号向大列号依次减去已求出的贡献。
- invalid、divide_by_zero、overflow或非有限中间结果导致CALIB_INVALID；underflow/inexact不单独视为失败。

算法顺序参考 [C++ matrix.cpp](../../../algorithom/closer2fpga/closer2fpga/common/matrix.cpp)。对输入和中间结果额外做了显式有限性检查。

## 运行

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\compute\run_gauss_solver.ps1
```

ModelSim SE-64 10.1c验证结果：**45组矩阵＋8组接口/复位场景，0错误**。

| 测试 | 内容 |
| --- | --- |
| n=1..26 | 每个维度均有稠密、对角占优方程；偶数维度反转输入行以触发主元换行 |
| 数值边界 | 奇异零矩阵、线性相关行、相对主元阈值、绝对阈值及等于阈值、微小消元跳过、巨大b、小尺度/大尺度矩阵 |
| 非有限及异常 | NaN、正负Inf、消元溢出、解溢出；正常下溢仍返回成功 |
| 输入协议 | 连续输入与带空拍输入、命令参数锁存、非法n、提前/缺失last |
| 响应协议 | 等待响应期间拒绝新任务与矩阵输入、结果背压稳定、未用解槽清零、任务重复执行 |
| 复位 | 加载途中、浮点事务途中、等待接收响应时复位；复位后重新求解 |

参考向量由generate_gauss_vectors.js生成，按C++相同运算顺序用独立JavaScript数值计算生成期望解。正常矩阵还在生成时检验原始A*x-b的归一化残差不超过1e-12。下溢专用用例不使用该残差门限。TB逐位比较全部26槽，不只检查状态码或大致误差。

gauss_vectors.txt已保存，可直接运行；gauss_cases.txt列出用例名称。若有Node.js，可运行generate_gauss_vectors.js重新生成。

输出位于build_gauss/：gauss_results.txt记录各矩阵状态和计算周期，simulation.log保存运行日志与通过标记。脚本在超时、检查失败或未完成全部用例时返回非零退出码。

浮点除法改为逐拍迭代后，本组26元用例从输入完成到结果就绪约91,288个测试平台等待周期（受当前控制调度定义影响），不是所有矩阵的固定延迟，也不是板上频率测量。上述45组矩阵及8组接口/复位场景已在新算术核心下重新通过。

尚未验证综合资源、RAM映射、布局布线和目标时钟频率；共享浮点运算器当前仍有较宽组合通路。完整LM链路尚未实现，单独求解通过不代表标定链路已验证。本机ModelSim退出时已有的FileWatch Tcl报错不影响本次完成标记和0错误结果，运行脚本同时检查进程退出码和通过标记。
