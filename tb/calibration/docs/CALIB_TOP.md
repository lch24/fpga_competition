# calib_top：完整标定任务调度

> 配置更新：本文的3视图、40点、27项状态等具体尺寸描述默认构建；可配置范围与当前接口以 [CONFIGURATION.md](../../../parameter/CONFIGURATION.md) 和 calib_defs.vh 为准。

实现位于 [calib_top.v](../../../rtl/control/calibration/calib_top.v)。保留原有端口，不访问 DDR；输入是三张图各 40 个有序角点，输出是相机参数、诊断和完成响应。下层算术核全部使用已有 RTL。

2026-09-30，ModelSim SE-64 10.1c 验证：**45组控制场景、2组真实RTL集成场景，全部0错误**。成功场景完整执行五个初值和十五次LM调用，9个FP32相机参数与C++数值完全相同，全部41项FP64诊断在设定容差内。

## 一次任务如何运行

| 状态 | 做什么 | 何时进入下一步 |
| --- | --- | --- |
| IDLE / CLEAR | 接收 collect，保存 job_id，再给角点缓存一拍 clear | clear 结束后开放角点和检测响应 |
| COLLECT | 按图 0、1、2 收集角点及检测状态 | 三图 usable 全为 1；任何错误直接发布失败诊断 |
| WAIT_CMD | 等待相同 job 的标定命令，锁存宽高和格长 | 配置合法后启动 init |
| INIT_CMD / INIT_WAIT | 运行初始化，缓存最多五份 state 和对应 seed_id | 接收 init 完成响应，检查数量和编号 |
| LOAD_SEED / LM_CMD / LM_WAIT | 每个初值依次运行 stage0、stage1、stage2；最终阶段比较并更新 best | 每阶段把返回 state 传给下一阶段 |
| NEXT_SEED | 准备下一个初值，或确认已存在最佳候选 | 所有初值都处理后进入校验 |
| CHECK_CMD / CHECK_WAIT | 对最佳 state 运行 validate_result，锁存报告 | 校验完成后发布输出 |
| OUTPUT | 成功发 camera 和 diag；失败只发 diag | 所需输出分别握手完成 |
| RESPONSE | 发布最终 rsp | rsp 握手后允许下一次 collect |

三个读端口由阶段互斥路由到 corner_store。路由不增加寄存器，保持原有固定一拍读契约。只有子模块的完成响应到达后才切换阶段，因此正常切换时没有尚未消费的角点读返回。

## 最优候选的规则

与原 [calibrate.cpp](../../../algorithom/closer2fpga/closer2fpga/algo/calibrate.cpp) 一致：

1. 初始化成功输出的每个 seed 都参加三个阶段，不提前停止整个搜索。
2. stage0、stage1 未收敛也继续，使用其最后接受的状态。最终收敛标志仅取 stage2。
3. 只按 stage2 的最终 cost 严格下降更新 best；cost 相等保留较早的 seed。
4. 选择 best 后才检查收敛和参数有效性。最佳候选校验失败时返回失败，不换用次优候选。
5. accepted_steps 是最佳 seed 三阶段接受次数之和，使用 16 位，可容纳 450。

LM 返回的 cost 已对应其最后接受的 state，顶层直接使用；validate_result 会重新计算残差诊断。顶层不额外实例化一个残差核。非负有限 FP64 可以直接按位序比较；负零归一为正零，Inf、NaN、负 cost 不成为 best。所有候选都无有效 cost 时返回 CALIB_INVALID。

## 对接时应注意的行为

- `collect` 和 `cmd` 是两次不同握手。提前给出的 cmd 会背压到三张图收集完成；发送方须保持命令载荷。
- collect 后有一拍 clear，这一拍不接收角点或检测响应。job_id 只在 collect 时锁存；cmd 的 job_id 必须相同。
- 每图成功响应应在最后一点传输后发送。同图最后一点和成功响应同时有效时，角点先握手，响应等待下一拍。
- job/view 非法或向已完成图重复输入时，顶层接收非法事务并报 BAD_CONFIG，不把该事务送入缓存。同拍另一条正常输入流也不会被写入。
- 缓存发现顺序、last、点数或非有限坐标错误时，顶层停止收集，返回 BAD_CONFIG。检测失败状态原样传播。角点是否落在图内由初始化层检查。
- 收集失败不等待 cmd；返回一笔 diag 后返回 rsp。系统应停止或排空旧任务数据，再开始新 collect。
- `dbg_view_*` 在任务结束后保留，到下一 collect 的 clear 或复位才清除。
- camera 和 diag 是两个独立输出，可以任意先后接收，也可同拍接收。每个输出只发一次；等待期间载荷稳定。
- 参数包带 job_id 和宽高。外部应暂存 camera，并在成功 rsp 后切换参数。
- 非零子模块状态终止任务，并保留来源阶段；正常未收敛由 converged 表达。`diag_phase`：0 收集/配置，1 初始化，2 LM，3 校验失败，4 成功发布。
- `diag_metrics_valid=0` 时 RMS、每图 RMS、最大误差、姿态和 weak_geometry 清零。无最佳候选时 seed_id=7、accepted_steps=0。已有 best 时，后续错误仍可保留其编号、收敛标记和接受计数；数值诊断仍须看 metrics_valid。
- 计算子核在发布阶段由寄存器驱动的取消信号复位；角点缓存不随任务结束复位。外部 rst_n 取消整个任务。

没有针对上游数据、下游 ready 或运算子模块设置顶层超时；合法接口可以任意背压。系统级看门狗如有需要，应按实际时钟和允许的标定时长另行约定。

## 运行测试

在 `E:\fpga` 执行：

```powershell
# 快速控制分支测试：显式替代子模块响应，不验证数值计算。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/calibration/run_calib_top.ps1 -ControlOnly

# 完整真实 RTL：所有初始化候选、全部三个 LM 阶段、结果校验。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/calibration/run_calib_top.ps1

# 可选：用本机 MSVC 重新生成参考结果，默认使用 E:\vs。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/calibration/run_calib_top_reference.ps1

# 用一次真实图片的C++导出运行完整RTL，并与同目录的C++结果比较。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/calibration/run_calib_top.ps1 -ExportDirectory data\real\run_1790731382958_0
```

ModelSim 默认位于 `E:\pangu\Modelsim10.1c\win64`，可用 `-ModelSimBin` 修改。完整测试保持每阶段 150 轮、每轮 16 次重试的真实上限，未缩短算法，仿真耗时明显长于控制测试。

`-ExportDirectory` 使用Node.js校验导出完成标记、每图点数、算法配置和十进制/IEEE位模式的一致性，然后直接打包原始FP32角点及参考结果。宽高与格长从导出文件读取；C++参数只作输出比较，不参与RTL初始化。此模式目前用于C++已成功标定的导出，执行一组真实任务，允许初始化实际产生1..5份有效候选。它与`-ControlOnly`互斥，原合成向量不被覆盖。

真实任务输出位于 `build_calib_real/`：`real_source.json`保存源路径和SHA256，`real_vectors.txt`保存仿真输入，`actual_camera.hex`和`actual_metrics.hex`保存RTL原始结果，`real_comparison.md/.csv`记录全部50项数值的对比。脚本核对实际有效候选的阶段顺序、最低cost选择和接受次数累计；比较前再次核对源文件哈希。再次运行会更新该仿真目录，原C++导出目录保持原样。准备真实数据时需PATH中的Node.js。

脚本读取公开的 [files.f](../../../parameter/files.f) 编译全部 RTL，独立编译 [tb_calib_top.sv](../../../tb/calibration/tb_calib_top.sv)。默认测试与控制测试分别使用 build_calib_top 和 build_calib_top_control，不共享编译库。完整测试仅保留TB计数器的外部调试访问，并开启ModelSim优化；这不改变RTL或迭代上限。脚本检查退出码、完成标志、场景数量和错误数，并从十五次真实LM响应日志独立核对最终发布的seed和三阶段接受数。

## 测试覆盖与参考来源

控制测试共有 45 组，包含 6 组中途复位场景。使用真实顶层和角点缓存；只在 CONTROL_ONLY 模式将运算子核保持复位，并显式驱动其接口，因此这些测试只能证明调度和接口行为。

覆盖：非法 job/view、角点/视图乱序、重复完成、提前成功、检测失败、配置非法、init 无候选或失败、候选编号重复/倒序/越界/超量、数量不一致、三个 LM 阶段分别失败、跳号 seed、三阶段状态传递、最小值/相等值/非有限 cost、五个候选及 450 次接受计数、先选最小 cost 后判收敛、校验失败、诊断无效、输出三种接收顺序、任务连续执行，以及收集/init/LM/check/输出/rsp 阶段复位。

默认数值测试包含两组：

1. 从 Brown 模型生成三张不同姿态的 FP32 角点，经过真实 corner_store → init → 五个 seed 各三阶段 LM → validate_result，比较相机参数和全部 41 项 FP64 诊断。图像为 640×480，格长为 25，真值 fx=800、fy=820，k1=0.02、k2=-0.005、p1=0.001、p2=-0.002、k3=0。
2. 不复位直接发下一任务，使用三份相同视图，验证真实初始化拒绝重复几何，不启动 LM，也不发布旧参数。

[generate_calib_top_reference.cpp](../../../data/generators/calibration/generate_calib_top_reference.cpp) 直接调用原 C++ 的公开 `calibrate_camera` 获得期望报告；另外调用原初始化和 optimize 函数记录每个 seed、每阶段的 cost、收敛和累计接受数。没有重新编写一套软件优化算法作为参考。

相机字段按 `2e-5 × (1+|参考值|)` 比较；RMS/最大误差按 `2e-7 × (1+|参考值|)`，姿态按 `2e-5 × (1+|参考值|)` 比较，同时要求已知且有限。硬件与 C++ 浮点舍入可能改变几乎相同 cost 的最终排名，因此不要求跨实现 seed_id 或接受次数逐位一致；严格最小值选择和接受次数累计由控制测试另行验证。

日志包含每次 LM 完成时的累计周期、cost、收敛和接受数；完整周期含角点收集、计算及 TB 安排的输出等待，不能作为所有输入的最坏时间。实际运行时间还需除以综合后能够达到的时钟频率。

## 本次实测结果

| 项目 | 结果 |
| --- | --- |
| 完整成功任务 | 371,220,918 周期，755,640 次角点读取 |
| 五个候选 | 每个均完成三个阶段并收敛；seed0接受24次，seed1..4各接受25次 |
| RTL最佳候选 | seed4，cost=1.2122773863732069e-8，累计接受25次 |
| C++最佳候选 | seed3，cost=1.2122773862968384e-8，累计接受25次 |
| 输出参数 | fx≈799.999817、fy≈819.999817、cx≈319.999908、cy=240；全部9个FP32值与C++相同 |
| 下一任务：三份重复视图 | 458,311 周期，返回CALIB_INVALID；不启动LM，不发camera |
| 控制测试 | 45组，0错误，其中6组为中途复位 |

各候选最终cost之间只有极小差别，跨实现的最后几位舍入改变了最佳编号。独立日志检查确认RTL选择的seed4确实是其五个结果中cost最低者，接受次数累计也正确；不存在改选次优或漏跑候选。

若最终能达到100/50/25 MHz，此成功样例分别约需3.71/7.42/14.85秒。这只是周期换算，不表示当前RTL已经达到这些频率，也不是所有图像的耗时上限。

尚未进行目标器件综合、资源评估、静态时序分析或上板测试。完整链路使用合成角点，不能代替真实相机数据的标定质量验证。
