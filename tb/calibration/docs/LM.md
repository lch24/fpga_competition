# LM 阶段与验证

`lm_controller` 负责装入参数和角点、启动共享标定程序，以及执行程序发起的残差 HOST 请求。差分、正规方程、阻尼、消元回代、接受/拒绝更新和收敛判断在 `scripts/calibration/engine/build_lm.py` 与 `data/programs/calibration/solve.asm` 中。

程序按活动参数列执行前向差分（+h 扰动与该状态基准残差相减），每列只调用一次残差服务。共享相机参数和各图 R/t 一起更新：stage 0/1/2 分别激活 22/23/27 列，stage 2 含 k3，与重映射核的三阶径向项一致。试探代价下降时接受状态，否则调整阻尼再求步长。收敛判据除步长和梯度外，还要求本步代价下降不小于当前代价的 1e-6，避免噪底后的多余迭代。整个阶段保有工作 RAM 所有权。

同一条 5×8 归档数据（`data/calibration/uart_coverage_corners.json`，固定 k3）的 RTL 回放从中心差分的 11 步、138,937,550 周期降到前向差分的 9 步、**69,642,644 周期**；代价从 1.385983380551 变为 1.385983384472（相对差 2.8e-9），RMS 仍为 0.10747028。

实板（2026-10-30，新比特流）：真实三张鱼眼照片上 LM 接受 **36 步**、RMS 1.0367、固定 k3、收敛；从回看确认到参数发布共 **7.50 s**（初值 0.65 s + LM/检查 6.85 s），旧版同一段为 21.84 s。真实数据的迭代数远多于干净的归档夹具（旧版同样会跑几十步），这正是相对下降阈值要解决的尾部抛光。

```powershell
python scripts/calibration/engine/check_lm.py
python scripts/calibration/engine/check_lm_service.py
scripts/calibration/run_lm_fisheye.ps1   # 合成鱼眼角点 + 独立数值参考
```

前者验证程序及阶段接口，`check_lm_service.py` 连接真实残差服务，`run_lm_fisheye.ps1` 用独立 Python 模型生成的鱼眼角点验证 k3 能从零初值收敛。TB 分别是 `tb/calibration/tb_lm_controller.sv`、`tb/control/calibration/engine/tb_lm_service.sv` 和 `tb/calibration/tb_lm_fisheye.sv`。输出在 `build/calibration_engine` 下同名子目录。

旧 `jacobian`、`normal_equation`、`damped_step`、`gauss_solver` RTL 已删除；修改这些步骤应改程序，而不是重新增加并行控制器。

`data/calibration/lm_controller_vectors.txt` 由 `scripts/calibration/engine/generate_lm_reference.py` 生成：物理 Brown 真值加确定性初值扰动，期望值来自同目录的独立 Python 参考解。该参考解先在旧 C++ fixture 上对过（stage 0/1 状态差 1.6e-14、接受步数相同），再把 k3 计入 stage 2。
