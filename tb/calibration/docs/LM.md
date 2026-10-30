# LM 阶段与验证

`lm_controller` 负责装入参数和角点、启动共享标定程序，以及执行程序发起的残差 HOST 请求。差分、正规方程、阻尼、消元回代、接受/拒绝更新和收敛判断在 `scripts/calibration/engine/build_lm.py` 与 `data/programs/calibration/solve.asm` 中。

程序按活动参数列执行前向差分（+h 扰动与该状态基准残差相减），每列只调用一次残差服务。共享相机参数和各图 R/t 一起更新：stage 0/1/2 分别激活 22/23/27 列，stage 2 含 k3，与重映射核的三阶径向项一致。试探代价下降时接受状态，否则调整阻尼再求步长。收敛判据除步长和梯度外，还要求本步代价下降不小于当前代价的 1e-6，避免噪底后的多余迭代。整个阶段保有工作 RAM 所有权。

同一条 5×8 归档数据（`data/calibration/uart_coverage_corners.json`，固定 k3）的 RTL 回放从中心差分的 11 步、138,937,550 周期降到前向差分的 9 步、**69,642,644 周期**；代价从 1.385983380551 变为 1.385983384472（相对差 2.8e-9），RMS 仍为 0.10747028。

实板（2026-10-30，新比特流）：真实三张鱼眼照片上 LM 接受 **36 步**、RMS 1.0367、固定 k3、收敛；从回看确认到参数发布共 **7.50 s**（初值 0.65 s + LM/检查 6.85 s），旧版同一段为 21.84 s。真实数据的迭代数远多于干净的归档夹具（旧版同样会跑几十步），这正是相对下降阈值要解决的尾部抛光。

**条件式分阶段预热**（2026-10-30 第二轮）：程序把请求列数 ≥ 状态宽度−1（即含畸变列）且初始针孔拟合代价 ≥ 1080（约 240 残差上 RMS 3 px）的 stage 2/3 任务拆成 22 列、23 列两个预热阶段再跑请求列数；否则只跑一个阶段。真实鱼眼数据（初始 RMS≈24 px）单阶段 41 步 → 26 步，按列数加权工作量约 −41%，最终 RMS 不变（1.975150454 对 1.975150476）；近针孔归档数据（初始 RMS≈1.2 px）仍走单阶段，不受影响。合成 Brown 夹具 stage 2 初始代价极小，仍为单阶段 6 步；鱼眼夹具畸变很强，走三阶段（33 次接受），数值仍收敛到 1.28e-8。

实板同一版（2026-10-30）：预热路径接受 34 步、RMS 1.495、收敛；phase 2（初值+LM/检查）**5.69 s**，上一版单阶段 6.85 s。实板没有达到归档夹具的 41%，因为这次照片的最后全模型阶段仍用了约 19 步；门限以下的照片会自动走单阶段，不会变慢。

**按视图裁剪残差**（2026-10-30 第三轮）：姿态列只影响自己那一视图的重投影。程序给 +h 扰动的残差服务带上窗口 `svc_first/svc_last`（内参列仍是 0..239 全范围），`lm_controller` 把窗口写进 `cmd_first/cmd_last`，`residual_engine` 跳过窗口之前的视图、在窗口所属视图结束时立即响应；差分、列缩放以及正规方程/梯度的内积循环也只遍历该列的残差区间。试探、基准与接受后的代价仍用完整残差，所以收敛判据和报告的 cost/RMS 不变。同一归档夹具（固定 k3、9 次接受、代价 1.385983384472 不变）的 RTL 回放从 69,642,644 周期降到 **46,927,790 周期（−32.6%）**；指令级模型（3 视图 40 点）从 1,290,565 条降到 1,028,360 条（−20%）。

实板同一场景（2026-10-30）：接受 **38 步**、RMS **0.7233**、收敛、fixed_k3；初值+LM/检查 **4.28 s**（上一版 5.69 s，−24.8%），其中 LM/检查约 3.63 s（上一版约 5.04 s）；KEY3→参数发布 11.7 s，KEY4 建表 0.5 s；视频 29.90–30.00 fps / 25.60 ms，失败 0、跳帧 0。

```powershell
python scripts/calibration/engine/check_lm.py
python scripts/calibration/engine/check_lm_service.py
scripts/calibration/run_lm_fisheye.ps1   # 合成鱼眼角点 + 独立数值参考
```

前者验证程序及阶段接口，`check_lm_service.py` 连接真实残差服务，`run_lm_fisheye.ps1` 用独立 Python 模型生成的鱼眼角点验证 k3 能从零初值收敛。TB 分别是 `tb/calibration/tb_lm_controller.sv`、`tb/control/calibration/engine/tb_lm_service.sv` 和 `tb/calibration/tb_lm_fisheye.sv`。输出在 `build/calibration_engine` 下同名子目录。

旧 `jacobian`、`normal_equation`、`damped_step`、`gauss_solver` RTL 已删除；修改这些步骤应改程序，而不是重新增加并行控制器。

`data/calibration/lm_controller_vectors.txt` 由 `scripts/calibration/engine/generate_lm_reference.py` 生成：物理 Brown 真值加确定性初值扰动，期望值来自同目录的独立 Python 参考解。该参考解先在旧 C++ fixture 上对过（stage 0/1 状态差 1.6e-14、接受步数相同），再把 k3 计入 stage 2。
