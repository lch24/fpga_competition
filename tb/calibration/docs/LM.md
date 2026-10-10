# LM 阶段与验证

`lm_controller` 负责装入参数和角点、启动共享标定程序，以及执行程序发起的残差 HOST 请求。差分、正规方程、阻尼、消元回代、接受/拒绝更新和收敛判断在 `scripts/calibration/engine/build_lm.py` 与 `data/programs/calibration/solve.asm` 中。

程序按活动参数列执行中心差分，分别调用正负扰动的残差服务。共享相机参数和各图 R/t 一起更新，当前固定 k3。试探代价下降时接受状态，否则调整阻尼再求步长。整个阶段保有工作 RAM 所有权。

```powershell
python scripts/calibration/engine/check_lm.py
python scripts/calibration/engine/check_lm_service.py
```

前者验证程序及阶段接口，后者连接真实残差服务。TB 分别是 `tb/calibration/tb_lm_controller.sv` 和 `tb/control/calibration/engine/tb_lm_service.sv`。输出分别在 `build/calibration_engine/lm` 和 `lm_service`。

旧 `jacobian`、`normal_equation`、`damped_step`、`gauss_solver` RTL 已删除；修改这些步骤应改程序，而不是重新增加并行控制器。
