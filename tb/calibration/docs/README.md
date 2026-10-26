# 标定验证导航

测试对象是阶段适配器、共享指令执行器和数值服务。算法设计统一见
[标定架构](../../../docs/CALIBRATION_ENGINE_ARCHITECTURE.md)，
取指和 HOST 时序见[指令指南](../../../docs/INSTRUCTION_CONTROL_GUIDE.md)。
以下命令从仓库根目录执行，仿真产物写入 `build`。

| 范围 | 验证入口 |
|---|---|
| 角点收集与同步读取 | `scripts/memory/run_corner_store.ps1` |
| 初值程序与阶段接口 | `python scripts/calibration/engine/check_init.py` |
| LM 程序与阶段接口 | `python scripts/calibration/engine/check_lm.py` |
| LM/真实残差服务联调 | `python scripts/calibration/engine/check_lm_service.py` |
| 完整 k3 与形状约束 | `python scripts/calibration/engine/check_lm_lens.py` |
| 合成鱼眼数值联调 | `python scripts/calibration/engine/check_lm_fisheye.py` |
| 最终参数与映射检查 | `python scripts/calibration/engine/check_validate.py` |
| 标定阶段控制、错误及背压 | `scripts/calibration/run_calib_top.ps1 -ControlOnly` |
| 完整标定任务 | `scripts/calibration/run_calib_top.ps1` |
| 可配置视图/棋盘/迭代上限 | `scripts/calibration/run_configurable.ps1`，`-TopOnly` 只检查顶层接线 |
| 独立姿态及不同视图数 | `python scripts/calibration/check_pose_views.py` |
| 浮点后端 / 基础算术 | `scripts/compute/run_fp_operator.ps1`、`run_calib_alu.ps1` |
| 旋转 / 投影 / 畸变 / 残差 | `scripts/compute/run_rotation.ps1`、`run_project_point.ps1`、`run_brown_distort.ps1`、`run_residual_engine.ps1` |

初值和检查支持 `--quick` 单例；检查支持 `--policy` 选择验收政策用例。
阶段测试支持 `--compile-only`。控制桩测试验证调度，数值正确性由真实服务与独立参考测试覆盖。

初值/LM/检查输入在 `data/calibration/*_vectors.txt`，实际输出在
`build/calibration_engine/<阶段>`。LM 参考由 `generate_lm_reference.py`、
`generate_lm_fisheye.py` 生成；完整镜头测试同时包含已知真值和归档角点。
真实导出数据可通过 `run_calib_top.ps1 -ExportDirectory <目录>` 输入。

顶层生产流程只有一组初值（seed ID=2）和一个完整模型阶段（stage=2）。
这些是接口编号，不表示还枚举前面的初值或阶段。
完成响应、相机参数和诊断独立握手，测试分别施加背压和复位。
