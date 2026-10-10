# 当前标定验证导航

测试对象是“阶段适配器＋共享指令执行服务”。初值、LM 和检查使用同一个板级执行架构，验证其数值、RAM 交接、背压及复位；旧的独立矩阵控制器测试已经移除。

| 内容 | 实现说明 | 验证入口（仓库根目录） |
|---|---|---|
| 角点收集与读取 | `rtl/memory/parameters/corner_store.v` | `scripts/memory/run_corner_store.ps1` |
| 初值 | [INIT](INIT.md) | `python scripts/calibration/engine/check_init.py` |
| LM | [LM](LM.md) | `python scripts/calibration/engine/check_lm.py` |
| LM/残差联调 | [LM](LM.md) | `python scripts/calibration/engine/check_lm_service.py` |
| 最终检查 | [VALIDATE_RESULT](VALIDATE_RESULT.md) | `python scripts/calibration/engine/check_validate.py` |
| 标定任务控制 | [CALIB_TOP](CALIB_TOP.md) | `scripts/calibration/run_calib_top.ps1` |
| 浮点后端 | [FP_OPERATOR](FP_OPERATOR.md) | `scripts/compute/run_fp_operator.ps1` |
| 投影和残差 | [MODEL](MODEL.md) | `scripts/compute/run_project_point.ps1`、`run_residual_engine.ps1` |

角点存储按 view 和点号保存，成功提交一张图后才作为标定输入。读取是同步 RAM 接口，检测状态与点流分别检查；失败图不通过补点伪装成成功。

旧完整算法的实拍对照归档在 [data/reports](../../../data/reports/README.md)，不作为当前代码刚刚完成端到端验证的记录。整体架构见[系统设计](../../../docs/SYSTEM_ARCHITECTURE.md)。
