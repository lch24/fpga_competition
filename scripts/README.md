# 程序生成、仿真与工程脚本

脚本根据自身位置定位仓库。以下示例从根目录运行，输出写入 `build`。ModelSim 通过 `MODELSIM_BIN` 或 PATH 定位，PDS 通过 `PDS_SHELL`、PATH 或脚本参数定位。

## 程序生成入口

| 源文件 | 生成什么 | 对应执行器 |
|---|---|---|
| `image/build_detection_program.py` | 检测任务 ROM、服务编号及指令清单 | `detection_program` |
| `calibration/engine/build_init.py` | DLT/Jacobi/姿态初值程序 | 标定执行器 |
| `calibration/engine/build_lm.py` | 差分、方程、阻尼和参数更新程序 | 标定执行器 |
| `calibration/engine/build_validate.py` | 参数、残差统计、姿态和映射检查程序 | 标定执行器 |
| `calibration/engine/build_rom.py` | 合并上述三个阶段，生成入口地址 | `calib_execution_service` |
| `compute/microcode/build_feature.py` | 张量求解、坐标更新和收敛量程序 | `feature_program` |
| `compute/microcode/build_math.py` | 复杂数学函数程序 | `fp_math_program` |

```powershell
python scripts/calibration/engine/build_rom.py
python scripts/image/build_detection_program.py
python scripts/compute/microcode/build_feature.py
python scripts/compute/microcode/build_math.py
```

程序生成物在 `rtl/include`，随源码提供；修改程序源时再生成。输入汇编与可读清单见[数据目录](../data/README.md)。

## 按修改范围选择验证

| 修改范围 | 运行入口 |
|---|---|
| 检测任务顺序、回退和背压 | `python scripts/image/check_detection_program.py` |
| 检测共享加减 | `python scripts/compute/check_pair_add_pool.py` |
| 检测候选输入边界 | `run_checks.ps1 -OnlyTest tb_detection_capture` |
| 检测整条数值链 | `run_checks.ps1 -OnlyTest tb_detection_fixed` |
| 初值程序/阶段接口 | `python scripts/calibration/engine/check_init.py` |
| LM 程序/阶段接口 | `python scripts/calibration/engine/check_lm.py` |
| LM 与真实残差服务 | `python scripts/calibration/engine/check_lm_service.py` |
| 结果检查 | `python scripts/calibration/engine/check_validate.py` |
| 精修数值程序 | `python scripts/compute/check_feature_program.py` |
| 浮点后端 | `scripts/compute/run_fp_operator.ps1` |
| 投影模型 | `scripts/compute/run_project_point.ps1` |
| 角点存储 | `scripts/memory/run_corner_store.ps1` |
| 板级任务与连接 | `run_checks.ps1 -Board -OnlyTest tb_board_flow` |

表中的 `run_checks.ps1` 位于 `scripts/system`。例如：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
```

初值和检查脚本支持 `--quick`，仅做编译可用 `--compile-only`。完整 LM 单独运行，避免每次局部修改都重复长仿真。数值参考部分使用 Node.js 或 Python，依赖见 `data/generators/requirements.txt`。

## 其余目录

`memory`、`image`、`remap`、`video` 保存各功能的仿真驱动；`system/integration_tests.json` 定义联合测试清单；`common` 负责工具定位。输入数据生成器放在 `data/generators`，不与运行器混放。

`build/check_rtl_layout.py` 检查源码、include、仿真清单和 PDS；`check_test_layout.py` 检查测试路径。`configure_pds.ps1` 同步工程后重新打开核对。`check_partition_synthesis.py` 在隔离目录测量局部资源，带内存保护。`generate_wiring.js` 和 `generate_camera_wiring.js` 维护算法集成连线。
