# 程序生成、仿真与工程脚本

`demo/run_checks.py --modelsim <目录> --board` 验证可移除的人工畸变扩展及两种整板配置；`demo/run_images.cmd` 用真实照片和渲染棋盘运行像素级软件闭环。入口与结果见[人工畸变演示](../docs/DISTORT_DEMO.md)。

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
| 实测角点的固定 k3 重试 | `python scripts/calibration/engine/check_lm_replay.py`（先生成下述复算结果） |
| 结果检查 | `python scripts/calibration/engine/check_validate.py` |
| 标定任务控制（默认几何） | `scripts/calibration/run_calib_top.ps1` |
| 可配置几何（视图/棋盘/迭代上限） | `scripts/calibration/run_configurable.ps1`（`-TopOnly` 只跑顶层接线） |
| 精修数值程序 | `python scripts/compute/check_feature_program.py` |
| 浮点后端 | `scripts/compute/run_fp_operator.ps1` |
| 投影模型 | `scripts/compute/run_project_point.ps1` |
| 角点存储 | `scripts/memory/run_corner_store.ps1` |
| 板级任务与连接 | `run_checks.ps1 -Board -OnlyTest tb_board_flow` |
| 串口与主机协议 | `run_checks.ps1 -Board -OnlyTest tb_board_debug` |
| 调试快照与历史错误 | `run_checks.ps1 -OnlyTest tb_debug_snapshot`、`tb_core_debug` |

表中的 `run_checks.ps1` 位于 `scripts/system`。例如：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
```

初值和检查脚本支持 `--quick`，仅做编译可用 `--compile-only`。完整 LM 单独运行，避免每次局部修改都重复长仿真。数值参考部分使用 Node.js 或 Python，依赖见 `data/generators/requirements.txt`。

## 其余目录

`debug/uart_monitor.py` 是电脑端串口入口：`python scripts/debug/uart_monitor.py COM5` 持续接收状态，`--once progress` 查询执行进度，`--corners data/debug/corners.json` 导出角点。依赖为 `pyserial`；`--self-test` 仅需 Python 标准库。协议和操作见[板上操作](../docs/BOARD_OPERATION.md#串口调试)。

实测角点的诊断链按以下顺序运行，结果统一写入 `build/debug`：

```powershell
python scripts/debug/replay_calibration.py data/calibration/uart_fold_corners.json
powershell -File scripts/debug/replay_cpp.ps1 -VsRoot <VisualStudio目录>
python scripts/calibration/engine/check_lm_replay.py
```

第一步独立复算完整模型和固定 `k3=0` 模型，并检查全图映射；第二步调用生产 C++ 算法交叉比较；第三步用同一批角点验证 RTL LM 的固定 k3 路径。第二步不是第三步的依赖。RTL 的自动重试顺序由 `run_calib_top.ps1 -ControlOnly` 单独验证。新采集数据放 `data/debug`，有长期回归价值的数据再归档到 `data/calibration`。

`memory`、`image`、`remap`、`video` 保存各功能的仿真驱动；`system/integration_tests.json` 定义联合测试清单；`common` 负责工具定位。输入数据生成器放在 `data/generators`，不与运行器混放。

`build/check_rtl_layout.py` 检查源码、include、仿真清单和 PDS；`check_test_layout.py` 检查测试路径。`configure_pds.ps1` 同步工程后重新打开核对。`check_partition_synthesis.py` 在隔离目录测量局部资源，带内存保护。`generate_wiring.js` 和 `generate_camera_wiring.js` 维护算法集成连线。
