# 调度与指令控制

`system`、`frame` 管理采集、帧交接、标定和校正任务；`detection` 用任务指令组织金字塔、候选和方向循环；`calibration` 管理初值、LM、检查与结果发布。

板级操作入口是 `system/board_buttons.v`（同步与消抖）和 `system/video_board_flow.v`（预览、照片、标定、视频和帧所有权）。`system/vision_sequence.v` 在算法域分别执行标定、校正命令，参数在两次命令之间保留。具体状态交接见[板上操作](../../docs/BOARD_OPERATION.md)。

标定 `init/lm/check` 是阶段适配器，数值步骤由共享 `engine/calib_sequencer.v` 执行。检测的具体数值服务在 `image/features`，标定共享 ROM 和工作区连接在 `compute/service`。

阅读顺序：系统任务 → `calib_top` / `detection_flow_control` → 阶段适配器 → 指令核心。程序维护入口在根 `scripts/calibration/engine` 和 `scripts/image/build_detection_program.py`。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
