# 检测到标定的数据交接

检测入口为 [corner_detect_ddr_top.v](../../../rtl/image/features/corner_detect_ddr_top.v)。输入 DDR 中的 Gray8 图，输出原图坐标下、行优先排列的 FP32 角点，以及本张图的检测状态。RGB565 到 Gray8 在外层完成。

板级检测采用两遍响应扫描：第一遍计算 Harris 全图最大响应，第二遍重算并做阈值/NMS，不缓存整幅响应图。灰度金字塔和候选工作区在 DDR，窗口及随机访问局部缓存留在片上。

`detect_ctrl` 接入检测任务程序，调度层切换、候选合并/精修/圆环检查、90 个方向搜索和网格精修。具体服务与指令接口见[检测设计](../../../docs/DETECTION_ENGINE.md)。

| 接口 | 含义 |
|---|---|
| `process_frame` | 空闲时单拍启动，帧配置在任务期间稳定 |
| `busy/done/status` | 任务进度与结果，done 保持到下次启动 |
| `cfg_gray_base/stride/w/h` | Gray8 字节基址、行跨度和尺寸 |
| `out_valid/out_ready/x/y` | FP32 原图像素坐标流，背压时保持 |
| `out_total/out_grid_ok` | 网格点数和是否找到有效棋盘 |
| `m_*` | 逻辑 DDR 请求/返回，由系统适配到物理总线 |

[vision_sequence](../../../rtl/control/system/vision_sequence.v) 将点流编上 view/index，向角点存储提交每图状态。点号顺序为 `row*COLS+col`，默认每图 40 点。检测完成和点流是两个交接条件；图失败时保留状态，不补造角点。

运行 `scripts/system/run_checks.ps1 -OnlyTest tb_detection_fixed` 检查真实检测链；`tb_detection_capture` 检查空输入、点数不足、正常和溢出排空；程序级回退与背压由 `scripts/image/check_detection_program.py` 检查。
