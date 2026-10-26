# 集成入口

`calibrated_view_top.v` 是 PDS 板级顶层，连接摄像头、DDR、40 MHz 算法域和 HDMI。`vision_ddr_top.v` 是算法入口，完成 DDR 原图到 DDR 校正图。`vision_camera_top.v` 保留为相机接口组合及独立集成测试入口，不是第二个板级综合顶层。

板级顶层直接维护。后两个顶层的连线源分别为 `scripts/build/generate_wiring.js`、`generate_camera_wiring.js`。参数、DDR 分区和模块端口改变时同步生成器，文件清单由 `scripts/build/check_rtl_layout.py` 检查。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。

当前板级采用按键流程，`MANUAL_CONTROL=1`：三键拍照/选图，一键标定，一键建表并启动/停止校正视频。`key_n[4:0]` 和 `led[7:0]` 的引脚已写入 FDC，原来的独立初始化 LED 端口改为内部信号。拍照状态、DDR 分区和接口交接见[板上操作](../../docs/BOARD_OPERATION.md)。


串口使用 TX C24 / RX B24、115200 8N1 和独立的 27 MHz 时钟。板级开启 `vision_ddr_top.DEBUG_ENABLE`，调试复位独立于工作模块复位。接口及电脑脚本见[板上操作](../../docs/BOARD_OPERATION.md#串口调试)。
