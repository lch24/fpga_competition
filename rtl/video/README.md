# 摄像头与显示通路

`board` 管理摄像头/HDMI 配置、I²C 和显示时序；`dma` 搬运采集帧与显示帧；`stream` 处理像素格式和跨时钟流接口。

采集把 RGB565 图写入命令指定的 DDR 槽位，整帧完成后才提交。显示全屏读取选定的原图或校正图区；切换时先排空 DDR 读事务，再清空 HDMI FIFO，从新帧首恢复输出。算法计算与显示分时使用 DDR，计算期间黑屏。操作流程见[板上操作](../../docs/BOARD_OPERATION.md)。

实际连接入口是 `rtl/top/calibrated_view_top.v`，时钟/引脚约束在 PDS 工程同名 FDC。厂商 PLL、DDR IP 保留在工程 `ipcore`，应用 RTL 在这里维护。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
