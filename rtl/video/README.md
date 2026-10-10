# 摄像头与显示通路

`board` 管理摄像头/HDMI 配置、I²C 和显示时序；`dma` 搬运采集帧与显示帧；`stream` 处理像素格式和跨时钟流接口。

采集把 RGB565 图写入 DDR，任务控制等待完整帧后交给算法；显示从已发布的图像区域持续读取。算法计算与显示读取互相通过 DDR 仲裁及跨域 FIFO 解耦。

实际连接入口是 `rtl/top/calibrated_view_top.v`，时钟/引脚约束在 PDS 工程同名 FDC。厂商 PLL、DDR IP 保留在工程 `ipcore`，应用 RTL 在这里维护。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
