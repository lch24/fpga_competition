# 图像处理数据通路

`kernels` 保存窗口、梯度、Harris、张量和插值等局部运算；`features` 保存候选合并、圆环、排序、网格与亚像素服务；`remap` 保存映射坐标生成与图像校正。

检测入口为 `features/corner_detect_ddr_top.v`，内部 `detect_ctrl` 接入 `control/detection` 的任务程序。两遍 Harris 响应扫描避免整图响应缓存；任务程序循环调用数值服务，精修算术进入共享运算池。

校正从有效相机参数生成 X/Y 映射表，然后顺序遍历输出像素，读取源图邻域、插值并写回 DDR。它保留专用数据通路，不由标定程序逐像素解释执行。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
