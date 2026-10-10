# 集成入口

`calibrated_view_top.v` 是 PDS 板级顶层，连接摄像头、DDR、40 MHz 算法域和 HDMI。`vision_ddr_top.v` 是算法入口，完成 DDR 原图到 DDR 校正图。`vision_camera_top.v` 保留为相机接口组合及独立集成测试入口，不是第二个板级综合顶层。

板级顶层直接维护。后两个顶层的连线源分别为 `scripts/build/generate_wiring.js`、`generate_camera_wiring.js`。参数、DDR 分区和模块端口改变时同步生成器，文件清单由 `scripts/build/check_rtl_layout.py` 检查。

整体流程见[系统架构](../../docs/SYSTEM_ARCHITECTURE.md)，其他模块入口见[RTL 导航](../README.md)。
