# 保留的历史验证报告

清理旧工作目录时，只保留以下精简证据，不保留旧 RTL 副本、模拟器库和一次性修改脚本。

- [真实角点对比](real_calibration/real_comparison.md)：原完整 LM 回归的 50 项对比；同目录保存实际输出位模式、逐项 CSV、任务统计和输入哈希。
- [几何与重试缓存资源对照](synthesis/geometry_cache_summary.json)：六个局部分区的改造前后结果，原始 `.snr` 报告位于同目录。
- [上一轮结构优化汇总](synthesis/structure_summary.json)：此前分区资源统计。

这些是历史结果，不代表最新代码重新通过了完整 LM 或整板综合。输入数据在 `data/real`，新运行结果仍写入根 `build`。
