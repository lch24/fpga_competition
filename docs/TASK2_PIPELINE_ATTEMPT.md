# 提高任务二尝试记录：邻居缓存 II=1 流水线

日期：2026-10-29

## 目标

选题指南高阶任务 2 要求"针对复杂的坐标映射和浮点/定点运算，实现图像的实时矫正"。当时的基线是：直通预览约 30 fps，但经过鱼眼校正的视频只有约 15 fps（[CORNER_REVIEW.md](CORNER_REVIEW.md) 已记录 15.03 fps）。目标是把校正视频拉到 30 fps（每帧 ≤33.3 ms，125 MHz 下 ≤4.17M 周期）。

## 尝试的改动

改 [video_neighbor_cache.v](../rtl/image/remap/video_neighbor_cache.v)：命中路径原来是固定的 `IDLE→CHECK` 两拍（II=2），每个像素都要空一拍。改为：

- 当前像素命中、没有跨 16 像素 beat 边界、且下游 `out_ready` 有效时，同一拍接收下一个描述符并发出下一次 BRAM 查找；
- 下一拍照常比对标签并输出；
- 跨 beat 的像素仍需第二次查找（II=2，占每行 1/16）；
- 缺失时停流并整行填充，不读旧数据；每帧开始清空 tag/valid。

补丁保存在 [patches/task2_cache_ii1.patch](patches/task2_cache_ii1.patch)，可单独重放。

## 仿真结果

- `scripts/system/run_realtime_checks.ps1` 11 项全部通过，包含逐像素的 `tb_realtime_remap`、`tb_realtime_fullframe`。
- `tb_advanced_remap`（仿射 + Gamma）通过。
- 全命中帧：5120 像素 / 5962 周期，约 1.16 周期/像素（II=1 生效）。
- 真实鱼眼 warp + 仿真 DDR 模型：2.805 周期/像素，低于 30 fps 预算 4.52。

## 构建结果

| 项 | 结果 |
| --- | --- |
| CLMA | 10,910 / 11,675（余 765） |
| CLMS | 4,671 / 4,975（余 304） |
| LUT | 61,900 / 66,600 |
| 算法域 40 MHz | +0.330 ns，0 失败端点 |
| DDR 域 125 MHz | −0.922 ns，79 失败端点 |

- 比特流：2026-10-28 23:46:59，SHA256 `E730FE998185DC55FF20F9CB0D7474B551BEFBEFB18D5D0518750525788ECA55`。
- 该比特流只保留在本地 `build/task2_attempt/calibrated_view_top_ii1_20261029.sbit`（`build/` 不入库），未替换归档版本。

## 实板测量

摄像头：3 秒计数 +96，确认 30 fps 源。

校正视频，10 秒统计：

| 配置 | 采集 / 校正 / 显示 | 跳帧 | 失败 | 单帧周期 |
| --- | --- | --- | --- | --- |
| 仿射关闭 | 15.0 / 15.0 / 15.0 fps | 152 / 10 s | 0 | 4.53M（36.24 ms） |
| 仿射 180° | 15.0 / 15.0 / 15.0 fps | 153 / 10 s | 0 | 第一遍 4.55M（36.4 ms）+ 第二遍 1.59M（12.8 ms）= 6.15M（49.18 ms） |

`health_bits=255`，`sticky_display_error=false`，`sticky_underflow=false`。跳帧数约等于采集数，说明摄像头 30 fps，但流水线每两帧只能接一帧。

## 结论

II=1 把缓存命中吞吐翻倍，仿真成立，但没有跨过 30 fps 门槛：

- 单遍校正仍需 36.2 ms，已经超过 33.3 ms；
- 仿射第二遍只增加 12.8 ms，不是主因；
- 单遍流量约 11 MB/帧，其中地图表 7.37 MB，实测约 300 MB/s，已接近 HMIC 的有效带宽。

因此瓶颈是 DDR 地图流量，不是缓存命中节拍。要真正达到 720p30，需要降低地图带宽。

## 后续可选路线（本次未实施）

| 方案 | 做法 | 预期 | 风险 |
| --- | --- | --- | --- |
| A. 半分辨率地图 | 地图 640×360，remap 读 2×2 网格插值坐标 | 地图 7.37→1.84 MB，单遍约 20 ms | 改地图生成、prefetch、描述符生成；精度损失最小 |
| B. 32 位描述符 | x/y 打包 16 位定点，mask 由边界推导 | 地图减半，单遍约 24 ms | 跨 builder/cache/仿射，bilinear 分数精度降到 1/16 |
| C. 运行时算畸变 | 9 参数在视频域定点计算，彻底不读地图 | 带宽最低 | 最大改动，需与 FP64 建表精度对齐 |

## 最终处理

- 本项目不采用 II=1 改动，工作区已恢复为任务 1 + 任务 3 版本。
- 已验证的归档比特流仍为 [calibrated_view_top_demo_corner_advanced_equalize_720p_PG2L100H_20261028_2231.sbit](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_720p_PG2L100H_20261028_2231.sbit)，SHA256 `BF89BBDD60839ABF985FFD0E643E8B90221215CCF9D905C3209B7D3DC8155AD9`。
- 板上当前烧的是 II=1 尝试版，需要重新烧录上述 22:31 归档版；重烧后参数丢失，需要重新拍三张、两次 KEY3 标定。

## 复现方式

```powershell
# 应用尝试补丁
git apply docs/patches/task2_cache_ii1.patch
# 重新生成工程并跑完整流程
python scripts/build/prepare_demo_project.py
pds_shell -file build/demo/full_flow.tcl
# 回归
powershell -File scripts/system/run_realtime_checks.ps1 -ModelSimBin <ModelSimBin>
```

测量命令：

```powershell
python scripts/debug/uart_monitor.py COM14 --video-seconds 10
python scripts/debug/advanced_control.py --port COM14 --status
```
