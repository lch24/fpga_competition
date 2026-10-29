# 提高任务二尝试记录：邻居缓存 II=1 流水线

日期：2026-10-29

> 2026-10-29 后续：方案 A（4×4 控制点网格）已实施，并与 128 行源缓存一起在实板达到 **30.016 fps**；详见文末“方案 A 实施结果”。本文前半部分记录的是更早的 II=1 缓存尝试，该尝试最终未采用。

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

> 2026-10-29 实测修正：方案 A 的地图流量下降本身只把单遍从 36.24 ms 降到 35.90 ms；真正瓶颈是源缓存缺失延迟。方案 A + 128 行源缓存后单遍 28.57 ms，实板 30.016 fps。详见下一节。

## 方案 A 实施结果（4×4 网格 + 128 行源缓存）

### 设计

- [packed_map_builder.v](../rtl/compute/geometry/packed_map_builder.v) 增加 `GRID_MAP/MAP_STEP/GRID_STRIDE`：按 4 个输出像素一个控制点建表，321×181 个点（含右边、下边各一条补齐点），行跨距 384 点；每个控制点存 signed Q16.16 源坐标，地图 **0.556 MB/帧**（旧表 7.37 MB）。
- 新增 [video_grid_prefetch.v](../rtl/image/remap/video_grid_prefetch.v)：3 个轮转行缓冲、512 位描述符窗口、整数双线性插值、由插值坐标派生 mask；每 16 个输出像素取两个字，每 4 个像素推进一个控制列。地图行按 512 B 对齐。
- [realtime_remap.v](../rtl/image/remap/realtime_remap.v) 通过 `GRID_MAP=1/MAP_STEP=4` 接入；[calibrated_view_top.v](../rtl/top/calibrated_view_top.v) 把 `CACHE_INDEX_BITS` 从 6 改为 7（每 bank 128 行，源缓存 16 KB→32 KB）。
- 资源/时序优化版把 valid 位从 128 位向量改为 **1-bit distributed RAM**（`ve/vo`），帧开始用 LINES 个周期清 valid；tag 仍为 23 位 BRAM，保持 tag 的 BRAM 推断。这样去掉了 128:1 valid LUT mux，行为不变，仿真 14/14 通过。另一个“valid 位存进 tag 字 + 清 tag”的方案在综合时 tag BRAM 被推断为寄存器（LUT 64,070 / FF 60,059），已放弃。

### 实板结果

| 版本 | 单遍周期 | 单遍时间 | 校正帧率 | 采集/校正/显示（10 s） | 失败/跳帧 |
| --- | --- | --- | --- | --- | --- |
| 22:31 归档（全分辨率地图，64 行缓存） | 4.53M | 36.24 ms | 15.0 fps | — | — |
| 4×4 网格 + 64 行缓存 | 4.49M | 35.90 ms | 14.99 fps | 152/152/152 | 0 / 152 |
| 4×4 网格 + 128 行缓存 | **3.57M** | **28.57 ms** | **30.016 fps** | **304/304/304** | **0 / 0** |

- 128 行版本 `last_remap_cycles=3,570,636`（28.565 ms），`display_corrected=true`，`health_bits=255`，无 underflow/display error。
- 30 fps 版本已归档：[calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_720p_PG2L100H_20261029_1520.sbit](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_720p_PG2L100H_20261029_1520.sbit)，SHA256 `6D0C4185AB2BC7EAEE1C8ECA49B9C670F8A0D165CE833503D04D2F0515F43922`。

### 关键结论（修正前面的判断）

- **地图流量不是实际瓶颈**。4×4 网格把地图从 7.37 MB 降到 0.556 MB，但单遍只从 36.24 ms 降到 35.90 ms（−0.9%）。真正卡住流水线的是**源缓存缺失后的 DDR 填充延迟**：每次 miss 都要等 4-beat 行填充完成，缓存 miss 数不变时，地图再小也不会变快。
- **128 行源缓存才是关键**。单遍 35.90→28.57 ms（−20.5%），校正视频 15→30.016 fps，采集/校正/显示三者完全对齐、跳帧 0。之前的全帧 TB 用无延迟 DDR 模型，掩盖了 miss 延迟，所以仅凭 TB 的带宽数字得出了错误结论。
- 精度：4×4 双线性控制点相对完整 FP32 地图，真实标定参数最大坐标误差 **0.0044 px**；Kannala-Brandt 鱼眼约 **0.0125 px**；mask 差异 0/20 像素（共 921,600）。画质与完整地图版本一致。

### 资源与时序

| 版本 | CLMA | CLMS | LUT | DRM | 125 MHz WNS | 40 MHz |
| --- | --- | --- | --- | --- | --- | --- |
| 网格 + 64 行 | 11,555/11,675 | 4,921/4,975 | 62,485/66,600 | 114/155 | −0.592 ns | +0.636 ns |
| 网格 + 128 行 | 11,575/11,675 | 4,937/4,975 | 62,638/66,600 | 114/155 | −0.833 ns | +0.280 ns |
| 网格 + 128 行 + valid 1-bit RAM | 11,584/11,675 | 4,937/4,975 | 62,835/66,600 | 114/155 | −0.473 ns | +1.013 ns |

- valid 1-bit RAM 版本把 125 MHz WNS 从 −0.833 ns 改善到 **−0.473 ns**，40 MHz 从 +0.280 ns 改善到 **+1.013 ns**；`g_grid_map` 不再出现在失败端点，最差路径回到原有的 `interpolate`/DDR 路径。最终路由资源仍为 100% CLMA/CLMS（余 91/38），所以后续加法仍需先重新布局。
- 候选 bitstream：[calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_validram_720p_PG2L100H_20261029_1637.sbit](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_validram_720p_PG2L100H_20261029_1637.sbit)，SHA256 `6B338CEDE934A62054D0BDB2540964982155FCB685C11F90CA026915B2EDB3CD`。行为与 1520 已验证版一致，仅每帧多 LINES 个周期清 valid（128 行约 1 µs）；待上板复测 30 fps 后再替换已验证归档。

### 仿射 180° 两遍模式评估

- 当前 30 fps 是**单遍校正**（仿射关闭，`flags=0`）。仿射 180° 走两遍：第一遍地图校正 28.57 ms，第二遍仿射+Gamma 至少受 II=2 缓存下限约束（约 1.84M 周期 = 14.7 ms），合计 ≥43.3 ms，即 **≤23 fps**；按现有 TB/旧实板第二遍 12.8–17 ms 估算，实际约 **22 fps**。
- 30 fps 预算 33.3 ms，第一遍已占 28.57 ms，给第二遍只剩 4.7 ms，低于第二遍的 14.7 ms 下限。因此**两遍仿射不可能达到 30 fps**，除非把仿射融合进地图校正的单遍架构（按仿射坐标求控制点网格）。融合需要非光栅的网格行访问，是架构级改动，且会破坏当前已验证的 30 fps 单遍版本。
- 若开启 Gamma equalize，第二遍还包含自适应伽马直方图；`tb_advanced_remap` 的 Gamma 帧约 5.36 cpp（39.5 ms），总帧约 68 ms ≈14.7 fps。仿射+Gamma 演示建议保持 Gamma 关闭或使用轻量模式；30 fps 不可达。
- 建议：任务 2 的 30 fps 以单遍校正版本交付；任务 3 的仿射 180° 演示保持约 22 fps（Gamma 关闭），不强行追 30。若必须 30 fps 仿射，单独做融合版本/单独 bitstream，不在当前版本上改。

## 最终处理（2026-10-29 更新）

- II=1 缓存改动未采用；方案 A 4×4 网格 + 128 行源缓存已实板验证 **30.016 fps**，当前任务 2 的 30 fps 版本为 [calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_720p_PG2L100H_20261029_1520.sbit](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_720p_PG2L100H_20261029_1520.sbit)，SHA256 `6D0C4185AB2BC7EAEE1C8ECA49B9C670F8A0D165CE833503D04D2F0515F43922`。
- 当前工作区源码为 valid 1-bit RAM 时序优化版；候选 bitstream [validram_20261029_1637](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_grid4_cache128_validram_720p_PG2L100H_20261029_1637.sbit)（SHA256 `6B338CEDE934A62054D0BDB2540964982155FCB685C11F90CA026915B2EDB3CD`，WNS −0.473 ns，网格无失败端点）已构建，待上板确认 30 fps；1520 归档版仍是已实板验证的 30 fps 版本。
- 22:31 归档 [calibrated_view_top_demo_corner_advanced_equalize_720p_PG2L100H_20261028_2231.sbit](../bitstreams/calibrated_view_top_demo_corner_advanced_equalize_720p_PG2L100H_20261028_2231.sbit) 保留作任务 1+3 的旧基线（全分辨率地图，单遍 15 fps）；SHA256 `BF89BBDD60839ABF985FFD0E643E8B90221215CCF9D905C3209B7D3DC8155AD9`。
- 板上当前烧的是 30 fps 网格+128 行版本；重烧/复位后参数丢失，需要重新拍三张、KEY3 标定、KEY4 建表（建表约 0.1 s）。

## 复现方式（2026-10-29 方案 A）

```powershell
# 重新生成工程并跑完整流程
python scripts/build/prepare_demo_project.py
pds_shell -file build/demo/full_flow.tcl
# 回归
powershell -File scripts/system/run_realtime_checks.ps1 -ModelSimBin <ModelSimBin>
```

II=1 尝试补丁仍保存在 [patches/task2_cache_ii1.patch](patches/task2_cache_ii1.patch)，仅作历史复现，不用于当前 30 fps 版本。

测量命令：

```powershell
python scripts/debug/uart_monitor.py COM14 --video-seconds 10
python scripts/debug/advanced_control.py --port COM14 --status
```
