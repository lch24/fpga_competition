# 实时视频的数据通路与帧调度

## 流程与操作阶段

默认 `REALTIME_VIDEO=1`，摄像头采集、重映射和显示通过 DDR 分离。
板内处理 960×540 RGB565，显示出口放大到 1280×720。

```text
video_capture_hmic → DDR 三个原图槽
                             ↓
地图预取 → 源像素缓存 → 双线性插值 → 结果写回 → DDR 两个结果槽
                                                      ↓
                       HDMI ← 跨域像素桥 ← 放大器 ← 显示 DMA
```

`video_board_flow` 复用同一套通路完成预览和校正视频。
预览请求 `remap_bypass=1`，普通模式为恒等复制，不依赖标定参数；
人工模式先应用人工映射。按拍照、回看、标定、校正或模式键后进入 STOP，
停止安排新任务，等已接受的事务结束，再进入相应操作。

照片槽独立于视频槽。拍照和回看成功后，从显示读取第一帧开始计时两秒，随后返回预览。
KEY4 的校正视频先按当前参数建表；参数未改变时再次启动可以复用表。
视频阶段 KEY0 只改变后续帧是否校正，在途帧继续完成。

## 模块与接口

所有 ready/valid 命令只有在 `valid && ready` 时交接；未接受时必须保持 VALID 和整个载荷。所有 native HMIC 地址都是 **32 位字地址 = 字节地址 / 4**，不是 AXI 字节地址。

| 模块 | 职责与关键接口 |
|---|---|
| `control/system/video_board_flow.v` | UI、照片 lease、地图有效位、三个 raw / 两个 dst 的所有权；校正命令 `remap_src/remap_dst`，完成响应 `remap_status` |
| `video/dma/video_capture_hmic.v` | core→pclk 描述符队列；pclk→core START/DATA/END 队列；`rsp_base` 关联已完成的原图槽；下一帧不必等前帧响应才预排队 |
| `common/async_bram_fifo.v` | 捕获数据和 HDMI 跨域共用同步读双时钟 BRAM FIFO；输出被接收时同拍预取下一条，可连续每拍传输，背压时保持数据；判满结果寄存，避免比较器直接驱动 RAM 写使能 |
| `compute/geometry/packed_map_builder.v` | 复用 `map_coord_core` 与 FP32 服务离线建表；一次坐标只发一次；两笔 32 位写完成后才进入下一像素 |
| `image/remap/realtime_remap.v` | 每帧锁存源、结果、地图地址，启动下列四级；最后物理写完成后返回响应及 `last_cycles` |
| `image/remap/video_map_prefetch.v` | 双 64 像素块地图缓冲，16 个 native beat / 突发；四个 64 位同步 RAM 银行同时接收 256 位返回；描述符流 II=2 |
| `image/remap/video_neighbor_cache.v` | 奇/偶源图行分银行，默认每银行 64 条 cache line，每条 64 像素；数据和 tag 同步 BRAM、只有 valid 位清零；每帧 invalidation，防基址循环后的旧数据 |
| `image/kernels/video_bilinear_pipe.v` | 六级弹性流水、II=1，输入寄存器隔开缓存选择和插值；差分乘法形式减少互补权重乘法；保持 RGB565 扩展和一次舍入结果 |
| `video/dma/video_frame_writer.v` | 双 256 像素块 BRAM；每块全部就绪后才提 16-beat AW；完成含最终 `WUSERO_LAST`，不把最后 WREADY 当作帧完成 |
| `memory/ddr/video_hmic_arbiter.v` | READ: 算法、HDMI、地图、源缓存；WRITE: 摄像头、算法、结果写回。各方向只允许一个 outstanding，选择先锁定，再提出物理 VALID |
| `video/dma/board_display_hmic.v` | 输出 `frame_start`，整帧首次读前锁存基址；寄存请求地址并按突发递增；输出原尺寸 RGB565 与帧首尾 |
| `video/stream/video_scale_4_3.v` | 两行 BRAM、五级移位加法双线性放大，960×540 → 1280×720，再送 HDMI 跨域 FIFO |
| `video/stream/hdmi_pixel_bridge.v` | 2048×18 位 BRAM 队列跨到像素时钟域；每拍一个像素，帧首与 VSYNC 对齐，欠载后丢弃旧帧尾并等待下一帧 |

HMIC **没有 RREADY 和 WVALID**。客户必须存下已经接受的整个读突发；写入前必须备好整段数据。仲裁器在 RLAST / WUSERO_LAST 前不能换客户；显示读有优先权，其余循环公平。读和写可以同时各有一笔事务，不能用旧全局 owner 门控反馈。

### 帧所有权

- raw：FREE → 预留并排队采集 → 完整捕获 READY → REMAP 持有 → FREE。READY 槽以三深度小队列按完成顺序处理，不能按槽号优先导致显示时间倒退或某槽饥饿。失败采集或停止时未使用的 READY 可释放；排队中、采集中和校正中的槽不能提前释放。
- dst：FREE → REMAP/物理写 → READY → DISPLAY → FREE。只有成功完成响应才生成 READY。显示在下一整帧起始锁存 READY 的基址，再释放旧 DISPLAY。
- 显示队列里已复制出的像素不再依赖旧 DDR 槽，因此可以在**读取器**的整帧边界释放，而不必等显示器物理 VSYNC；HDMI bridge 的 SOF token 仍对齐显示帧。若发生 underflow，丢弃余帧并等下个 SOF/显示帧，不移位接着显示。
- 没有可用原图槽或相机队列未准备好时，跳过整帧并计数，不覆盖有主人的槽。输出没新 READY 时重复旧完整帧，不能算新显示帧。
- 溢出、尺寸不符、read ID/RLAST 不符、写错误只让当前帧失败，失败结果不显示。相机字队列预留 START/END marker 空间，数据溢出仍能报告 END，不靠复位脱困。

相机不再产生 VSYNC、或者 DDR 完全不返回 RLAST/WUSERO_LAST 时，安全 drain 本来就无法完成；此时用日志定位并联合复位整板/DDR，不能仅复位仲裁器后复用在途地址。当前没有强行取消事务的超时开关。

## DDR 和映射格式

默认地址分配集中说明在[根 README](../README.md#当前-ddr-分区)，
人工演示附加区见[演示设计](DISTORT_DEMO.md#硬件组织与数据流)。

一条地图对应一个**输出像素**，按行优先、8 字节、little endian。数据不是输出坐标到输出坐标，也不是源图被按坐标“推送”：输出 `(u,v)` 顺序扫描，表告诉我们应去源图 `(x0,y0)` 附近取哪些像素。

| 位 | 内容 |
|---|---|
| `[15:0]` | dx，Q0.16 小数 |
| `[31:16]` | dy，Q0.16 小数 |
| `[45:32]` | x0，14 位有符号 floor 坐标 |
| `[59:46]` | y0，14 位有符号 floor 坐标 |
| `[63:60]` | 邻域有效 mask，`{p11,p01,p10,p00}` |

邻域为 p00=(x0,y0)、p10=(x0+1,y0)、p01=(x0,y0+1)、p11=(x0+1,y0+1)。**负坐标必须符号扩展**，不是把 −1 当作 16383。越界邻域补黑，不 clamp；mask=0 时坐标无需有效、不发源图读。Q16 地图复用既有 floor/小数转换，只有存储格式变化。

视频处理尺寸当前为 960×540，显示尺寸为 1280×720。复用 runtime 模块时需 WIDTH 为 64 的倍数、总像素为 256 的倍数，源基址 128 B、结果/地图 512 B 对齐；cache fill 按 128 B 边界对齐（native 地址清低 **5** 位）。本板配置满足这些条件。

## 流水与缓存

源缓存按奇偶行分银行，每银行 64 条、每条 64 像素，有效像素容量 16 KiB。
地图和写回缓冲各 1 KiB；显示放大只缓存两行，共 3,840 字节。
这些数据保存在同步 BRAM，tag/数据读取与命中判断分拍进行。

源缓存先寄存地址，再访问 RAM；插值器输入寄存器隔开缓存选择与乘加。
显示请求地址先寄存，按突发递增。摄像头 FIFO token 锁存和解析分拍完成。
这些寄存边界属于数据通路协议，整理代码时需要保留数据、valid 和帧标记的对齐关系。

每秒 30 个新帧对应每帧约 4,166,667 个 125 MHz 周期，960×540 下平均约
8.04 周期/像素。源缓存缺失率、DDR 仲裁和两遍人工演示处理决定实际吞吐。
每帧基础 DDR 流量包括地图 4.15 MB、采集写/结果写/显示读各 1.04 MB，以及源缓存填充；
显示放大不会增加 DDR 像素读取次数。

## 验证与观察

`tb_realtime_remap` 覆盖查表、缓存、插值、写回及错误恢复；
`tb_board_video` 检查显示完整帧；`tb_photo_preview` 检查预览和照片操作交接。
运行器是 `scripts/system/run_realtime_checks.ps1` 和 `scripts/demo/run_checks.py`。

串口 `--once video` 提供采集、校正、显示计数和耗时，`--video-seconds 10` 统计新帧率。
显示重复旧帧与 HDMI 保持同步都不计作新的校正帧。
采集/尺寸诊断见[板上操作](BOARD_OPERATION.md#串口调试)。
