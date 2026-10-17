# 实时视频链路：设计与交接

## 当前结论

默认板级入口 `calibrated_view_top` 的 `REALTIME_VIDEO=1` 已接入连续摄像头采集、一次建表、源像素块缓存、定点插值、结果帧写回和 HDMI 完整帧显示。目标为 **1280×720 / RGB565 / 30 个新校正帧每秒**。2026-10-12 本版资源已放下、布局布线完成；可以交给下一位继续验证，但布局时序未收敛，真实板上 30 fps 和校正质量尚未验证。

本次不重写检测/标定数值算法，不重新采集已占用的标定照片，不支持在线更换参数或边建表边显示。`REALTIME_VIDEO=0` 保留旧照片校正模式，`vision_ddr_top.PACKED_MAP=0` 保留原独立算法接口。编译期 generate 只实例化选中的通路，不在默认整板中复制旧采样器。

## 操作与阶段

1. 摄像头就绪后自动实时预览。KEY0、KEY1、KEY2 拍三张标定照片；成功后显示 2 秒回到预览，已有照片也只回看 2 秒，不覆盖。
2. KEY3 检测、标定，九个 FP32 参数保存在 `vision_sequence`，结束后返回拍照预览。
3. KEY4 停显、排空旧读事务，发建表命令。`PACKED_MAP=1` 使 `vision_sequence.BUILD_ONLY=1`：建表成功直接返回，**不再调用旧单照片 remap**。
4. 建表完成自动进入 VIDEO，无需再按键，默认显示校正后。KEY0 切换校正前／校正后；KEY1～KEY3 短按在视频阶段被忽略。退出后 KEY0 恢复第一张照片功能，再次进入视频默认显示校正后。
5. 视频中 KEY4 进入 STOP：不再新建采集/校正命令，已提出的 VALID 必须保持到接受；等待已排队相机帧、正在校正的帧和显示 DDR 事务完成，再恢复拍照预览。正常活动摄像头下最多还需排空三份采集描述符。
6. 再按 KEY4 复用地图。KEY3 重新标定使旧地图失效。全局复位清照片与参数有效位。

标定和建表在 40 MHz 域；视频校正、DDR 仲裁和帧调度在 125 MHz 域。HDMI 时序仍为现有 37.125 MHz、720p30。摄像头真实 PCLK/帧率仍需测量，FDC 的相机 100 MHz 是预算而非实测值。

校正前／后选择在 `video_board_flow` 分配新处理任务时锁存为 `remap_bypass`，按键不会改变已提交或等待接收的描述符。普通模式的校正前显示由 `realtime_remap` 产生整数坐标、零插值权重的恒等采样流，不读取映射表，复用源缓存、插值器和输出写入器，逐像素原样复制到输出双缓冲。人工畸变模式则由 `distort_demo_flow` 直接将加畸变结果写入输出槽，跳过第二遍校正；不会显示未加畸变的相机原始帧。两种方式都保留原有缓存占用和完整帧发布规则，不新增整帧缓存。按键后可能先显示已在途的旧模式帧，再显示新模式帧；这不是同一帧冻结对比。校正帧计数只累计实际完成校正的帧，显示帧计数包含两种画面。

## 2026-10-12 首次布局结果（**历史记录**）

> 本节的资源与时序数字属于**首次布局**的现场记录，仅作对照；当前提交版（`cbf9b00`）的实测与边缘路径清单见 [TIMING_CLOSURE_REPORT.md](TIMING_CLOSURE_REPORT.md)。

用户使用当前 PDS 工程完成综合（21:45）、布局布线/优化（22:00）、时序报告（22:02）。层级报告已确认包含 `algorithm/g_packed.correction`、`g_realtime.remap`、`g_video_capture.capture` 和 `native_memory`，不是旧照片版结果。

| 资源 | 最新结果 | 余量/说明 |
|---|---|---|
| 综合 LUT | 60,310 / 66,600，90.56% | 不等于布局后可用空间 |
| 布局后物理 LUT | 66,024 / 66,600，99.14% | 仅剩 576 |
| CLMA + CLMS 逻辑簇 | 11,662 + 4,965 = 16,627 / 16,650，99.86% | 仅剩 23 簇 |
| 布局后 FF | 52,804 / 133,200，39.64% | FF 不是瓶颈 |
| RAM 物理位置 | 105 / 155，67.74% | 综合容量统计为 102 个 DRM36K 等效块 |
| APM | 70 / 240，29.17% | 乘法器不是瓶颈 |

实时校正主体综合为 **1,300 LUT、17 DRM、10 APM**：源缓存 843 LUT/9 DRM/1 APM，插值 150 LUT/9 APM，地图预取 166 LUT/4 DRM，结果写入 104 LUT/4 DRM。捕获 BRAM FIFO 已推断为 4 DRM，原生仲裁为 334 LUT。大缓存没有落成数千 LUT 的寄存器阵列。

时序（**已被 `cbf9b00` 取代**）：当时 125 MHz DDR/视频域 WNS −0.949 ns、113 个 setup 失败端点，40 MHz 算法域 +0.308 ns。当前提交版设计侧只剩 **9 个 setup 失败端点、整体最差 −0.339 ns**，三条边缘路径的域/起点/终点/机制见 [TIMING_CLOSURE_REPORT.md](TIMING_CLOSURE_REPORT.md) 第 5 节。**布线成功不等于时序通过**，不得把旧 bitstream 上的照片验证当作本版视频验证。

本地证据：`prj_tasks/syn_1/synthesize/calibrated_view_top.snr`、`prj_tasks/pnr_1/place_route/calibrated_view_top.ror`、`prj_tasks/pnr_1/report_timing/calibrated_view_top.rtr`（均在 `OV5640_DualView_100H` 内）。生成报告/工作数据库不提交 Git；本节记录可供队员交接的摘要。

### 2026-10-15 复核（提交版 RTL，仅综合）

在本机对**已提交**的 RTL 重跑 `compile + synthesize`：综合 LUT 60,008 / 66,600（90.10%），LUT-as-dram 2,543，DRM 102，APM 70，寄存器 52,422。比 2026-10-12 那次**少 302 LUT**，说明最后一批 RTL 改动是净减少。**限制资源仍是布局后的物理 LUT 与 CLMA/CLMS（接近 100%），不是综合余量**；任何新增逻辑都必须重新做完布局再判断。

顺带否掉了原计划的"大 mux / LUTRAM 转 BRAM"：
- `display(board_display_hmic)` 实测只有 **337 LUT、58 FF、128 个 LUT-as-dram**。它那个 16×256 位行缓冲已经被综合器推断成分布式 RAM，并没有落成 4096 个寄存器加大 mux；改成 BRAM 只会多一个读时拍和 DRM，没有收益。
- `bridge(hdmi_pixel_bridge)` 是 784 LUT（其中 576 为 LUT-as-dram），而且**它就是全部 2,543 个 LUT-as-dram 中占比最大又最容易动的一块**。但它的 FIFO 是**异步读（II=1，每个 pclk 出一个像素）**；换成 `async_bram_fifo` 的同步读（II=2）会把显示取像素速率直接减半，720p 每行 1280 个 DE 周期只能取 640 个像素。要省这 784 LUT 就得改成"两像素打包成一字 + 配对对齐"，会有像素配对错位的静默风险，收益约 1.2%，**不值得做**。结论：视频通路上目前没有安全可省的大块资源，后续要加逻辑只能从算法域（`vision_ddr_top` 占 48,800 LUT）里腾，或者先做完布局再决定。

**2026-10-16 更新**：这版 RTL 已完成完整布局，设计侧 setup 失败端点 **66 → 9**、整体最差 setup **−1.633 → −0.339 ns**（改动、四版对照与三条边缘路径见 [TIMING_CLOSURE_REPORT.md](TIMING_CLOSURE_REPORT.md)）。

## 与旧架构的差别

| 项目 | 旧照片版 | 默认实时版 |
|---|---|---|
| 输入 | 一次命令拍一张，再等待按钮 | 相机域提前排队下帧地址，连续 VSYNC 接帧 |
| 地图 | 每次 KEY4 建两个 FP32 平面，再校正 | 参数不变只建一次 64 位定点描述符表 |
| 每帧计算 | 串行地图读取、转换、取邻域、小写事务 | 原生突发预取、两行银行块缓存、定点流水插值 |
| DDR 所有权 | 拍摄/算法/显示全局分时 | 四读客户、三写客户，读写独立仲裁 |
| 输出 | 一份静态结果区 | 两份结果槽，完整帧提交后才换基址 |
| 输入保护 | 标定照片长期冻结 | 三个视频原图槽，CAPTURE 与 REMAP 不得同时复用 |
| 帧率判断 | 重复显示同一张也可稳定 HDMI | 统计采集、校正、显示的**新帧计数** |

## 模块与接口

拍照预览复用 `video_board_flow` 的 VIDEO 状态与三输入槽、双输出槽，内部 `preview` 位区分操作语义，UART 阶段号为 12。预览任务固定请求 `remap_bypass=1`：普通模式恒等复制，人工模式仅执行人工映射。无需有效标定参数，也不建校正表。收到照片键、标定键、校正键或模式切换时进入 STOP，完成已接受事务后转入相应操作。照片槽不参与预览循环。回看使用原有显示读取器和一个定时计数器，没有新增图像缓存、浮点单元或重映射器。

本轮先缩减 `residual_engine`：外参按视图顺序移位，代替六路按视图选择 FP64 参数的多路选择器；浮点写回只译码实际可写的临时寄存器，不再对整个状态数组动态写入。保持 FP64 精度、运算顺序和迭代次数。49 组数值与 4 组协议/复位用例通过。修改前用户综合结果为 60,617 LUT；修改后尚未综合，实际净收益及布局余量待测。

所有 ready/valid 命令只有在 `valid && ready` 时交接；未接受时必须保持 VALID 和整个载荷。所有 native HMIC 地址都是 **32 位字地址 = 字节地址 / 4**，不是 AXI 字节地址。

| 模块 | 职责与关键接口 |
|---|---|
| `control/system/video_board_flow.v` | UI、照片 lease、地图有效位、三个 raw / 两个 dst 的所有权；校正命令 `remap_src/remap_dst`，完成响应 `remap_status` |
| `video/dma/video_capture_hmic.v` | core→pclk 描述符队列；pclk→core START/DATA/END 队列；`rsp_base` 关联已完成的原图槽；下一帧不必等前帧响应才预排队 |
| `common/async_bram_fifo.v` | 捕获数据使用同步读双时钟 BRAM，输出保持寄存器；有意读 II=2，每 16 摄像头像素才产生一条 DATA，因此足够 |
| `compute/geometry/packed_map_builder.v` | 复用 `map_coord_core` 与 FP32 服务离线建表；一次坐标只发一次；两笔 32 位写完成后才进入下一像素 |
| `image/remap/realtime_remap.v` | 每帧锁存源、结果、地图地址，启动下列四级；最后物理写完成后返回响应及 `last_cycles` |
| `image/remap/video_map_prefetch.v` | 双 64 像素块地图缓冲，16 个 native beat / 突发；四个 64 位同步 RAM 银行同时接收 256 位返回；描述符流 II=2 |
| `image/remap/video_neighbor_cache.v` | 奇/偶源图行分银行，默认每银行 64 条 cache line，每条 64 像素；数据和 tag 同步 BRAM、只有 valid 位清零；每帧 invalidation，防基址循环后的旧数据 |
| `image/kernels/video_bilinear_pipe.v` | 五级弹性流水、II=1；差分乘法形式减少互补权重乘法；精确保持旧 RGB565 扩展和一次舍入结果 |
| `video/dma/video_frame_writer.v` | 双 256 像素块 BRAM；每块全部就绪后才提 16-beat AW；完成含最终 `WUSERO_LAST`，不把最后 WREADY 当作帧完成 |
| `memory/ddr/video_hmic_arbiter.v` | READ: 算法、HDMI、地图、源缓存；WRITE: 摄像头、算法、结果写回。各方向只允许一个 outstanding，选择先锁定，再提出物理 VALID |
| `video/dma/board_display_hmic.v` | 输出 `frame_start`，只在整帧第一次读前锁存基址；RGB565 按帧首/尾标记送 HDMI FIFO |

HMIC **没有 RREADY 和 WVALID**。客户必须存下已经接受的整个读突发；写入前必须备好整段数据。仲裁器在 RLAST / WUSERO_LAST 前不能换客户；显示读有优先权，其余循环公平。读和写可以同时各有一笔事务，不能用旧全局 owner 门控反馈。

### 帧所有权

- raw：FREE → 预留并排队采集 → 完整捕获 READY → REMAP 持有 → FREE。READY 槽以三深度小队列按完成顺序处理，不能按槽号优先导致显示时间倒退或某槽饥饿。失败采集或停止时未使用的 READY 可释放；排队中、采集中和校正中的槽不能提前释放。
- dst：FREE → REMAP/物理写 → READY → DISPLAY → FREE。只有成功完成响应才生成 READY。显示在下一整帧起始锁存 READY 的基址，再释放旧 DISPLAY。
- 显示队列里已复制出的像素不再依赖旧 DDR 槽，因此可以在**读取器**的整帧边界释放，而不必等显示器物理 VSYNC；HDMI bridge 的 SOF token 仍对齐显示帧。若发生 underflow，丢弃余帧并等下个 SOF/显示帧，不移位接着显示。
- 没有可用原图槽或相机队列未准备好时，跳过整帧并计数，不覆盖有主人的槽。输出没新 READY 时重复旧完整帧，不能算新显示帧。
- 溢出、尺寸不符、read ID/RLAST 不符、写错误只让当前帧失败，失败结果不显示。相机字队列预留 START/END marker 空间，数据溢出仍能报告 END，不靠复位脱困。

相机不再产生 VSYNC、或者 DDR 完全不返回 RLAST/WUSERO_LAST 时，安全 drain 本来就无法完成；此时用日志定位并联合复位整板/DDR，不能仅复位仲裁器后复用在途地址。当前没有强行取消事务的超时开关。

## DDR 布局与打包格式

| 内容 | 字节起始地址 | 大小 |
|---|---|---|
| 标定照片 0/1/2 | `00000000` / `00200000` / `00400000` | 每份 1,843,200 B |
| 视频原图 0/1/2 | `00800000` / `00A00000` / `00C00000` | 每份 1,843,200 B |
| 视频结果 0/1 | `01000000` / `01200000` | 每份 1,843,200 B |
| packed map | `02000000`，终止不含 `02708000` | 7,372,800 B |
| 灰度金字塔 | `03000000` | 1,152,000 B |
| 候选区 | `03200000` | 262,144 B |

一条地图对应一个**输出像素**，按行优先、8 字节、little endian。数据不是输出坐标到输出坐标，也不是源图被按坐标“推送”：输出 `(u,v)` 顺序扫描，表告诉我们应去源图 `(x0,y0)` 附近取哪些像素。

| 位 | 内容 |
|---|---|
| `[15:0]` | dx，Q0.16 小数 |
| `[31:16]` | dy，Q0.16 小数 |
| `[45:32]` | x0，14 位有符号 floor 坐标 |
| `[59:46]` | y0，14 位有符号 floor 坐标 |
| `[63:60]` | 邻域有效 mask，`{p11,p01,p10,p00}` |

邻域为 p00=(x0,y0)、p10=(x0+1,y0)、p01=(x0,y0+1)、p11=(x0+1,y0+1)。**负坐标必须符号扩展**，不是把 −1 当作 16383。越界邻域补黑，不 clamp；mask=0 时坐标无需有效、不发源图读。Q16 地图复用既有 floor/小数转换，只有存储格式变化。

视频尺寸当前固定 1280×720。复用 runtime 模块时需 WIDTH 为 64 的倍数、总像素为 256 的倍数，源基址 128 B、结果/地图 512 B 对齐；cache fill 按 128 B 边界对齐（native 地址清低 **5** 位）。本板配置满足这些条件。

## 资源与性能边界

**旧布局后的逻辑簇已约 99%、物理 LUT 约 97.7%，所以不能保证新代码一综合就能放下。** 停止运行标定不会释放硬件资源。本版的节约是实在改变电路：默认不实例化旧逐像素 remap，新增大缓冲使用 BRAM，像素打包使用固定移位而非可变位置写，运行时不再做 FP32 坐标转换，插值用互补权重的差分等价式。离线建表仍保留一套 FP32 服务，不能宣称整板已消除浮点。

插值逻辑只有六个 9×17 和三个 25×17 乘积；宽乘积可能拆成多个 APM，APM/LUT 数由 PDS 决定，不把“9 个乘法表达式”算成“9 个 APM”。源缓存有效载荷为 16 KiB、地图缓冲 1 KiB、结果块缓冲 1 KiB、捕获 token FIFO 9 KiB；实际 DRM 使用有宽度/深度碎片化，不能只按有效字节除以 36 Kibit。

资源检查顺序：

1. **重新 Compile，再 Synthesize**，避免用旧编译产物。顶层仍为 `calibrated_view_top`，`REALTIME_VIDEO` 必须为 1。
2. 查层级资源中是否存在 `algorithm/g_packed` 和 `g_realtime/remap`；旧 `algorithm/g_static/correction` 不应被实例化。检查 cache、map、writer、capture FIFO 是否推断 DRM，而不是数千 LUT 的寄存器阵列。
3. 看总 LUT、CLMA/CLMS、DRM、APM；不要只看综合 LUT 余量。若很接近上限，继续布局看实际簇占用。优先检查 BRAM 推断与变量位选，不先减少 Q16 精度或取消安全帧所有权。
4. 若仍超额，进一步共享离线建表的浮点后端或优化原标定/检测大头；不是再并联新采样器。缓存只在 BRAM 紧张时考虑减少 `CACHE_INDEX_BITS`，较小缓存可能显著降低帧率，需重新测量。
5. 查 125 MHz 新数据通路与旧 DDR 路径时序，40 MHz 标定路径、FIFO CDC、恢复时间和 PHY 约束。FDC 移除了 camera/HDMI↔DDR 的全局 false-path，改为 8 ns datapath bound，保留异步 hold cut；Gray 指针和跳帧 Gray 计数必须逐位满足该界。新约束可能暴露原被隐藏的路径，这是待签核项，不能用全局 false-path 掩盖。

30 fps 要每帧不超过约 **4,166,667 个周期 / 33.33 ms**，平均 ≤4.52 周期/像素，还需摄像头源帧率与显示接收速率足够。cache 普通 hit II=2，跨 16 像素 beat 边界要额外查一次，miss 会阻塞。

`tb_realtime_fullframe` 用**真实 cache（64 线/组，`CACHE_INDEX_BITS=6`）**、按 1280×720 整帧比例取 1280×192 条带、warp 由等距鱼眼模型生成，逐像素比对邻居和插值结果：

| 场景 | 周期/像素 | 结论 |
|---|---|---|
| `f_raw=f_out=560`，端侧压缩 warp（输出行 0～191） | **3.146** | 通过，全部像素正确 |
| `f_raw=560,f_out=300`，中心 1.9× 拉伸 warp（输出行 264～455） | **3.014** | 通过，全部像素正确 |

折算整帧约 2.8M 周期（22 ms），**余量约 1.5×**。也就是说**鱼眼量级的畸变参数不构成 30 fps 瓶颈**：真实反畸变地图是单调平滑 warp，早先"随机散点 114k 周期/5120 像素"是刻意构造的最坏情形，不代表可达的畸变。注意该测试的 DDR 仍是 tb 内的仲裁模型（读写各一 outstanding、显示优先、循环公平），不是真实 HMIC。

每帧基础流量：地图 7.37 MB + 源缓存填充 + 捕获写 1.84 MB + 校正写 1.84 MB + 显示读 1.84 MB；源读按 miss 率变化，不能只按“四个像素”算 DDR 带宽。

## 编译、针对性检查与下一位验证

PDS 工程还是 `OV5640_DualView_100H/DualView_OV5640.pds`，源码清单已同步。此前用户工程备份为同目录 `DualView_OV5640.backup_realtime_20261012.pds`；没有更换芯片、引脚或算法时钟。不要用旧 bitstream 验证新功能。

```powershell
# 仅配置/编译，不自动综合或烧板
python scripts/build/check_rtl_layout.py
powershell -ExecutionPolicy Bypass -File scripts/build/configure_pds.ps1 -PdsBin <pds_shell.exe>
# 建议从 build/system 工作目录执行，PDS 的自动 shell 工程落在忽略目录
& <pds_shell.exe> -file <仓库绝对路径>/scripts/build/compile_realtime.tcl

# 定向检查；自动选用 PATH 上的 iverilog，否则用 vsim；也可显式指定工具目录
powershell -ExecutionPolicy Bypass -File scripts/system/run_realtime_checks.ps1 [-IcarusBin <目录> | -ModelSimBin <目录>]
python scripts/debug/uart_monitor.py --self-test
```

定向检查包含 packed-map 单发/格式/错误写回、连续三帧采集、短帧/溢出排空与恢复、frame lease/停止重启、随机整数/小数/负边界/beat 与 cacheline 跨界、跨帧缓存 invalidation、原生仲裁停顿及错误帧排空、1280×192 条带的鱼眼 warp 整帧比例吞吐，同时回归旧显示切换、旧手动算法命令和 UART 页选择。没有重跑全部标定数值回归，也不模拟厂商 DDR PHY、电气或物理 CDC；吞吐用的是 tb 内的 DDR 仲裁模型，不是真实 HMIC。

资源能过后交给下一位：

1. 重新生成并下载 bitstream，确认相机、DDR、HDMI 初始化及 UART 心跳。
2. 重拍三张完整棋盘（5×8 **内角点**），标定得到有效参数；第一次 KEY4 等建表结束，观察实际运动物体，而不是只看静止棋盘。
3. 进入稳定视频后，查两类页面：

```powershell
python scripts/debug/uart_monitor.py COM11 --once status
python scripts/debug/uart_monitor.py COM11 --once video
python scripts/debug/uart_monitor.py COM11 --video-seconds 10
```

`video` 页 op=0x56、页号 FFFD，核心 32 B 为八个 little-endian u32：state / UI events / captured / corrected / displayed-new / failed / skipped-since-reset / last-remap-cycles。只解析页匹配且新鲜的快照。计数按 2^32 回绕；视频计数从功能复位起累计，skipped 包含标定等待时期，用差值评估视频期间。`displayed_new_frames` 只在 DISPLAY 换到新成功结果时加一，重复旧帧不加。UI 事件不再计数每次 display idle，避免淹没查询。

4. 对比 10 秒计数差：captured、corrected、displayed-new 均接近 300（允许统计边界 ±1～2 帧），failed/skipped 差为 0，`last_remap_ms <33.33`；状态页没有新增 display error/underflow。显示器刷新率不算证明。
5. KEY4 停止应恢复原图；再启动无需建表，测试多次停止/启动与不同运动场景；重标定应重新建表且不显示旧地图结果。
6. 与软件用同一九参数、黑色边界和 RGB565 格式比较；最后验证真正畸变校正质量、长期帧率和资源/时序。若 NEW 帧率低，按 captured→corrected→displayed 的差值区分采集缺帧、cache/DDR 校正瓶颈、显示 underflow。

最终签核条件仍是新整板资源、布局时序、持续的新帧计数和校正质量，而非“代码编译过”或“屏幕有图”。
