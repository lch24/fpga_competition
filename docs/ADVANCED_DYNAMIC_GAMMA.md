# 选题二提高部分 1、3：独立开发版

基于 `main@41bc59d` 独立开发，提交分支为 `advance`（开发时分支名为 `advanced/dynamic-gamma`）。本版没有叠加随后 main 的红点预览功能；两套扩展的联合集成需另行验证。本版只完成代码集成与定向检查，**还没有经过 PDS 综合、布局布线或板上验证**，不能标记为比赛指标全部验收。

## 与当前上板工程隔离

当前上板继续使用 `D:\camera_project\fpga_competition` 原目录，不需要替换文件或重新 program。

本次开发位于 `D:\camera_project\fpga_competition_advanced`。未访问串口、JTAG，也未运行 PDS。原工程已有的 PDS 修改和 `output/` 未动。

权威顶层 `rtl/top/calibrated_view_top.v` 新参数 `ENABLE_ADVANCED=0`，默认综合裁剪提高逻辑，原 `ENABLE_DISTORT_DEMO=1` 默认值不变。提高版要求 `REALTIME_VIDEO=1, ENABLE_DISTORT_DEMO=0, ENABLE_ADVANCED=1`，不支持与人工畸变演示模式同时启用。

后续准备提高版工程，在**开发目录**执行：

```powershell
python scripts/build/prepare_advanced_project.py
```

独立工程：`OV5640_Advanced_100H/Advanced_DynamicGamma.pds`。

该命令只离线生成配置，不启动 EDA。生成顶层 `build/advanced/project_top.v` 从权威顶层自动派生，只改两个参数默认值，模块名和内部实例路径不变，避免破坏原 FDC 的 PLL、DDR PHY 层级约束。工程只引用本 worktree 的源码/IP，包含路径为本目录的 `rtl/include`；ROM 镜像及编译结果位于独立工程目录。修改顶层后应重新生成；不要手工维护派生顶层。生成器会把旧任务状态置为 INVALID，不把历史实现结果当作新版本结果。

生成工程/FDC/顶层不提交 Git；提交源 RTL、脚本和本文后，下一位可重新生成。

## 第 1 项：上位机控制的动态旋转／仿射

选用指南允许的“上位机指令”途径，暂不接 IMU。FPGA 继续自行标定镜头参数；上位机只发送**二维逆仿射系数**，支持任意角度、平移、缩放及直接输入六系数的仿射变换。不是重新上传九个镜头参数，也不是完整三维相机姿态/深度补偿或自动防抖。

每个输出像素 `(x,y)` 的采样位置：

```text
u = a00*x + a01*y + a02
v = a10*x + a11*y + a12
```

六个系数为 signed Q16.16，打包顺序 `[a00,a01,a02,a10,a11,a12]`，a00 在低 32 位。上位机计算三角函数；FPGA 逐行/逐像素递推相加，48 位累加器避免极端合法系数回绕。坐标越界按四邻域掩码补黑，支持左/上边缘的负小数坐标。

畸变映射和仿射一般不可交换，因此采用：

```text
原始帧 ── 原镜头去畸变 ── DDR scratch ── 逆仿射 ── Gamma ── 输出帧
            第 1 遍                         第 2 遍
           └──────── 同一个 remapper / cache / 双线性插值 / writer ────────┘
```

静态镜头 map 仍只在标定后构建；改变角度不重建整张 map。第一遍中间帧使用字节地址 `0x01800000..0x019FFFFF` 的 2 MiB 槽位，720p RGB565 占 1,843,200 字节，不与原照片、实时输入、输出、演示或映射表槽位重叠。调度器一帧完全完成才接下一帧，原输入/输出租约保持到第二遍结束；停止视频也先完成已接受的任务。第一遍失败不启动第二遍。

关闭仿射时，去畸变、Gamma、输出仍为一遍。**相机预览和实时视频的 before/after 都会接收上位机仿射与 Gamma**：预览不做去畸变，直接在原始采集帧上做仿射采样并叠加 Gamma；校正视频仍按“去畸变→逆仿射→Gamma”两遍执行，before 是不去畸变但带仿射/Gamma 的直通采样，便于只对比镜头矫正。红点查看的复制帧是诊断用的原始坐标副本，**故意保持不动**，否则标记点会与图像错位；2 秒拍照回看是 DDR 直显，同样不经过重映射。提高版默认上电配置为全部关闭；使用原标定/启动视频步骤，再通过 UART 启用。

## 第 3 项：自适应 Gamma

在最后一遍的双线性插值后、DDR 写回前，使用 RGB565 分量 LUT。保持 RGB565 接口，不增加浮点、实时幂运算、一般乘除法或一整帧片上缓存。

公式 `out = 255 * (in/255)^gamma`，然后量化回 5/6/5 位。

| 配置 mode | 上位机名称 | 曲线 |
| --- | --- | --- |
| 0 | auto | 自动从以下五条中选择 |
| 1 | identity | gamma=1，像素位精确直通 |
| 2 | brighten | gamma=0.75 |
| 3 | brighten-strong | gamma=0.5 |
| 4 | darken | gamma=1.3 |
| 5 | darken-strong | gamma=2 |
| 6 | equalize | 每帧 64 bin 亮度直方图，生成对比度受限的均衡曲线 |

自动统计增强前的近似亮度 `(R8+2*G8+B8)/4`。完全无效的映射像素不参加统计，避免越界黑边被当成夜景。均值 `<32 / <64 / 64..160 / >160 / >208` 分别推荐强提亮、提亮、恒等、压暗、强压暗。同一推荐连续出现三个有效帧才切换；每帧曲线锁存，不会半帧改变。统计只在启用增强的最终遍进行，不将第一遍重复统计。刚启用时默认恒等，允许数帧适应。

这是全局亮度自适应，不是局部 HDR，也不能恢复已经过曝截断或欠曝丢失的细节。自动效果仍需强光/暗光实际场景验证。

### mode 6：直方图均衡

`equalize` 是独立的第 6 条曲线，不再依赖固定 Gamma 档位：

- 每个启用的最终遍对**有效像素每 4 个采样一次**，累计 64 bin 亮度直方图；黑边不参与。
- 帧末把直方图累加成 CDF，逐 bin 用 32 周期的串行除法计算 `cdf*255/总数`，再与恒等映射按 **3:1 混合**，限制极端对比度拉伸。
- 生成的 8 位曲线拆成 R/B 的 32 级和 G 的 64 级查找表；逐像素仍然只查表，不增加乘除、浮点或整帧缓存。
- 若一帧的亮度分布跨度小于 8 个 bin（接近纯色/平坦画面），本帧不启用均衡，避免把噪声或单一灰度拉爆。
- 建表在帧边界串行完成：64 个 bin 加 32 个 5 位表项，约 2 k 个 125 MHz 周期（约 16 µs），期间 `in_ready` 拉低；不影响已完成的上一帧显示。
- 本次 demo 构建实测新增约 231 LUT、2.5 个 DRM，APM 不变；逐像素路径只多一个查表选择，没有新增时序关键路径。
- 仍属于全局色调映射，不是局部 HDR；强光过曝或夜间欠曝丢失的细节不能恢复。

仿真入口为 `tb/advanced/tb_hist_equalize.sv`（已并入 `scripts/system/run_advanced_checks.ps1`），覆盖建表、暗区拉伸、单调性、平坦保护和禁用帧直通；原 `tb_adaptive_gamma.sv` 继续覆盖五条固定曲线、auto 迟滞和背压。

上位机启用：

```powershell
python scripts/debug/advanced_control.py --port COM14 --gamma equalize
```

## UART 接口与原子更新

复用原 115200 8N1 UART，无新增引脚。每条指令沿用六字节外包：

```text
A5 5A opcode lo hi (opcode XOR lo XOR hi)
```

| opcode | 16 位 data |
| --- | --- |
| 60..6B | 六个 32 位系数的 12 个半字，小端低半字先发 |
| 6C | flags：bit0 仿射使能；bit1 Gamma 使能；bits4:2 mode；其他必须为 0 |
| 6D | 前 26 个配置字节的 CRC16-CCITT，初值 FFFF，多项式 1021 |
| 47 | 查询提高版状态，分页 FFFB，data 忽略 |

每次提交必须完整写入 13 个半字。CRC 正确才发布 held mailbox；请求/确认 toggle 双触发器跨域，payload 从校验开始保持到接收确认，现有 sys/core 8 ns bundled-data 约束仍保留。两个 mailbox 端共同异步复位、各域同步释放，避免一端复位导致旧请求重放。

配置在**下一条完整帧任务接受时**一次性生效，执行中的两遍处理始终使用同一份配置。等待期间又写入会拒绝并计错，不覆盖现有配置。写配置不被原调试查询/TX busy 阻挡；CRC 不正确、字段缺失、保留位非法或 mode>5 都不应用。关闭提高版参数时这些新指令不生效。

47 回复沿用原 130 字节包和包 CRC，core 的 8 个 32 位字：

| core 字号 | 含义 |
| --- | --- |
| 0、1 | 原状态字、UI 事件计数 |
| 2 | applied_count，已在帧任务开始接受的配置次数 |
| 3、4、5 | 最近一次完成的逻辑帧总周期、第一遍周期、第二遍周期 |
| 6 | 配置拒绝/校验错误计数，在 sys 域打包，未直接跨域读取 |
| 7 | bits15:0 active_flags；bits18:16 自动曲线编号；bits31:24 固定 A1 |

三个周期字段在逻辑任务完成时一起发布；配置 applied_count 表示任务开始接收，**不表示该帧已送到显示器**。新配置刚接受时，周期字段仍可能属于前一任务。总周期包含调度等待，不能简单当成端到端摄像头到屏幕延迟。

上位机工具默认 **dry-run，不打开串口**：

```powershell
python scripts/debug/advanced_control.py --angle 10 --gamma auto --dry-run
```

提高版进入预览或视频后执行（已在 demo 配置实板验证）：

```powershell
python scripts/debug/advanced_control.py --port COM14 --status
python scripts/debug/advanced_control.py --port COM14 --angle 10 --gamma auto
python scripts/debug/advanced_control.py --port COM14 --angle -15 --scale 1.1 --gamma equalize
python scripts/debug/advanced_control.py --port COM14 --gamma off
```

协议每包都带完整的 26 字节，所以工具把上一次**成功应用**的矩阵和 Gamma 模式记在 `data/debug/advanced_control_state.json`，每条命令只改指定的那一项：

| 命令 | 仿射 | Gamma |
| --- | --- | --- |
| `--angle/--scale/--tx/--ty/--matrix` | 更新并打开 | 保持不变 |
| `--gamma brighten/auto/...` | 保持不变 | 更新并打开 |
| `--gamma off` | 保持不变 | 关闭 |
| `--affine` | 恒等（或与给定变换一起）并打开 | 保持不变 |
| `--affine-off` | 关闭 | 保持不变 |
| `--reset` | 关闭 | 关闭 |

例如先转 180° 再单独调亮度、再分别复原：

```powershell
python scripts/debug/advanced_control.py --port COM14 --angle 180
python scripts/debug/advanced_control.py --port COM14 --gamma auto
python scripts/debug/advanced_control.py --port COM14 --gamma off      # 只关 Gamma，保留 180°
python scripts/debug/advanced_control.py --port COM14 --affine-off     # 只关仿射
python scripts/debug/advanced_control.py --port COM14 --reset          # 两者全关
```

状态文件只在板上 `applied_count` 增加 1 且 flags 与请求一致后才写入。换比特流或板上电后若 `--status` 的 flags 与文件不一致，工具会在 stderr 提示并按合并后的配置下发；此时先 `--reset` 对齐最稳妥。`--dry-run` 只打印合并结果，不打开串口也不写状态文件。

（`--port COM14` 是当前板载 CP210x 的实际端口；旧文档里的 COM11 只是示例。）

角度正值表示显示图像顺时针转动（图像 y 向下）；若补偿相机 roll，应由控制端按实际坐标系选择相反角度。`--tx/--ty` 是输出图像平移像素数，`--scale` 是正的输出缩放比例。工具对中心位置取 `(639.5,359.5)`，自动生成逆矩阵，也支持 `--matrix` 直接六系数。使用实际串口需要 pyserial。

工具先确认 A1/FFFB 状态页，再发送整组数据，并检查 applied_count 与错误计数，不将“串口写完”当作生效。视频/预览停止时没有帧边界，配置会等待；超时后不要盲目重发，应先启动视频或读取状态。只支持一个配置客户端，不能与串口助手同时占用端口。

## 检查与后续验收

```powershell
python scripts/build/prepare_advanced_project.py
python scripts/build/check_rtl_layout.py
python scripts/build/check_test_layout.py
python scripts/build/check_advanced_elaboration.py
powershell -ExecutionPolicy Bypass -File scripts/system/run_advanced_checks.ps1 -IcarusBin <iverilog工具目录>
```

`check_advanced_elaboration.py` 使用 pyslang 检查关闭/开启两种板级实例树，只有明确列出的 PLL/DDR/GTP 厂商 IP 黑盒，不等于 PDS 编译或综合通过。仓库原有类型/位宽等 warning 未在本任务大范围整改，日志保存在 build/advanced。

定向测试覆盖：恒等/负边界/90度/剪切/极端系数和坐标停顿；五条 Gamma LUT、逐帧锁定、背压、三帧迟滞和黑边排除；CRC、双时钟 mailbox、帧边界切换、第二遍、错误/背压和共同复位；实际 UART RX/TX、旧查询链路回归；小帧 DDR 仲裁、缓存、双线性/补黑与仿射+Gamma 数值参考；上位机固定数据包及状态解析。

资源策略是共享现有重映射器、不增加第二套 cache/插值器、不增加通用乘法/浮点核，并关闭人工畸变演示逻辑。邮箱在 busy 期间禁止改写，直接复用 shadow 保存 payload；调度器直接复用 active 系数，不重复保存命令矩阵，两组寄存器合计减少 400 位。新增主要为系数/邮箱寄存器、48 位递推加法、亮度统计和少量 LUT ROM；Gamma ROM 逻辑容量 4,352 bit，若 R/B 两读口导致复制则为 5,632 bit，实际 BRAM/LUT 映射以综合报告为准。**不能仅凭 RTL 数量推断一定放得下**，尤其原工程布局资源已紧，新增寄存器也会占用簇。

开启仿射要做两遍 DDR 处理并重复插值，较大旋转会降低缓存局部性。没有测过这一版硬件帧率，不能保证 720p30；若比赛最终需要动态仿射也达到 30 fps，必须根据报告再决定组合映射/带宽/时钟优化。只开 Gamma 不增加第二遍，但也需检查新增逻辑的时序。

下一位在当前 main 上板验证结束后，再对独立提高版 PDS 做综合、布局布线，检查 LUT/FF/簇/BRAM、ROM 是否推断为块存储及 125 MHz 关键路径。通过后再验证动态角度连续更新、边角补黑、强/暗光、前后对比、长时间运行与停止重启，测实际新增帧率和端到端延迟；未完成前不要替换当前可用 bitstream。
