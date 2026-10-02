# 三分支合并与接口检查

> 本文是合并时的历史检查记录。随后已经补上实际闭环、图像格式转换、帧协议适配和参数完成屏障，并修复联调发现的 DDR 仲裁死锁；当前使用方式与剩余限制见 [闭环使用说明](CLOSED_LOOP.md)。

检查日期：2026-10-02。合入本地 `main` 的分支端点：`para` / `17afd55`、`origin/corner` / `91fbddb`、`origin/undistort` / `0b17026`。

**结论：三个子系统可以放在同一源码库编译，但尚未组成摄像头→检测→标定→校正的自动闭环。** 下面区分已对齐接口与尚缺的连接逻辑；单独子系统测试通过不代表全链路已通过。

## 1. 源码与去重

| 功能 | 唯一实现 / 入口 | 处理 |
|---|---|---|
| 检测 | `algorithom/closer2fpga/rtl/detect/corner_detect_ddr_top.v` | 保留 corner 实现 |
| 标定 | `parameter/rtl/control/calib_top.v` | 保留 para 实现；删除未实现、未被调用的 `undistort/biaoding.v` 占位接口 |
| 采集、DDR、校正 | `undistort/rtl/top/camera_system_top.v` | 保留 undistort 实现；该顶层尚未实例化前两部分 |
| 同步 FIFO | `algorithom/closer2fpga/rtl/common/sync_fifo.v` | 删除 undistort 中无人调用的同名版本；使用 `DATA_WIDTH/ADDR_WIDTH` 参数及其同步复位语义 |
| 复位同步 | `algorithom/closer2fpga/rtl/common/reset_sync.v` | 删除重复实现，保留两级同步并补上 `ASYNC_REG` 属性 |
| 检测侧事务管理 | `algorithom/closer2fpga/rtl/mem/ddr_port_adapter.v` | 逻辑请求在途管理与零长度处理，仍然需要保留 |
| 真实 DDR 适配 | `undistort/rtl/mem/hmic_ddr_adapter.v` | 从 `ddr_port_adapter` 改名，更新顶层及 TB 引用，避免编译覆盖 |

两套 DDR adapter 是不同层次的模块，不能删掉其中一个。FP32 流水运算、标定 FP64 运算与校正 FP32 服务的接口、延迟和数值实现也不同，本次不按相似功能强行替换。现有 Brown / 插值实现同样不是可直接互换的重复文件。

根 `.gitignore` 原先的 `sim/` 会屏蔽任意层级的仿真目录，已改为 `/sim/`，保留根目录临时实验的忽略，同时将参数模块已有的测试源码、脚本和向量纳入版本管理。各子系统生成目录仍由本地 `.gitignore` 忽略。

## 2. 检测 → 标定：缺少帧协议适配

FP32 x/y 的数值类型、原图像素坐标约定和默认 5×8 的行优先顺序一致，但端口不能直接一一相接。

| 检测端 | 标定端 / 适配要求 |
|---|---|
| `process_frame` 单拍，`busy`，保持型 `done` | 顶层锁存 job/view/config，空闲时发一个启动脉冲；一帧的 done 只处理一次，新启动后等待旧 done 清零，不能把旧电平当成新完成 |
| `out_valid/out_ready/out_x/out_y` | 对接 `corner_valid/corner_ready/corner_x_fp32/corner_y_fp32` |
| 不输出 job/view/index/last | 适配器补 `corner_job_id/view_id`；只在角点握手时增加 index；最后点产生 last。不能按时钟周期计数 |
| `status[1:0]` | **不是统一错误码**：`01` 表示帧流程正常结束，可能没有棋盘；不能零扩展成标定 status |
| `out_grid_ok/out_total` | 仅 `status==01 && grid_ok && total==PAR_POINTS && 实收点数==PAR_POINTS` 可提交 `view_rsp_status=0`；正常结束但无有效棋盘→`NO_BOARD=2` |
| `status==10` | 合并了读、写、检测内部失败，当前端口无法区分原因；需补错误原因输出或明确映射为上层通用失败，不能一概宣称是 DDR 错误 |

每个 job 先与 `calib_top.collect_*` 握手，再依次提交 view 0…V−1。不要等 `cmd_ready` 才开始送角点：它只在所有角点及视图成功响应齐备后才拉高。收集失败时标定可直接输出诊断和失败响应，不会等启动命令。

适配器需要将保持型 done 转成**可背压的单次 `view_rsp_valid` 事务**，响应未被接收时保持载荷。失败后必须停止或排空旧检测任务，再开启下一 job，防止旧点混入新任务。

## 3. 标定 → 校正：参数布局匹配，必须经完成屏障

9 个 FP32 从低位到高位均为 `fx,fy,cx,cy,k1,k2,k3,p1,p2`，共 288 位。注意标定内部状态中的畸变字段顺序不同，外部应使用 `camera_params`，不能接内部 state。

推荐使用已有 `undistort/rtl/top/calibration_mailbox.v`：

| `calib_top` | `calibration_mailbox` |
|---|---|
| `camera_valid / camera_ready` | `param_valid / param_ready`（流握手） |
| `camera_usable` | `param_camera_valid`（数值有效字段，与上行 valid 含义不同） |
| `camera_calib_id` | `param_calib_id` |
| `camera_width / camera_height` | `param_width / param_height` |
| `camera_params` | `param_values` |
| `rsp_valid / rsp_ready / rsp_job_id / rsp_status` | `calib_rsp_valid / calib_rsp_ready / calib_rsp_id / calib_rsp_status` |

顶层应为同一 job 启动 mailbox 的 `begin_*`。`diag_ready` 必须接诊断接收器；暂不使用时绑 1。标定等待 camera 和 diag 两路完成后才发 rsp，不能让诊断阻塞整个系统。

mailbox `result_status==0` 后才向 `camera_system_top` 发 `cmd_opcode=1` 建表，传入 result 的 id、width、height、values 并置 `cmd_camera_valid=1`。等待建表成功才能使用新图表。失败 result 必须接收并报告，不能当成建表命令。mailbox 当前只是独立模块，**系统顶层尚未把它和真实 calib_top 接起来**。

## 4. 检测 ↔ DDR：逻辑协议一致，图像格式不一致

检测顶层 `m_*` 接系统顶层 `ext_*`，五个通道的方向、32 位字节地址/长度、32 位数据、4 位 keep、16 位 tag、last/error 和背压语义匹配：

| 检测端前缀 | 系统端前缀 | 字段差异 |
|---|---|---|
| `m_rd_req_*` | `ext_rd_*` | `len_bytes` → `len` |
| `m_rd_ret_*` | `ext_r_*` | `data/keep/tag/last/error` 原样连接 |
| `m_wr_req_*` | `ext_wr_*` | `len_bytes` → `len` |
| `m_wr_dat_*` | `ext_w_*` | `data/keep/last` 原样连接 |
| `m_wr_cplt_*` | `ext_b_*` | `tag/error` 原样连接 |

每行的 valid/ready 也对应连接。不要把检测顶层自身的 `ext_*` 扩展客户端口误当成系统 DDR 出口；未使用的检测扩展请求 valid 绑 0，载荷绑已知值。

尚需解决：

1. **图像格式**：采集和校正使用 RGB565，检测 `gray_fetch` 每像素读一个 Gray8 字节。不能把 `raw_pin_base` 直接作为 `cfg_gray_base`。需要 RGB565→灰度→独立 DDR 灰度区的转换路径，且灰度算法与检测参考数据一致。原 README 的 BGR888 输入约定不是当前顶层实现。
2. **地址冲突**：检测示例灰度区 `0x00001000`、响应图区 `0x00300000` 都落在系统默认 raw 区 `[0,0x00c00000)` 内，直接复用会覆盖原图。正式分配需同时覆盖 raw、output、map_x、map_y、gray、response，逐个检查范围互斥；示例地址不能作为系统配置直接照抄。
3. **图像尺寸**：检测 `W0/H0/DEPTH` 是编译参数，部分 cfg 端口虽然可输入不同尺寸，但检测与响应长度仍依赖编译参数。联调必须匹配编译尺寸、金字塔深度及 cfg，不能认为任意运行时分辨率均已支持。
4. **板型**：参数算法支持可配置 V/R/C；检测 `detect_ctrl.sv` 中仍有 `.ROWS(5), .COLS(8)`。当前端到端只认可 5×8；扩展板型时需传递检测侧参数并重新验证，不能只改 `calib_config.vh`。
5. **帧所有权**：读取 raw 前通过 `raw_pin_*` 取得槽位；转换/读取结束前不得 release。若同一帧还需校正，要另外明确它的后续所有权，不能假定 release 后仍保留该帧。
6. **硬件完成边界**：HMIC 适配器使用 `axi_wusero_last` 作为写完成信号。仿真模型验证了该约定，上板仍需核实 IP 对写后读可见性的保证；这里没有宣称已完成上板验证。

## 5. 联合检查与复现

从仓库根运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File integration/run_checks.ps1
```

脚本检查生产 RTL 同名模块冲突，把三个子系统编译进**同一个 work 库**，分别展开真实 `calib_top`、`corner_detect_ddr_top`、`camera_system_top`，随后执行参数 mailbox、DDR 适配、参数存储、建表、CDC FIFO、采集、HDMI 桥、校正、相机系统、检测 DDR 仲裁、角点缓存、标定顶层控制的 12 个已有自检 TB。完整日志及编译清单在忽略目录 `integration/build/`。可用 `-OnlyTest tb_calib_top` 单独复测一项。

本次实测：三个顶层展开通过；上述 12 项全部通过，其中角点缓存 21 个场景 / 3240 次检查、标定顶层 45 个控制场景，均为 0 错误。标定顶层控制测试显式模拟子模块响应，用于检查调度与握手，不是再次运行完整 LM 数值算法；顶层展开使用的则是真实全部子模块。

展开仍有检测分支原有的 5 条位宽警告：`grid_order_ctrl` 四个索引 RAM 的窄输出接到 32 位线后再次取低位，以及 `detect_ctrl` 的 20 位 dump 地址接较窄的分层响应 RAM。默认尺寸下分别是零扩展和有效地址范围内的截断；本次记录这些警告，没有宣称所有可配置尺寸已经验证。扩展检测尺寸时应统一这些地址位宽。

此次为兼容本机 ModelSim 10.1c，将 `window3x3` 的参数位宽转换表达式改成相同值的定宽 localparam 赋值，冻结阈值仍是 6。

这些验证不包含真实三部分闭环、重新运行全部数值回归、综合资源/时序或上板。检测交接文档中记录的弹性浮点模块潜在背压死锁仍属原有已知限制，本次没有用编译通过替代其验证结论。
