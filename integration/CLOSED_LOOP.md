# 闭环使用说明

本目录把三人的真实 RTL 接在一起。算法边界是：RGB565 原图已在 DDR → Gray8 → 有序角点 → 初值/LM/检查 → 畸变映射表 → 双线性校正 → RGB565 结果写回 DDR。摄像头包装层进一步负责采集和原图缓冲区占用。

## 1. 入口与分层

| 文件 | 作用 |
|---|---|
| `rtl/vision_camera_top.v` | 摄像头入口。接 DVP 和现有 HMIC 端口；每次拍照后自动触发完整图像处理，不向外要求 job_id |
| `rtl/vision_ddr_top.v` | DDR 图像入口。连接真实检测、`calib_top`、mailbox、建表/校正和共享 DDR 服务；支持预存图像联调 |
| `rtl/vision_sequence.v` | 顺序调度、视图计数、角点编号、任务隔离、错误汇总、缓冲区释放；不计算算法数值 |
| `rtl/rgb565_gray_dma.v` | RGB565→Gray8 DDR 转换，源行 padding 不读入图像、目标 padding 不改动，写完成后才报告结束 |
| `rom/*.mem` | 检测所需的方向及高斯权重 ROM，随源码分发，避免依赖队友本地的 `tests/build` |

`vision_camera_top` 复用 `camera_system_top` 的采集、帧所有权和物理 DDR 服务；只向它发送 SNAPSHOT 命令。其原来的建表、校正、显示命令没有同时运行，校正统一由 `vision_ddr_top` 负责。板级摄像头 I²C、PLL、管脚约束和 DDR IP 本身仍由原板级工程提供。

## 2. 摄像头入口：外部只管开始、拍照和取结果

1. `start_valid && start_ready`：启动一次标定任务，锁存 `square_size_fp64`。格长未知可填 FP64 的 1，即 `64'h3ff0000000000000`。
2. 等 `capture_ready=1`，调整棋盘姿态，再用 `capture_valid` 握手拍一张。握手是“允许开始等待摄像头完整帧”，不是该拍已经采完。
3. 每张图完成灰度和角点处理后，`debug_view` 增加，再等待下一次 `capture_ready`。重复 `PAR_VIEWS` 次。不要把 capture_valid 永久绑高，否则会自动连拍，容易得到几张姿态接近的图。
4. 最后一张完成后自动做标定、建表，并**校正最后一张原图**，不再要求额外拍一张。
5. `rsp_valid=1` 时查看 `rsp_status`。仅 0 表示结果图可用；其地址/跨度/尺寸由 `result_base/result_stride/result_width/result_height` 给出。结果参数和 RMS 同时保留。
6. 读取/使用结果后拉高 `rsp_ready`。接收响应结束本次任务，下一次 start 可以覆盖结果区。若需长期保存结果，应在接收响应前复制或明确保证不发下一次 start。

外部不需要提供 job_id、view_id、point_index、corner_last。内部仍自动分配递增 job_id，并按握手产生 view/index/last；这样保留了原有模块的错误检查，不需要删掉各人已经验证过的接口。`debug_job/debug_view/debug_phase` 仅用于调试。

这是“标定并校正最后一张图”的闭环入口；目前再次 start 会重新标定。只标定一次、持续校正普通视频帧仍可使用原 `camera_system_top` 的参数建表和校正命令接口，尚未作为本包装层的新操作码开放。此入口不驱动 HDMI 显示；任务边界是写回 DDR。

## 3. DDR 入口：图像描述符和所有权

`vision_ddr_top` 的 start/rsp 与上面相同。它用 `frame_valid/frame_ready` 代替拍照触发，每张图提供：

| 字段 | 含义 |
|---|---|
| `frame_base` | RGB565 第一个像素的 32 位 DDR 字节地址 |
| `frame_stride` | 每行字节数，至少 `2*WIDTH` |
| `frame_capacity` | 从 base 开始可访问的容量；须容纳最后一行有效像素 |
| `frame_status` | 0=图像可用；非零=采集/上游失败，终止整个任务 |

同一时刻只持有一个原图。成功描述符被接收后，上游不得修改图像，直到 `frame_release_valid && frame_release_ready`。前 V−1 张在检测完成后释放；最后一张保留到校正结束或失败清理。非零 frame_status 不代表取得了原图，所以不会为它发释放请求。

释放请求也遵守 valid/ready，上游必须接收；不要等最终 rsp 才接收 release，否则调度会按协议等待释放。非法地址、容量或重叠会在访问 DDR 前拒绝；如果已接收的是成功描述符，仍会归还该图像所有权。

逻辑 DDR 五通道与现有服务相同：32 位字节地址/长度、32 位数据、4 位 keep、16 位 tag，小端排列。摄像头包装层已经把这些口接到物理服务，不需要再次接一套 DDR 控制器。

## 4. 配置和内存

`WIDTH/HEIGHT/DEPTH` 是顶层编译参数，不是逐帧变化的输入。V/R/C 取自 `parameter/rtl/common/calib_config.vh`，已经传到检测排序和精化模块，默认仍是 3 张、5×8。标定模块自身上限更高；**当前完整检测链的排序缓冲容量是 64 点，完整闭环仅接受 R*C≤64**，超出会报 BAD_CONFIG，避免静默截断。分辨率接受 32…1920 × 32…1080，金字塔 1…4 层，最深层宽高至少 16；这些是接口检查范围，不代表每种配置都已做数值验收。

默认地址：

| 区域 | 基址 | 使用大小 |
|---|---|---|
| 摄像头原图槽 | `0x00000000` | 槽容量 4 MiB；实际图像 `2*WIDTH*HEIGHT` 字节 |
| 校正输出 | `0x01000000` | `2*WIDTH*HEIGHT` |
| map_x | `0x02000000` | `4*WIDTH*HEIGHT` |
| map_y | `0x02800000` | `4*WIDTH*HEIGHT` |
| Gray8 | `0x03000000` | `WIDTH*HEIGHT` |

灰度、结果和两张表做两两重叠/溢出检查；每个输入描述符也检查其图像覆盖范围不与它们重叠。检测响应图导出在闭环入口关闭，避免另分调试区和增加访存；不影响其独立模块原有功能。

灰度展开沿用现有 RGB565 的低位补零约定，转换公式为 `(299*R+587*G+114*B+500)/1000`。当前灰度 DMA 每像素一笔 2 字节读和一笔 1 字节写，优先保证正确性，尚未做行缓存/突发聚合优化，不能据此宣称实时帧率。校正使用原有黑色边界模式。

## 5. 时序与错误

- `valid && !ready` 时保持载荷；只有握手推进计数。done 电平经过调度状态隔离，不会把上一张的 done 当成下一张完成。
- 检测 `status==01` 只代表流程结束。还要检查 grid_ok、total 与实际接收点数，均正确才提交视图成功；无棋盘、少点、多点均使任务失败。
- 检测 status==10 未提供细分原因，映射 CALIB_INVALID=4；灰度 DMA 和建表/校正可明确识别的访存失败映射 MEM_ERROR=5。不能从现有检测状态虚构更具体的错误原因。
- mailbox 必须同时收到相机参数包和同 job 的成功响应，才允许建表。诊断 ready 已接 1，并锁存 RMS。
- 相机只在收到拍照确认且有空闲槽时采集；前一张未释放时不能覆盖。最后一张在整个 LM 阶段保持只读。
- 复位会取消任务；物理 DDR 在途时应与 HMIC 一起复位。没有用任意短超时强行取消已发出的 DDR 事务。

联调还修复了检测 DDR 仲裁器的实际死锁：原来 valid 依赖 ready，接到等待 valid 再授权的系统服务后双方互等。现已使 valid 独立于 ready，并在背压期间锁定选择；零长度事务在本地报错，不再等下游 ready。

## 6. 验证入口

```powershell
# 联合编译、展开及快速回归（含新的灰度、闭环控制、摄像头真实失败路径）
powershell -NoProfile -ExecutionPolicy Bypass -File integration/run_checks.ps1

# 三张独立透视棋盘图，真实检测/初值/LM/建表/校正；耗时较长
node integration/tools/generate_board_images.js
powershell -NoProfile -ExecutionPolicy Bypass -File integration/run_checks.ps1 -OnlyTest tb_vision_numeric
node integration/tools/check_numeric_output.js

# 非默认配置的接口/缓存/输出通路回归（检测和标定数值使用明确的测试替身）
powershell -NoProfile -ExecutionPolicy Bypass -File integration/run_checks.ps1 -OnlyTest tb_vision_ddr -Configuration PAR_VIEWS=4,PAR_BOARD_ROWS=6,PAR_BOARD_COLS=7
```

各测试范围：

- `tb_gray_dma`：逐像素公式参考、奇地址、跨物理块、padding、读写失败、背压、重试。
- `tb_arbiter_ready`：下游等待 valid、背压时新客户端到来、请求载荷稳定、零长度错误。
- `tb_vision_ddr`：真实灰度、角点缓存、mailbox、建表、校正与 DDR；检测点流和最终标定数值明确用测试替身，检查成功/少点/多点/采集及访存失败/连续任务，逐像素比对输出。
- `tb_vision_camera`：没有算法替身。实际 DVP→FIFO→HMIC→灰度→检测，两次空白图均返回 NO_BOARD，检查所有权释放、无错误输出写入及连续重试。
- `tb_vision_numeric`：没有 force/算法替身。三张独立渲染的透视棋盘图走完整算法链；再用软件 Brown 映射和整数插值逐像素核对 DDR 图像，并检查估计焦距与渲染参数接近。

命名连线由 `tools/generate_camera_wiring.js` 生成两个顶层；生成后的 Verilog 已放入 rtl，可直接编译，不要求构建机安装 Node。ROM 的标准 C++ 生成源在 `tools/generate_roms.cpp`，已生成的 `.mem` 随源码分发；ModelSim 和综合应能访问 `integration/rom/` 相对路径。`run_checks.ps1` 自动放置这些文件到仿真工作目录。

生产 RTL 清单为 `integration/files.f`，从仓库根使用 `vlog -sv -f integration/files.f`；清单包含 package 的正确顺序，不包含测试平台。导入综合工程时还应添加三个 ROM 初始化文件并保留路径。新增源文件后运行 `node integration/tools/list_sources.js` 更新清单。

当前验证不包含综合时序、资源容量或上板测试；不要把仿真完成等同于 100 MHz 时序收敛或实时吞吐。

## 7. 本次实际结果（2026-10-02）

| 验证 | 实际结果 |
|---|---|
| 同库编译与展开 | 三个原顶层和两个新闭环顶层均通过 |
| 快速回归 | 17 个测试平台通过；含新仲裁背压用例和摄像头两次真实失败恢复 |
| 闭环控制/像素通路 | 默认 3×5×8、非默认 4×6×7 各通过 11 项任务场景；这两项明确替换检测/标定数值输出 |
| 真实数值闭环 | 三张 160×120 透视 RGB565 图，真实检测各 40 点，真实 init/LM、建表、校正全部成功；无算法替身 |
| 标定 RMS | 0.1081369486 像素 |
| 焦距检查 | 渲染 fx=fy=150，估计 fx=147.9495087、fy=148.2970581，最大相对偏差约 1.37% |
| 输出图检查 | 19,200 个 RGB565 像素与独立软件映射/插值参考完全一致 |
| 真实数值用例周期数 | 376,614,393；仅此 160×120 用例，不代表默认 1280×720 性能 |

紧凑证据保存于 [数值结果](validation/numeric_160x120.json) 和 [真实 RTL 检测出的角点](validation/detected_corners.csv)。完整日志、源图、相机参数和输出图逐字节转储在忽略目录 `integration/build/`；上述脚本可重新生成。此用例是独立合成图，不是实拍摄像头标定验收。
