# 棋盘角点检测：分层、接口与验证

检测入口是 [corner_detect_ddr_top.v](../../../rtl/image/features/corner_detect_ddr_top.v)。输入 DDR 中的 Gray8 图像，输出原图坐标下的有序 FP32 角点及检测结果。当前系统在检测前由集成层将 RGB565 转为 Gray8；检测本身不读取 RGB565。

板级 `vision_ddr_top` 设置 `REPLAY_RESP=1`：第一遍只统计响应最大值，等待阈值乘法完成后，复位并重新启动同一条灰度/梯度/响应流水线；第二遍直接送行缓存及 NMS，不保存整幅响应图。两遍之间灰度图必须保持不变，第二遍产生的响应不再参与阈值统计。候选流完全排空后才允许筛选器读取共享灰度口。精度、阈值及相等峰值规则保持不变，增加的是一次响应计算扫描。

`shi_tomasi_replay` 定义在已有 `shi_tomasi_ctrl.sv` 中，无需新增 PDS 源文件。独立检测入口默认 `REPLAY_RESP=0`，保留原存储/响应图导出路径；需要完整响应图导出时使用该模式。重算模式要求 `cfg_resp_dump_en=0`，`detect_ctrl` 对不兼容请求以 `status=11` 拒绝，不会返回伪造的响应图。板级原本就关闭此调试导出功能。灰度图 RAM 尚未改成 DDR 随机访问缓存，当前修改不能保证整板存储资源足够。

## 分层与模块职责

| 层级 | 模块 | 实现方法与控制 |
| --- | --- | --- |
| 帧调度 | frame_task_ctrl、corner_detect_ddr_top | 灰度搬入 → 金字塔 → 检测 → 可选响应图写回 → 完成；同一实例支持连续任务 |
| DDR | gray_fetch、raster_dma、ddr_port_adapter | 按行读取灰度，将字节事务转换为本地像素；保持行跨度和返回握手 |
| 仲裁/导出 | ddr_port_arbiter、resp_ddr_writer | 多客户端读写仲裁；响应图写完成后才能报告完成 |
| 多尺度 | pyramid_ctrl、downsample2x、detect_ctrl | 生成缩图、逐层检测并决定回退；输出坐标恢复到原图尺度 |
| 响应前端 | window3x3、Sobel/张量核、min_eigen_core、shi_tomasi_ctrl | 梯度与窗口累加得到 Shi-Tomasi 响应；先求全图阈值，再扫描 NMS 候选 |
| 候选筛选 | candidate_filter_ctrl、merge/nearest、ring_check | 候选合并、近邻尺度估计、圆环亮暗结构检查，逐候选推进 |
| 网格组织 | index_sort、grid_order_ctrl、grid_validate | 多方向投影、迭代归并排序、按行组织网格并比较代价；保存最佳网格 |
| 亚像素 | subpixel_ctrl、subpixel_accum、bilinear_core、tensor_solve | 读取局部窗口，累计梯度方程、解位移并迭代，达到收敛或上限后输出 |
| 整网格精化 | grid_refine_ctrl | 逐点精化并验证最终网格，一致后输出行列序角点 |
| 公共基础 | rtl/common、rtl/arithmetic | 同步 FIFO、RAM、复位同步及 FP32/FP64 运算；延迟和背压以模块端口契约为准 |

目录中已有独立运算/模块测试，历史 M1～M8 报告已合并为本说明。数学原理见 [算法总览](ALGORITHM_OVERVIEW.md)，C++ 到 RTL 的设计思路见 [分层规划](VERILOG_DESIGN_PLAN.md)。

## 对外接口

| 接口 | 契约 |
| --- | --- |
| process_frame | busy=0 时单拍启动；配置在启动前稳定并在帧期间保持 |
| busy / done | busy 表示在处理；done 完成后保持，下次启动清零，不能按通用单拍响应处理 |
| status / out_grid_ok | 检测内部 status=01 表示处理成功，10 表示子模块错误；是否有可用棋盘还须检查 out_grid_ok 和点数 |
| cfg_gray_base/stride/w/h | Gray8 的字节基址、行跨度与尺寸，不能传 RGB565 原图基址 |
| out_valid/out_ready/x/y | 标准握手；背压时坐标保持。x/y 为 FP32，像素中心坐标，x 向右、y 向下 |
| out_total | 成功网格共 P=ROWS×COLS 点，顺序 row×COLS+col；默认 40 点 |
| cfg_resp_dump_en/base | 可选的 FP32 响应图导出，当前整机闭环关闭，独立调试可用 |
| m_* DDR | 模块对系统发出的逻辑 DDR 请求/返回，字节地址及字节长度，32 位数据、4 位 keep、16 位 tag |
| ext_* DDR | 外部客户端接入检测内部仲裁器的接口，方向与 m_* 不同，不能把两者混用 |

检测不直接生成标定 job/view/index；[vision_sequence](../../../rtl/control/system/vision_sequence.v) 将点流编号并把原始完成状态转换为系统状态。点数不足、网格无效或失败响应不能作为完整标定视图提交。

ROWS/COLS 来自共享配置。完整链路每图最多 64 点；改变棋盘或图像尺寸需重编译和重测。所有 DDR 区域由系统顶层分配，独立测试中的地址不作为整机默认地址。

## 实现时必须保持的规则

- Gray RAM 为同步读，读地址、使能和返回拍必须对应；不能把不同 RAM 的等待周期混用。
- 排序按 key 比较，平局按原索引；负浮点投影需先做单调位模式转换，不能直接按原始无符号位模式排序。
- 两遍响应扫描之间要有完整帧屏障；响应导出开启时，写端须先就绪，避免两边互等。
- GF/PYR 阶段对检测核心做帧级软复位，进入检测前释放。变更此时序必须重跑连续帧测试。
- 历史连续帧测试曾暴露浮点弹性通路在背压下残留 token/卡住的问题；当前保留帧级复位。现有回归通过不等于所有长期背压组合已证明无死锁，该限制不能随着开发日志删除而遗忘。
- 方向和高斯权重 ROM 来自 integration/rom；综合与仿真都需要正确加载。

## 验证入口

从仓库根目录运行：

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -OnlyTest tb_detection_compat
~~~

该测试使用三张独立渲染的透视棋盘图，运行真实检测并将 120 个角点与 data/system/detected_corners.csv 逐位比较，检查完成状态和 DDR 协议；不执行后续 LM。
独立测试保留在 sim，参考向量/导出工具保留在 tests。连续帧重点看 tb_frame_top、tb_frame_b5x2；DDR 仲裁看 tb_arbiter。模块头部注释是具体信号时序的依据。
