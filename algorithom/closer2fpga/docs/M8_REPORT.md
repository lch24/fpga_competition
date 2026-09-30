# M8 说明文档：DDR 总线整合（多客户端仲裁 + 帧级任务流 + 联调顶层）

- 负责人：苏晨（corner 分支）
- 状态：已完成（待提交）
- 对应规划：M7 之后"收敛为单一 DDR 端口、对接真实控制器"的整合里程碑
- 用途：面向**队友联调交接**与**后续开发者**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威。

---

## 1. 范围与验收

M8 把 M7 各分散 DMA 客户端收敛为**单一 DDR 端口** + **帧级一键处理接口**，形成可交队友联调的顶层子系统：

| 交付 | 验收结果 |
|---|---|
| `ddr_port_arbiter.v`（读/写独立 round-robin 仲裁，在途单笔，零长吸收，端口数组化） | 单元 tb_arbiter 7 用例 ALL PASSED（errs=0 proto=0） |
| `frame_task_ctrl.v`（帧级任务流：GF → PYR → DET → RESP 串行调度，电平沿检测） | 集成通过（见 tb_frame_top） |
| `corner_detect_ddr_top.v`（联调顶层：arbiter + gray_fetch + pyramid + detect_ctrl + resp_ddr_writer + gray RAM） | 集成通过（见 tb_frame_top） |
| `tb_arbiter.sv` | 两读交错/两写交错/读写并发×2/轮转公平/在途阻塞×2/零长伪返回，7 用例全过 proto=0 |
| `tb_frame_top.sv` | big 帧 A（pyramid+resp dump）40/40 + resp 读回 921600B 0 错；帧 B（连续帧复用+resp_base 重锁存）40/40 + 读回 0 错；帧 C（dump 关闭）40/40；board5x8 帧 D（DEPTH=1）40/40 + 读回 104448B 0 错；err=0 proto_violations=0 |

**对外联调接口（top 端口）**：
- `process_frame` + cfg 总线（gray base/stride/w/h、ram_base、resp_base、resp_dump_en、pyr_en）——上层只发"处理一帧"命令；
- `m_*` DDR 端口（单组，模型侧直连，方向同 ddr_memory_model）——**对接强文韬真实 DDR 服务层控制器的替换点**；
- `ext_rd_*`（读客户端槽 1）——下游读回响应图/标定用；
- `ext_wr_*`（写客户端槽 1）——上游预载灰度（相机通路写入点）；
- 40 点输出流 + busy/done/status。

**不在 M8 范围**：OV5640 通路、真实控制器时序对接、PDS 综合、标定/remap。

---

## 2. 架构总览

```
上层（队友）: process_frame + cfg ─┐
                                  ▼
corner_detect_ddr_top
  ├─ frame_task_ctrl：GF → PYR → DET → RESP（串行，电平沿检测，status 汇总）
  ├─ gray_fetch ──────────────┐（读客户端 0）
  ├─ resp_ddr_writer ─────────┤（写客户端 0）
  ├─ ddr_port_arbiter #(N_RD=2, N_WR=2)
  │    ├─ 读：gray_fetch / ext_rd(读客户端 1)
  │    └─ 写：resp_ddr_writer / ext_wr(写客户端 1)
  │         └─ m_* 单组端口 → ddr_memory_model / 真实控制器（强文韬侧替换点）
  ├─ gray RAM（片上，同步写 / registered 读 1 拍；读写口阶段互斥 mux）
  ├─ pyramid_ctrl（cfg_pyr_en 可选生成 L1..）
  └─ detect_ctrl（DEPTH 层槽位链；resp_dump_* 直连 resp_ddr_writer.in_*）
```

帧级流程（frame_task_ctrl）：`IDLE(process_frame) → GF(gray_fetch，等 done 沿) → PYR(可选) → DET(detect，等 done 沿) → RESP(dump_en 时等 writer done) → done`。

---

## 3. 模块详解

### 3.1 `ddr_port_arbiter.v`（rtl/mem/，M8 子代理 J）

读/写**独立**状态机 + 独立轮转指针。空闲时自指针循环扫描第一个 `req_valid` 客户端（旋转向量 + 优先级编码，N_RD/N_WR 参数化）；被选客户端 `ready=m_ready`，其余恒 0；**在途单笔**：返回/完成通道只回被选客户端，`rd_ret_last` 接受（读）/ `wr_cplt` 接受（写）后释放并轮转指针；在途期间所有客户端 req_ready=0（请求不吞，事务完成后按轮转接受）。`wr_dat` 仅在写 ACTIVE 透传被选客户端，不在途客户端数据 valid 屏蔽 → 模型"无在途写请求的写数据"零违规。**零长（len==0）**：镜像 ddr_port_adapter——客户端侧正常握手但不发模型，本地回伪返回/伪完成（error=1），模型零违规、上层可感知。tag 在请求握手时锁存回带（为未来乱序路由保留）。

### 3.2 `frame_task_ctrl.v`（rtl/detect/，M8 子代理 K）

帧级任务流状态机。关键决策：
- 所有等待用"done_d 每拍跟踪 + 前一拍电平判 0→1 沿"（防值拷贝冻结/防漏沿，M6/M7 纪律）；
- **det_status 语义**：01（有角点）/10（无角点）均视为正常帧完成（status=01），仅 11 为错误（status=10）；
- **dump_en=1 时 w_start 与 det_start 同拍发出**——detect 的 ST_DUMP 需 `resp_dump_ready` 才推进（resp_dump_done → det done），而 writer 须 start 后才拉 in_ready，串行启动必死锁（M7 tb_resp_ddr 同款手法）；
- 端口名 `process` 在 ModelSim 10.6e -sv 下是保留字 → 改名 `process_frame`（三文件一致）。

### 3.3 `corner_detect_ddr_top.v`（rtl/detect/，M8 子代理 K）

联调顶层。arbiter N_RD=2/N_WR=2：读槽 0=gray_fetch、1=ext_rd；写槽 0=resp_ddr_writer、1=ext_wr。gray RAM 读写口按子模块 busy 阶段互斥 mux（参考 tb_detect_ddr active 选通）。`cfg_words=(W0>>(DEPTH-1))*(H0>>(DEPTH-1))` 参数化。子模块 cfg 由 top 直连 cfg 总线（子模块在自身 start 拍采样，帧期间 cfg 稳定）。

**帧级软复位（连续帧关键）**：`det_rst_n = rst_n && !(f_busy || pyr_busy)`——GF/PYR 阶段给 detect 每帧软复位（见 §5 问题 1），det_start 时已释放，保证连续帧复用干净。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_arbiter.sv` | 两读交错 / 两写交错 / 读写并发×2 / round-robin 公平（各 10 笔无饿死）/ 在途阻塞×2 / 零长伪返回 | 7 用例 ALL PASSED，proto=0 |
| `tb_frame_top.sv` | big 帧 A（pyramid+resp dump）/ 帧 B（连续帧复用+resp_base 重锁存）/ 帧 C（dump 关闭）/ board5x8 帧 D（DEPTH=1） | 40/40×4，resp 读回 921600B×2 + 104448B 均 err=0，proto=0 |

模型配置：LATENCY_MIN=1..6、JITTER/BACKPRESSURE/PROTOCOL_CHECKS 全开。仿真 332ms / 真实 55 分钟（vsim 串行，-suppress vsim-3829）。向量复用 M7：`m6_big_gray`/`m6_chain_big`/`m5_board5x8_gray`/`m6_chain_board5x8`/`m7_resp_{big,board5x8}`（只读）。

---

## 5. 过程中发现并修复的问题

1. **grid_order_ctrl 连续帧死锁（M8 集成发现的既有潜在 bug，K 定位）**：`grid_order_ctrl` 主状态机 `S_DONE: if(start) 只转移 S_IDLE 不锁存`，而 detect 的 order_start 是 1 拍脉冲——**连续帧第 2 帧**起 order 残留 S_DONE 吃掉 start 卡 S_IDLE，done/gok 残留使 detect 误进 ST_ORDER 死锁（看门狗定位 det stage=2）。M6/M7 各 TB 每场景只用独立实例/未连续复用，从未触发。**规避（不动已验证模块）**：top 内帧级软复位 `det_rst_n = rst_n && !(f_busy||pyr_busy)`，GF/PYR 阶段复位 detect（含 order），det_start 时已回 IDLE。**遗留**：根治需改 grid_order_ctrl（S_DONE 遇 start 锁存复位），但须 M4 tb_order 回归 + 全链重对拍，M8 不做，记入 §6。
2. **`process` 关键字冲突**：ModelSim 10.6e -sv 保留字 → 帧命令端口改名 `process_frame`。
3. **ext_rd/ext_wr 方向初版写反**：req/dat 为输入、ret/done 为输出，已修正。
4. **ST_DUMP 串行启动死锁**：dump_en 时 writer 必须先 start 才有 in_ready，而 detect 的 ST_DUMP 等 resp_dump_ready 才推进——串行启动（det 完成再启 writer）必死锁；w_start 与 det_start 同拍解决（见 §3.2）。
5. **vsim-3839 multiply-driven 警告（28 条）**：ModelSim 10.6e 对含数组端口模块的端口连接误报（数组端口驱动方向被误判为双驱动）；已用最小实验 + 功能结果（4 帧全过）双重确认不影响方向与数据。

---

## 6. 遗留事项与接口契约（联调交接重点）

- **联调接口（队友对接点）**：
  - 强文韬（DDR 服务层）：top `m_*` 单组端口（方向同 ddr_memory_model）——真实控制器就绪后直接替换模型；
  - 上游（相机→灰度）：`ext_wr_*` 写客户端槽 1 把 L0 灰度写入 `cfg_gray_base` 区（top 帧级自动拉取）；
  - 下游（标定/remap）：40 点流（out_valid/ready/x/y/total/grid_ok）+ `ext_rd_*` 读响应图；
  - 上层：`process_frame` + cfg 总线 + busy/done/status。
- **grid_order_ctrl S_DONE 连续帧 bug 待根治**：现以 top 帧级软复位规避（`det_rst_n = rst_n && !(f_busy||pyr_busy)`）；根治须改 grid_order_ctrl 并回归 M4 tb_order + 全链重对拍。
- **帧级软复位纪律**：detect 在 GF/PYR 阶段保持复位（f_busy||pyr_busy 期间），det_start 前释放；任何改变阶段时序的改动须重跑 tb_frame_top 连续帧用例。
- **握手纪律（沿用）**：等待电平"前一拍值判沿"；请求"组合 valid+pending&&ready + ready 回落判接受"；数据非尾"wr_off_r 前进判接受"、尾拍"ready 回落判接受"。
- **范围外**：OV5640 通路、真实控制器时序、PDS 综合、标定/remap。
