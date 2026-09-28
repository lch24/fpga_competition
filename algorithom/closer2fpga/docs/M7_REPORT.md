# M7 说明文档：DDR 对接（raster_dma / byte_packer / ddr_port_adapter / gray_fetch + 灰度经 DDR 全链）

- 负责人：苏晨（corner 分支）
- 状态：已完成（待提交）
- 对应规划：M6.3 延后的 DDR 对接（VERILOG_DESIGN_PLAN §3.2 事务契约）
- 用途：面向**队友交接**与**后续开发者理解原理**。模块级接口契约以各 `.v/.sv` 文件头部注释为权威，本文档讲清"为什么这样设计"和"怎么验证的"。

---

## 1. 范围与验收

M7 把 M6 遗留的 DDR 对接落地。权威为 **M1 交付的 `ddr_memory_model.sv`（只读）** 及其 §3.2 事务契约——模型语义是**连续字节流**：写按 `mem[addr+off+k]` 逐字节地址写入（off 4 步进），keep 标记有效字节（满字 1111 / 尾字 0001/0011/0111），last 只在尾字；`wr_req_addr` 是首字节字节地址，**任意合法起点无需字对齐移位**；读返回同理，`keep[k]` 对应 `data[8k+:8]`（低位先）。

| 交付 | 验收结果 |
|---|---|
| `raster_dma.v`（行光栅 DMA，按行调度 DDR 读/写，字节流进出 + keep 解包/打包） | 单元 tb_dma 31/31 ALL PASSED |
| `byte_packer.v`（字节流 ↔ 32 位字流原语，打包/解包双向） | 单元 tb_packer ALL PASSED |
| `ddr_port_adapter.v`（§3.2 抽象事务 ↔ 模型端口薄封装，在途阻塞/零长伪返回） | 单元 tb_adapter ALL PASSED |
| **集成拷贝** `tb_ddr_copy.sv`（u_pre 预载 → u_rd 读 A 区 → u_wr 写 B 区 → 模型内存逐字节比对） | 4 用例（对齐/非对齐/尾字节/双路背压/读错误注入）err=0 proto_violations=0 |
| `gray_fetch.v`（层灰度 DDR → 片上 gray RAM，内部例化 raster_dma 读方向 + 写捕获） | 单元 tb_grayfetch 4 用例 ALL PASSED（err=0 proto=0） |
| **灰度经 DDR 全链** `tb_detect_ddr.sv`（DDR 预载 → gray_fetch L0 → pyramid 生成 L1 → detect_ctrl） | big 金字塔路径 40/40 + board5x8 native 路径 40/40，err=0 proto_violations=0 |
| `detect_ctrl.sv`（新增 resp_tap 探针端口，向后兼容） | 全链对拍通过，旧 TB 不受影响 |

全链场景（`tb_detect_ddr.sv`，仿真 123.5ms / 真实约 13 分钟）：
- **big（金字塔路径，1280×720）**：DDR @0x1000 预载 `m6_big_gray.bin`（stride=1280）→ `gray_fetch` 加载 L0 到片上 RAM（921600 字节 err=0）→ `pyramid_ctrl` 生成 L1 640×360（count=2）→ `detect_ctrl` DEPTH=2（native@L1 → 2p+0.5 → refine@L0）→ 40 点与 `m6_chain_big.bin` 逐位一致。
- **board5x8（native 路径，272×96）**：DDR @0x200000 预载 `m5_board5x8_gray.bin` → `gray_fetch` → `detect_ctrl` DEPTH=1 → 40 点与 `m6_chain_board5x8.bin` 逐位一致。

**不在 M7 范围**：响应图写 DDR（resp_tap → 字宽写路径 + 帧级任务流）留 **M7.3**；标定、remap 仍为后续里程碑。M7.3 的输入接口已就绪（detect_ctrl 新增 `resp_tap_valid/resp_tap_data`：Pass1 native 期间当前层 resp 流直出）。

---

## 2. 架构总览

```
DDR（ddr_memory_model，字节流语义）
  │
  └─ ddr_port_adapter（§3.2 抽象事务 ↔ 模型端口；未来真实控制器替换点）
       └─ raster_dma（行光栅 DMA：cfg_base/stride/row_bytes/rows/offset/dir，逐行一笔事务 tag=y）
            ├─ 读方向（cfg_dir=0）：DDR → 字节流（keep 解包，光栅序）
            └─ 写方向（cfg_dir=1）：字节流 → DDR（keep 打包）
                 └─ byte_packer（字节流 ↔ 32 位字流原语，打包/解包共用）
       └─ gray_fetch（内部例化 raster_dma 读方向 + gray RAM 写捕获：
            cfg_ddr_base/cfg_stride/cfg_w/cfg_h/cfg_ram_base；gray_wr_en/addr/data 同步写）
                 → 片上 gray RAM base0
                      → pyramid_ctrl（生成 L1.. 片上）
                           → detect_ctrl（DEPTH 层槽位链，resp_tap 探针）
                                → 40 点输出
```

灰度上链路径（M7.2）：**DDR 字节流（读） → gray_fetch → 片上 gray RAM（写）**，之后与 M6 完全一致（pyramid / detect 消费片上 RAM）。`gray_fetch` 的 `gray_wr_addr = cfg_ram_base + 像素号`（光栅序连续排布），`out_ready` 恒 1 无背压，同步写 1 拍完成。

---

## 3. 模块详解

### 3.1 `raster_dma.v`（rtl/mem/，M7.1 子代理 E）

把一片行光栅区域（`cfg_base` 起、每行 `cfg_stride` 字节跨度、每行 `cfg_row_bytes` 有效字节、共 `cfg_rows` 行、payload 起点 `cfg_offset`）作为连续字节流搬入/搬出 DDR。逐行 y=0..cfg_rows-1 发一笔 DDR 事务（addr = cfg_base + y*stride + offset，len=row_bytes，tag=y），一笔在途：读方向收齐 `rd_ret_last` 才发下行，写方向收齐该行 `wr_done` 才发下行。keep 规则：非尾拍 1111，尾拍按剩余字节；读按 `keep[k]` 对应 `data[8k+:8]` 低位先逐字节输出，写按"连续字节流"打包（字节 i 落 `data[8*(i%4)+:8]`），**与 (addr%4) 字对齐无关**——模型自行处理任意合法字节起点，无需本模块移位。错误语义：`rd_ret_error`/`wr_done_error` → status=10/11，排空在途事务后 done=1（错误也置 done）。start 单拍锁存配置并全状态复位（连续帧复用）。

### 3.2 `byte_packer.v`（rtl/mem/，M7.1 子代理 F）

通用字节流 ↔ 32 位字流转换原语：打包（cfg_dir=1，字节流 → 字流，供写 DDR）与解包（cfg_dir=0，字流 → 字节流，供读 DDR）。与模型契约对齐：字节 i 落 `data[8*(i%4)+:8]`（字节 0 恒在 data[7:0]），keep 位对应，非尾拍 1111、尾拍按剩余字节（0001/0011/0111/1111），整事务最后字 last=1。`cfg_addr` 仅记录首字节地址（不参与字节排列，真实控制器侧字对齐拆拍由上层完成）。len==0 → status=10 立即完成、不产生任何字流。解包"只输出 cfg_len 个字节"，多余 keep 位忽略；字流短于 cfg_len 则停在等字状态（上层协议错误，不假完成、不死锁）。

### 3.3 `ddr_port_adapter.v`（rtl/mem/，M7.1 子代理 F）

§3.2 抽象事务 ↔ 模型端口的薄封装（一对一映射，模型端口名 `rd_req_len_bytes`/`wr_cplt_*` 差异由本模块吸收）。**它是未来真实 DDR 服务层控制器（强文韬侧）的替换对接点**：届时仅将 m_* 端口改接到真实控制器，客户端侧接口不变。在途语义（契约第 3 条：每客户端一笔读、一笔写在途，读写并行）：客户端在途期间再次请求 → 拉低 ready 阻塞（不吞、零协议违规）；在途期间送写数据 → valid 门控屏蔽、ready 拉低，模型看不到"无在途写请求的写数据"。tag 在请求握手时锁存回带。len==0：客户端侧正常握手但不发模型，直接回一拍 error=1 伪返回/伪完成（模型零协议违规，上层可感知自身违规）。

### 3.4 `gray_fetch.v`（rtl/mem/，M7.2 子代理 G）

层灰度 DDR → 片上 gray RAM。内部例化 raster_dma（cfg_dir=0 读方向，cfg_base/cfg_stride/cfg_row_bytes=cfg_w/rows=cfg_h/offset=0），`start` 单拍转发（busy=0 时）；DDR 读通道经本模块端口直连模型。写捕获：`out_ready` 恒 1，`gray_wr_en` 组合跟随 out 接受拍，游标 `cnt=y*cfg_w+x`，`gray_wr_addr=cfg_ram_base+cnt`（光栅序连续排布），同步写 1 拍完成。完成沿：检测内部 DMA `done` 0→1 沿，后一拍锁存 status（01 成功 / 10 读错误）→ done 电平、busy 清零；start 全状态复位支持连续帧复用。

### 3.5 `detect_ctrl.sv` 新增 resp_tap 探针（M7.2 主控）

新增 `resp_tap_valid`/`resp_tap_data` 输出端口：Pass1 native 期间当前层 resp 流直出（供响应图写 DDR 等后续使用）。向后兼容——旧 TB 未连接该端口不受影响（全链对拍即为回归证据）。

---

## 4. 验证矩阵

| TB | 内容 | 结果 |
|---|---|---|
| `tb_dma.sv` | raster_dma 单元（读/写、keep 解包/打包、行调度、尾字节、错误注入） | 31/31 ALL PASSED |
| `tb_packer.sv` | byte_packer 单元（打包/解包、尾字节、零长违规） | ALL PASSED |
| `tb_adapter.sv` | ddr_port_adapter 单元（在途阻塞、零长伪返回、错误透传） | ALL PASSED |
| `tb_ddr_copy.sv` | **集成拷贝闭环**：u_pre 预载 → u_rd 读 A → u_wr 写 B → 模型内存逐字节比对（对齐/非对齐/尾字节/双路背压/读错误注入） | 4 用例 err=0 proto_violations=0 |
| `tb_grayfetch.sv` | gray_fetch 单元（big-L0 921600B / big-L1 230400B / 非对齐 127×63 stride=131 ram_base 非零 / 读错误注入） | 4 用例 err=0 proto_violations=0 |
| `tb_detect_ddr.sv` | **灰度经 DDR 全链集成**（DDR 预载 → gray_fetch → pyramid → detect；big 金字塔路径 + board5x8 native 路径） | 40/40 × 2，err=0 proto_violations=0 |

模型配置：LATENCY_MIN=1..6、JITTER/BACKPRESSURE/PROTOCOL_CHECKS 全开（随机延迟 + 随机背压下全过）。

复现：ModelSim vlog + vsim（单实例，授权码并发受限——**多个 vsim 并行会 license 争用失败**，务必串行：启动前 `Get-Process vsimk` 确认无实例）。编译链：detect 层槽位链（M6 集）+ pyramid_ctrl/downsample2x + raster_dma/gray_fetch + ddr_memory_model + TB。

---

## 5. 过程中发现并修复的问题

1. **连续字节语义契约（M7.1 确立）**：模型按"字节地址连续"读写（`mem[addr+off+k]`），首字节恒在 data[7:0]，`keep[k]↔data[8k+:8]` 低位先——非对齐起点（cfg_offset 任意）**无需字对齐移位**。若按"字对齐"思维做移位会与模型契约错位（参考 M1 §3.2 行为权威）。
2. **gray_fetch 初版漏接内部 DMA 读通道输入（G 发现并修复）**：`rd_req_ready`/`rd_ret_valid/data/keep/tag/last/error` 未从模块端口透传给内部 raster_dma，悬空 z → `1 && z = x` 请求握手永不成立、DMA 卡死 RD_IDLE、模型 RD_STREAM 死锁。补 7 条 `assign` 透传后 PASS。
3. **集成 TB 握手竞态教训（M7.1 主控）**：请求握手用"组合 valid=pending&&ready + ready 回落判接受"零竞态；数据拍**非尾**用模型 `wr_off_r` 前进判接受、**尾拍**用 `wr_dat_ready` 回落判接受——混用会导致尾字节重复或丢失。
4. **SV 任务入参值拷贝冻结（M7.1 集成 TB）**：任务 `input logic d` 为值拷贝，循环内 d 冻结为初值，永远等不到 done 沿 → 改用内联"前一负沿值判沿"循环（与 M6 集成 TB 同款教训，M7 再次确认此纪律）。
5. **模型关联数组警告洪水（非违规）**：ddr_memory_model 的 `rd_addr_r`/`rd_len_r` 在首次读请求前为 X，稀疏数组 X 索引查询触发 ~92 万条 `Non-existent associative array entry` 警告；参考 TB tb_ddr_copy 同样洪水，proto_violations=0，模型为只读未改动。
6. **响应写 DDR 未做（留 M7.3）**：resp 流写 DDR 需要**字宽写路径**（response_store_max 现为内部 RAM，其输出是窄位宽响应结构而非字节流）——按事务契约写 DDR 需先打包成字，本里程碑不做，detect_ctrl 已留 resp_tap 探针作为 M7.3 输入。

**项目级教训（license）**：ModelSim 授权码并发实例数有限，所有 vsim 必须串行（检查无 vsimk 进程再启动）。本阶段 3 个子代理 + 主控全程遵守。

---

## 6. 遗留事项与接口契约（交接重点）

- **M7.3 响应写 DDR（未做）**：resp_tap 探针流（Pass1 native 期间当前层 resp 流直出）→ 字宽写路径 → DDR。需评估：响应结构窄位宽打包成字、帧级任务流（detect 完成后统一搬移）、与 gray_fetch 的 DDR 读通道仲裁。
- **字节语义契约（冻结）**：模型读写均按连续字节地址；keep[k] 对应 data[8k+:8]；非尾 keep=1111、尾拍按剩余字节（0001/0011/0111/1111）、last 只在尾字；wr_req_addr/rd_req_addr = 首字节字节地址（任意起点，无字对齐要求）。**任何 DDR 侧新模块必须遵守，否则触发模型协议违规（PROTOCOL_CHECKS）并得到 error=1 事务。**
- **ddr_port_adapter 是未来真实控制器替换点**：真实 DDR 服务层控制器（强文韬侧）就绪后，仅需将 m_* 端口改接，客户端侧接口不变。
- **行调度纪律**：raster_dma/gray_fetch 一笔在途（读收 last / 写收 wr_done 才发下一行），tag=y 行号回带。
- **gray_fetch 与 detect 的灰度所有权**：gray_fetch 只写 L0（cfg_ram_base 可配置）；detect_ctrl 只消费不生成；pyramid_ctrl 生成 L1.. 并落位 base_d = base0 + Σ W_k*H_k（M6 冻结契约不变）。
- **集成 TB 握手纪律**：请求"组合 valid + ready 回落判接受"；数据非尾"wr_off_r 前进判接受"、尾拍"ready 回落判接受"；等待电平沿用"前一负沿值判沿"循环（防值拷贝冻结/防漏沿）。
