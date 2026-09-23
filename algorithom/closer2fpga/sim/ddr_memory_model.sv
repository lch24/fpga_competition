`timescale 1ns / 1ps
//==============================================================================
// ddr_memory_model.sv — DDR 服务层行为模型（仿真用，不可综合）
//------------------------------------------------------------------------------
// 依据团队 README 第4节《DDR 服务：由强文韬统一提供》的接口契约实现，
// 供苏晨（角点检测）在强文韬的 RTL 服务层就绪前独立开发与仿真。
// 本模型模拟"服务层对算法侧暴露的逻辑读写口"，不是 DDR PHY/控制器模型。
//
// 契约要点（README 第4节）：
//   1) 算法侧使用 32 位数据宽度；地址/长度单位为字节。
//   2) 支持任意合法字节起点；首个返回/写入字节放在 data[7:0]，
//      随后按低位到高位排列；尾拍用 keep 标记有效字节。
//   3) 每个客户端首版最多一笔读、一笔写在途（读、写通道相互独立可并行）。
//   4) 请求长度必须大于 0。
//   5) tag 在客户端内匹配请求。
//   6) 写请求握手后才发写数据；首版不交织多笔写数据。
//
// 模型附加能力（用于暴露算法侧 RTL 的健壮性问题）：
//   - 随机延迟（JITTER_EN）：每笔读返回/写完成的首拍延迟在
//     [LATENCY_MIN, LATENCY_MAX] 内伪随机（LFSR，SEED 固定则完全可复现）。
//   - 随机背压（BACKPRESSURE_EN）：服务端 ready（读请求/写请求/写数据）
//     按伪随机模式拉低。读返回/写完成的 ready 由客户端（TB/上层）驱动。
//   - 错误注入：TB 可层次调用 inject_error()/clear_error_injection()，
//     命中地址范围的读返回 error=1、写完成 error=1。
//   - 协议检查（PROTOCOL_CHECKS）：统计违规到 proto_violations，
//     包括 len==0、写数据 last 位置/keep 与长度不符、非尾拍 keep 有洞、
//     无写请求在途时出现写数据。模型永不因违规挂死（吞掉多余写数据拍）。
//     写数据类违规同时置粘滞标志，随本事务完成通道返回 error=1，
//     让客户端能感知自身协议错误。
//
// 时钟/复位：clk 单一时钟；rst_n 低有效（应已同步释放，如来自 reset_sync）。
//   复位清空状态机与协议计数；内存内容（关联数组）跨复位保留——
//   真实 DDR 同样如此，算法侧不得依赖"复位后内存为特定值"。
//
// 存储语义：稀疏字节关联数组，未写入地址读出 0x00。
//
// 注：本模型读写两通道允许并行（契约允许一笔读+一笔写在途）。
//   真实服务层内部还会做仲裁与 256bit 拆装，本模型不模拟这些内部细节。
//==============================================================================
module ddr_memory_model #(
    parameter int unsigned LATENCY_MIN     = 1,
    parameter int unsigned LATENCY_MAX     = 6,
    parameter bit          JITTER_EN       = 1'b1,
    parameter bit          BACKPRESSURE_EN = 1'b1,
    parameter bit          PROTOCOL_CHECKS = 1'b1,
    parameter [31:0]       SEED            = 32'h1234_5679
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- 通道1：读请求（算法侧 → 服务层） ----
    input  logic        rd_req_valid,
    output logic        rd_req_ready,
    input  logic [31:0] rd_req_addr,
    input  logic [31:0] rd_req_len_bytes,
    input  logic [15:0] rd_req_tag,

    // ---- 通道2：读返回（服务层 → 算法侧） ----
    output logic        rd_ret_valid,
    input  logic        rd_ret_ready,
    output logic [31:0] rd_ret_data,
    output logic [3:0]  rd_ret_keep,
    output logic [15:0] rd_ret_tag,
    output logic        rd_ret_last,
    output logic        rd_ret_error,

    // ---- 通道3：写请求（算法侧 → 服务层） ----
    input  logic        wr_req_valid,
    output logic        wr_req_ready,
    input  logic [31:0] wr_req_addr,
    input  logic [31:0] wr_req_len_bytes,
    input  logic [15:0] wr_req_tag,

    // ---- 通道4：写数据（算法侧 → 服务层） ----
    input  logic        wr_dat_valid,
    output logic        wr_dat_ready,
    input  logic [31:0] wr_dat_data,
    input  logic [3:0]  wr_dat_keep,
    input  logic        wr_dat_last,

    // ---- 通道5：写完成（服务层 → 算法侧） ----
    output logic        wr_cplt_valid,
    input  logic        wr_cplt_ready,
    output logic [15:0] wr_cplt_tag,
    output logic        wr_cplt_error,

    // ---- 协议违规计数（TB 自检） ----
    output logic [15:0] proto_violations
);

    //--------------------------------------------------------------------
    // 稀疏字节存储（跨复位保留）
    //--------------------------------------------------------------------
    bit [7:0] mem [bit [31:0]];

    //--------------------------------------------------------------------
    // 可复现伪随机源（LFSR，避免全零状态）
    //--------------------------------------------------------------------
    logic [31:0] lfsr;
    always @(posedge clk) begin
        if (!rst_n)
            lfsr <= (SEED | 32'h1);
        else
            lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    end

    // 各通道独立的背压门控（约 1/8 概率拉低）
    wire gate_rd_req = BACKPRESSURE_EN ? (lfsr[2:0]  != 3'b000) : 1'b1;
    wire gate_wr_req = BACKPRESSURE_EN ? (lfsr[5:3]  != 3'b000) : 1'b1;
    wire gate_wr_dat = BACKPRESSURE_EN ? (lfsr[8:6]  != 3'b000) : 1'b1;

    function automatic logic [31:0] rand_latency();
        logic [31:0] span;
        span = LATENCY_MAX - LATENCY_MIN + 1;
        if (!JITTER_EN || LATENCY_MAX <= LATENCY_MIN)
            return LATENCY_MIN;
        return LATENCY_MIN + (lfsr % span);
    endfunction

    //--------------------------------------------------------------------
    // 错误注入（TB 层次调用）；复位时清零，避免 X 值抑制正常写通路
    //--------------------------------------------------------------------
    logic        inj_en;
    logic [31:0] inj_addr_min, inj_addr_max;
    logic [15:0] viol;

    always @(posedge clk) begin
        if (!rst_n) begin
            inj_en       <= 1'b0;
            inj_addr_min <= 32'h0;
            inj_addr_max <= 32'h0;
        end
    end

    task automatic inject_error(input logic [31:0] amin, input logic [31:0] amax);
        inj_addr_min = amin;
        inj_addr_max = amax;
        inj_en       = 1'b1;
    endtask

    task automatic clear_error_injection();
        inj_en = 1'b0;
    endtask

    // 阻塞赋值：允许同一拍内多处违规都计数（NBA 多进程同拍会丢失计数）
    task automatic report_violation(input string msg);
        viol = viol + 1'b1;
        $display("[DDR-MODEL][VIOLATION] @%0t : %s", $time, msg);
    endtask

    //--------------------------------------------------------------------
    // 读通道
    //--------------------------------------------------------------------
    typedef enum logic [1:0] { RD_IDLE, RD_WAIT, RD_STREAM } rd_state_e;
    rd_state_e  rd_state;
    logic [31:0] rd_addr_r, rd_len_r, rd_off_r, rd_lat_r;
    logic [15:0] rd_tag_r;
    logic        rd_err_r;

    assign rd_req_ready = rst_n && (rd_state == RD_IDLE) && gate_rd_req;

    // 当前读拍字节范围（组合）
    logic [31:0] rd_remain;
    logic [31:0] rd_beat_addr;
    wire [7:0]   rb0, rb1, rb2, rb3;

    assign rd_remain   = rd_len_r - rd_off_r;
    assign rd_beat_addr = rd_addr_r + rd_off_r;
    assign rb0 = mem[rd_beat_addr + 32'd0];
    assign rb1 = mem[rd_beat_addr + 32'd1];
    assign rb2 = mem[rd_beat_addr + 32'd2];
    assign rb3 = mem[rd_beat_addr + 32'd3];

    assign rd_ret_valid = (rd_state == RD_STREAM);
    assign rd_ret_data  = {rb3, rb2, rb1, rb0};
    assign rd_ret_keep  = (rd_remain >= 4) ? 4'b1111 :
                          (rd_remain == 3) ? 4'b0111 :
                          (rd_remain == 2) ? 4'b0011 :
                          (rd_remain == 1) ? 4'b0001 : 4'b0000;
    assign rd_ret_last  = (rd_remain <= 4);
    assign rd_ret_tag   = rd_tag_r;
    assign rd_ret_error = rd_err_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            rd_off_r <= 32'd0;
            rd_lat_r <= 32'd0;
            rd_err_r <= 1'b0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (rd_req_valid && rd_req_ready) begin
                        if (rd_req_len_bytes == 0) begin
                            if (PROTOCOL_CHECKS)
                                report_violation("read request with len_bytes==0, dropped");
                            // 已握手但长度非法：丢弃该请求（不产生返回流），
                            // 客户端等待流会暴露其自身 bug。
                        end else begin
                            rd_addr_r <= rd_req_addr;
                            rd_len_r  <= rd_req_len_bytes;
                            rd_tag_r  <= rd_req_tag;
                            rd_off_r  <= 32'd0;
                            rd_err_r  <= inj_en && (rd_req_addr >= inj_addr_min) &&
                                         (rd_req_addr <= inj_addr_max);
                            rd_lat_r  <= rand_latency();
                            rd_state  <= RD_WAIT;
                        end
                    end
                end

                RD_WAIT: begin
                    if (rd_lat_r != 32'd0)
                        rd_lat_r <= rd_lat_r - 32'd1;
                    else
                        rd_state <= RD_STREAM;
                end

                RD_STREAM: begin
                    if (rd_ret_valid && rd_ret_ready) begin
                        if (rd_ret_last)
                            rd_state <= RD_IDLE;
                        else
                            rd_off_r <= rd_off_r + 32'd4;
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 写通道
    //--------------------------------------------------------------------
    typedef enum logic [1:0] { WR_IDLE, WR_DATA, WR_WAIT, WR_CPLT } wr_state_e;
    wr_state_e  wr_state;
    logic [31:0] wr_addr_r, wr_len_r, wr_off_r, wr_lat_r;
    logic [15:0] wr_tag_r;
    logic        wr_err_r;
    logic        wr_viol_r;   // 本事务内出现过写协议违规（粘滞，随完成上报）

    assign wr_req_ready = rst_n && (wr_state == WR_IDLE) && gate_wr_req;
    assign wr_dat_ready = rst_n && (wr_state == WR_DATA) && gate_wr_dat;

    assign wr_cplt_valid = (wr_state == WR_CPLT);
    assign wr_cplt_tag   = wr_tag_r;
    assign wr_cplt_error = wr_err_r || wr_viol_r;

    // 当前写拍的期望 keep（按长度计算）
    logic [3:0] wr_exp_keep;
    always @(*) begin
        if (wr_off_r + 4 >= wr_len_r) begin
            case (wr_len_r - wr_off_r)
                32'd1:    wr_exp_keep = 4'b0001;
                32'd2:    wr_exp_keep = 4'b0011;
                32'd3:    wr_exp_keep = 4'b0111;
                default:  wr_exp_keep = 4'b1111;   // 恰好 4 或越界保护
            endcase
        end else begin
            wr_exp_keep = 4'b1111;
        end
    end

    // 无在途写请求却出现写数据 → 违规
    always @(posedge clk) begin
        if (PROTOCOL_CHECKS && rst_n && wr_dat_valid && (wr_state != WR_DATA))
            report_violation("write data appeared without an accepted write request");
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_state  <= WR_IDLE;
            wr_off_r  <= 32'd0;
            wr_lat_r  <= 32'd0;
            wr_err_r  <= 1'b0;
            wr_viol_r <= 1'b0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (wr_req_valid && wr_req_ready) begin
                        wr_viol_r <= 1'b0;
                        if (wr_req_len_bytes == 0) begin
                            if (PROTOCOL_CHECKS)
                                report_violation("write request with len_bytes==0, dropped");
                        end else begin
                            wr_addr_r <= wr_req_addr;
                            wr_len_r  <= wr_req_len_bytes;
                            wr_tag_r  <= wr_req_tag;
                            wr_off_r  <= 32'd0;
                            wr_err_r  <= inj_en && (wr_req_addr >= inj_addr_min) &&
                                         (wr_req_addr <= inj_addr_max);
                            wr_state  <= WR_DATA;
                        end
                    end
                end

                WR_DATA: begin
                    if (wr_dat_valid && wr_dat_ready) begin
                        // keep 一致性检查
                        if (PROTOCOL_CHECKS && (wr_dat_keep != wr_exp_keep)) begin
                            wr_viol_r <= 1'b1;
                            report_violation("write data keep mismatch with len/offset");
                        end

                        // 字节写入（错误注入的写不生效，模拟地址不可达；越界部分忽略）
                        // 注：ModelSim 10.6e 不支持关联数组元素的非阻塞赋值，
                        //     此处用阻塞赋值（仿真模型语义：本拍写入立即可见）
                        for (int k = 0; k < 4; k++) begin
                            if (!wr_err_r && wr_dat_keep[k] && (wr_off_r + k < wr_len_r))
                                mem[wr_addr_r + wr_off_r + k] = wr_dat_data[8*k +: 8];
                        end

                        if (wr_dat_last) begin
                            // last 位置检查：尾拍要求 off+4>=len
                            if (PROTOCOL_CHECKS && (wr_off_r + 4 < wr_len_r)) begin
                                wr_viol_r <= 1'b1;
                                report_violation("write data LAST asserted too early");
                            end
                            wr_lat_r <= rand_latency();
                            wr_state <= WR_WAIT;
                        end else begin
                            // 非尾拍要求 off+4<len；否则是缺 last 的多余数据拍
                            if (PROTOCOL_CHECKS && (wr_off_r + 4 >= wr_len_r)) begin
                                wr_viol_r <= 1'b1;
                                report_violation("write data exceeds len without LAST");
                            end
                            wr_off_r <= wr_off_r + 32'd4;
                            // 多余数据已被上面的越界保护忽略，不死锁
                        end
                    end
                end

                WR_WAIT: begin
                    if (wr_lat_r != 32'd0)
                        wr_lat_r <= wr_lat_r - 32'd1;
                    else
                        wr_state <= WR_CPLT;
                end

                WR_CPLT: begin
                    if (wr_cplt_valid && wr_cplt_ready)
                        wr_state <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 协议违规计数输出（阻塞清零，与 report_violation 的阻塞累加一致）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            viol = 16'd0;
    end
    assign proto_violations = viol;

endmodule
