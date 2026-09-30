`timescale 1ns / 1ps
//==============================================================================
// resp_ddr_writer.v — 响应字流 → DDR 写（单事务，M7.3 基础件）
//------------------------------------------------------------------------------
// 语义（冻结契约）：
//   - start（busy=0 时单拍）锁存 cfg_base/cfg_words，随后发一笔写事务：
//     len_bytes = cfg_words*4，tag=0；写数据流 = 响应字流 in→wr_dat，
//     keep 恒 1111（cfg_words*4 恒为 4 的倍数，模型期望满字），
//     第 cfg_words-1 个字 last=1。
//   - 背压吸收：wr_dat_ready=0 时 in_ready 拉低，当前字不推进接受
//     （纯握手，上游保持 in_valid/in_data 即可，本模块无内部缓冲）。
//   - 收到写完成：error=0 → status=01；error=1 → status=10；
//     两者都 done=1、busy=0。done 电平保持，下次 start 清零。
//   - cfg_words==0：不发事务，直接 status=10 + done。
//
// 实现：内部例化 ddr_port_adapter 承担在途阻塞与 len==0 处理（读写通道
//   独立、同一时刻仅一笔写事务在途，由 start→done 串行保证）。
//   对外 wr_* 端口即模型侧直连端口（方向与 ddr_memory_model 一致）。
//==============================================================================
module resp_ddr_writer #(
    parameter ADDR_W = 32,
    parameter LEN_W  = 32,
    parameter TAG_W  = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 控制 ----
    input  wire                  start,          // busy=0 时单拍；锁存 cfg
    input  wire [ADDR_W-1:0]     cfg_base,       // DDR 字节首地址
    input  wire [LEN_W-1:0]      cfg_words,      // 字数（>0；==0 违规 status=10 立即 done）
    output reg                   busy,
    output reg                   done,           // 电平保持（下次 start 清零）
    output reg [1:0]             status,         // 01=成功 10=参数违规/写错误

    // ---- 响应字流 ----
    input  wire                  in_valid,
    output wire                  in_ready,       // 每拍收 1 字（in_valid&&in_ready）
    input  wire [31:0]           in_data,

    // ---- DDR 写通道（模型侧直连，方向同 ddr_memory_model）----
    output wire                  wr_req_valid,
    input  wire                  wr_req_ready,
    output wire [ADDR_W-1:0]     wr_req_addr,
    output wire [LEN_W-1:0]      wr_req_len,
    output wire [TAG_W-1:0]      wr_req_tag,
    output wire                  wr_dat_valid,
    input  wire                  wr_dat_ready,
    output wire [31:0]           wr_dat_data,
    output wire [3:0]            wr_dat_keep,
    output wire                  wr_dat_last,
    input  wire                  wr_done_valid,
    output wire                  wr_done_ready,
    input  wire [TAG_W-1:0]      wr_done_tag,
    input  wire                  wr_done_error
);

    localparam S_IDLE = 2'd0;
    localparam S_REQ  = 2'd1;
    localparam S_DATA = 2'd2;
    localparam S_DONE = 2'd3;

    reg [1:0]           state;
    reg [ADDR_W-1:0]    base_r;      // 锁存 cfg_base
    reg [LEN_W-1:0]     words_r;     // 锁存 cfg_words
    reg [LEN_W-1:0]     n_r;         // 已送出/已接受字数

    //--------------------------------------------------------------------
    // 内部 ddr_port_adapter 客户端侧信号
    //--------------------------------------------------------------------
    wire        c_wr_req_valid, c_wr_req_ready;
    wire [ADDR_W-1:0] c_wr_req_addr;
    wire [LEN_W-1:0]  c_wr_req_len;
    wire [TAG_W-1:0]  c_wr_req_tag;
    wire        c_wr_dat_valid, c_wr_dat_ready;
    wire [31:0] c_wr_dat_data;
    wire [3:0]  c_wr_dat_keep;
    wire        c_wr_dat_last;
    wire        c_wr_done_valid, c_wr_done_error;
    wire        c_wr_done_ready;

    // 接受拍（S_DATA 内 in_valid && in_ready；in_ready 已含背压条件）
    wire fire = in_valid && in_ready;

    // 请求：S_REQ 阶段持续拉 valid，等 adapter/模型就绪握手
    assign c_wr_req_valid = (state == S_REQ);
    assign c_wr_req_addr  = base_r;
    assign c_wr_req_len   = words_r << 2;          // 字数*4 = 字节数
    assign c_wr_req_tag   = {TAG_W{1'b0}};

    // 写数据：仅接受拍输出有效字（握手同步直通，无内部缓冲）
    assign c_wr_dat_valid = fire;
    assign c_wr_dat_data  = in_data;
    assign c_wr_dat_keep  = 4'b1111;               // cfg_words*4 恒 4 的倍数 → 全满字
    assign c_wr_dat_last  = (n_r == (words_r - {{(LEN_W-1){1'b0}}, 1'b1})) && fire;

    // 完成：S_DONE 接受（采样 error），完成后回 IDLE
    assign c_wr_done_ready = (state == S_DONE);

    // 响应字流接受条件：S_DATA 且模型写数据侧就绪且未送完
    assign in_ready = (state == S_DATA) && c_wr_dat_ready && (n_r < words_r);

    ddr_port_adapter #(
        .ADDR_W (ADDR_W),
        .LEN_W  (LEN_W),
        .TAG_W  (TAG_W)
    ) u_adapter (
        .clk (clk), .rst_n (rst_n),

        // 读通道：本模块不用，恒空闲
        .rd_req_valid  (1'b0),
        .rd_req_ready  (),
        .rd_req_addr   ({ADDR_W{1'b0}}),
        .rd_req_len    ({LEN_W{1'b0}}),
        .rd_req_tag    ({TAG_W{1'b0}}),
        .rd_ret_valid  (),
        .rd_ret_ready  (1'b0),
        .rd_ret_data   (),
        .rd_ret_keep   (),
        .rd_ret_tag    (),
        .rd_ret_last   (),
        .rd_ret_error  (),

        // 写通道客户端侧（内部）
        .wr_req_valid  (c_wr_req_valid),
        .wr_req_ready  (c_wr_req_ready),
        .wr_req_addr   (c_wr_req_addr),
        .wr_req_len    (c_wr_req_len),
        .wr_req_tag    (c_wr_req_tag),
        .wr_dat_valid  (c_wr_dat_valid),
        .wr_dat_ready  (c_wr_dat_ready),
        .wr_dat_data   (c_wr_dat_data),
        .wr_dat_keep   (c_wr_dat_keep),
        .wr_dat_last   (c_wr_dat_last),
        .wr_done_valid (c_wr_done_valid),
        .wr_done_ready (c_wr_done_ready),
        .wr_done_tag   (),
        .wr_done_error (c_wr_done_error),

        // 模型侧（对外）
        .m_rd_req_valid (),
        .m_rd_req_ready (1'b0),
        .m_rd_req_addr  (),
        .m_rd_req_len_bytes (),
        .m_rd_req_tag   (),
        .m_rd_ret_valid (1'b0),
        .m_rd_ret_ready (),
        .m_rd_ret_data  (32'd0),
        .m_rd_ret_keep  (4'd0),
        .m_rd_ret_tag   (16'd0),
        .m_rd_ret_last  (1'b0),
        .m_rd_ret_error (1'b0),

        .m_wr_req_valid (wr_req_valid),
        .m_wr_req_ready (wr_req_ready),
        .m_wr_req_addr  (wr_req_addr),
        .m_wr_req_len_bytes (wr_req_len),
        .m_wr_req_tag   (wr_req_tag),
        .m_wr_dat_valid (wr_dat_valid),
        .m_wr_dat_ready (wr_dat_ready),
        .m_wr_dat_data  (wr_dat_data),
        .m_wr_dat_keep  (wr_dat_keep),
        .m_wr_dat_last  (wr_dat_last),
        .m_wr_cplt_valid (wr_done_valid),
        .m_wr_cplt_ready (wr_done_ready),
        .m_wr_cplt_tag   (wr_done_tag),
        .m_wr_cplt_error (wr_done_error)
    );

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            status  <= 2'b00;
            base_r  <= {ADDR_W{1'b0}};
            words_r <= {LEN_W{1'b0}};
            n_r     <= {LEN_W{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        done <= 1'b0;
                        if (cfg_words == {LEN_W{1'b0}}) begin
                            // 参数违规：不发事务，立即结束
                            busy   <= 1'b0;
                            done   <= 1'b1;
                            status <= 2'b10;
                        end else begin
                            busy    <= 1'b1;
                            status  <= 2'b00;
                            base_r  <= cfg_base;
                            words_r <= cfg_words;
                            n_r     <= {LEN_W{1'b0}};
                            state   <= S_REQ;
                        end
                    end
                end

                S_REQ: begin
                    if (c_wr_req_valid && c_wr_req_ready)
                        state <= S_DATA;        // 请求握手成功，进写数据
                end

                S_DATA: begin
                    if (fire) begin
                        if (n_r == (words_r - {{(LEN_W-1){1'b0}}, 1'b1}))
                            state <= S_DONE;    // 尾字已送，等写完成
                        else
                            n_r <= n_r + {{(LEN_W-1){1'b0}}, 1'b1};
                    end
                end

                S_DONE: begin
                    if (c_wr_done_valid) begin
                        status <= c_wr_done_error ? 2'b10 : 2'b01;
                        done   <= 1'b1;
                        busy   <= 1'b0;
                        state  <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
