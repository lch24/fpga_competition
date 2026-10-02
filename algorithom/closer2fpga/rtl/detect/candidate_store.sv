`timescale 1ns / 1ps
//==============================================================================
// candidate_store.sv — M3 候选点 A/B 双缓冲存储
//------------------------------------------------------------------------------
// 语义：两个 64bit/地址 的同步读双口 RAM（A/B），每地址 {y[31:0], x[31:0]}
//   （fp32 位模式）。phase 选择写/读对象：
//     phase=0 : 写 A、读 B
//     phase=1 : 写 B、读 A
//   供主控 MERGE 阶段 A→B、B→A 轮换：merge 读当前满的一侧，结果流式
//   写进另一侧，一轮完成后翻转 phase 即可。
//
// 流式写口：wr_valid/wr_ready（wr_ready = 当前写侧未满 && !clr），内部自增
//   写指针；A/B 各自独立写指针与 count（phase 切换不串扰）。
//   clr 清零当前 phase 写侧的写指针/count（clr 优先，同拍不写）。
// 点随机读口：rd_en/rd_addr（对侧 buffer），1 拍延迟出整个点 {rd_x,rd_y}
//   （dual_port_ram 语义：rd_en=0 时输出保持上一次的值）。
//==============================================================================
module candidate_store #(
    parameter N_ADDR_W = 14                       // 深度 = 2**N_ADDR_W = 16384
) (
    input  wire                    clk,
    input  wire                    rst_n,
    // 控制
    input  wire                    clr,          // 清当前写侧（写指针/count）
    input  wire                    phase,        // 0:写A读B  1:写B读A
    // 流式写口
    input  wire                    wr_valid,
    output wire                    wr_ready,
    input  wire [31:0]             wr_x,
    input  wire [31:0]             wr_y,
    output wire [15:0]             count,        // 当前写侧已存点数
    // 点随机读口（对侧 buffer，1 拍延迟出整个点）
    input  wire                    rd_en,
    input  wire [N_ADDR_W-1:0]     rd_addr,
    output wire [31:0]             rd_x,
    output wire [31:0]             rd_y
);

    localparam DEPTH = (1 << N_ADDR_W);

    reg [N_ADDR_W-1:0] wr_ptr_a, wr_ptr_b;
    reg [15:0]         cnt_a, cnt_b;

    wire wr_fire = wr_valid && wr_ready;

    // 当前写侧指针/count（phase 选择）
    wire [N_ADDR_W-1:0] wr_ptr_sel = phase ? wr_ptr_b : wr_ptr_a;
    wire [15:0]         cnt_sel    = phase ? cnt_b    : cnt_a;

    assign wr_ready = !clr && (cnt_sel < DEPTH[15:0]);

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr_a <= {N_ADDR_W{1'b0}};
            wr_ptr_b <= {N_ADDR_W{1'b0}};
            cnt_a    <= 16'd0;
            cnt_b    <= 16'd0;
        end else if (clr) begin
            // 一次清两侧写指针/count：phase 可能随后切换，确保新写侧从 0 起
            wr_ptr_a <= {N_ADDR_W{1'b0}};
            wr_ptr_b <= {N_ADDR_W{1'b0}};
            cnt_a    <= 16'd0;
            cnt_b    <= 16'd0;
        end else if (wr_fire) begin
            if (phase) begin
                wr_ptr_b <= wr_ptr_b + 1'b1;
                cnt_b    <= cnt_b + 16'd1;
            end else begin
                wr_ptr_a <= wr_ptr_a + 1'b1;
                cnt_a    <= cnt_a + 16'd1;
            end
        end
    end

    assign count = cnt_sel;

    //--------------------------------------------------------------------
    // A/B 两个存储体（64bit/地址：{y,x}）
    //--------------------------------------------------------------------
    wire [63:0] wr_data = {wr_y, wr_x};
    wire        wa_en   = wr_fire && ~phase;      // 写 A
    wire        wb_en   = wr_fire &&  phase;      // 写 B
    wire        ra_en   = rd_en   &&  phase;      // 读 A（phase=1 时）
    wire        rb_en   = rd_en   && ~phase;      // 读 B（phase=0 时）

    wire [63:0] rd_a, rd_b;

    dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(N_ADDR_W)) u_ram_a (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (wa_en),
        .wr_addr  (wr_ptr_a),
        .wr_data  (wr_data),
        .rd_en    (ra_en),
        .rd_addr  (rd_addr),
        .rd_data  (rd_a)
    );

    dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(N_ADDR_W)) u_ram_b (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (wb_en),
        .wr_addr  (wr_ptr_b),
        .wr_data  (wr_data),
        .rd_en    (rb_en),
        .rd_addr  (rd_addr),
        .rd_data  (rd_b)
    );

    assign rd_x = phase ? rd_a[31:0]  : rd_b[31:0];
    assign rd_y = phase ? rd_a[63:32] : rd_b[63:32];

endmodule
