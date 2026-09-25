`timescale 1ns / 1ps
//==============================================================================
// nearest_spacing.sv — M3 最近邻间距 + 半径核（逐位复刻 candidates.cpp）
//------------------------------------------------------------------------------
// 语义（与 C++ 权威逐位一致，不得改动）：
//   nearest_distance(points, i)：result=FLT_MAX(32'h7f7fffff)；
//     j=0..N-1 且 j!=i：result = std::min(result, distance(points[i],points[j]))
//   distance = fp32_hypot(fp32_sub(ax,bx), fp32_sub(ay,by))（double 路径，
//     位级 = std::hypot(float,float)）。
//   radius = std::clamp(spacing*0.22f, 4.0f, 18.0f)
//           = min(max(mul(spacing,0x3e6147ae),0x40800000),0x41900000)。
//   输出每点 {spacing, radius} 位模式（i 增序）。
//
// 流水组织（点读口 registered，地址提前一拍，同 candidate_merge）：
//   锚点 i 读 3 拍（REQ→LATCH→FEED）；内循环 j 逐个读并喂 sub→hypot；
//   hypot 结果按 j 增序返回（in_flight 在途计数保序），min 链逐点
//   组合比较（std::min 语义 = (b<a)?b:a，全部非负 → 无符号位比较）。
//   半径 clamp 在 spacing 确定后 1 条 fp32_mul + 组合比较完成。
//==============================================================================
module nearest_spacing #(
    parameter N_ADDR_W = 14
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    input  wire [15:0]             n_in,           // 点数（start 时锁存）
    // 点读口（1 拍延迟，连到候选点 RAM）
    output reg                     rd_en,
    output reg  [N_ADDR_W-1:0]     rd_addr,
    input  wire [31:0]             rd_x,
    input  wire [31:0]             rd_y,
    // 结果输出（流式握手，{spacing,radius} 整点输出）
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_spacing,
    output reg  [31:0]             out_radius
);

    //--------------------------------------------------------------------
    // 常量（fp32 位模式）
    //--------------------------------------------------------------------
    localparam FLT_MAX   = 32'h7f7fffff;
    localparam C_022F    = 32'h3e6147ae;   // 0.22f
    localparam C_4F      = 32'h40800000;   // 4.0f
    localparam C_18F     = 32'h41900000;   // 18.0f

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE    = 4'd0;
    localparam S_AREQ    = 4'd1;   // 发读请求 points[i]
    localparam S_ALAT    = 4'd2;   // 等数据
    localparam S_AFEED   = 4'd3;   // 锁存锚点，初始化 min 链
    localparam S_IREQ    = 4'd4;   // 发读请求 points[j]
    localparam S_ILAT    = 4'd5;   // 等数据
    localparam S_IFEED   = 4'd6;   // 数据有效：喂 sub（j==i 跳过）
    localparam S_DRAIN   = 4'd7;   // 等所有在途 distance 清空
    localparam S_RAD     = 4'd8;   // 喂 mul(spacing, 0.22f)
    localparam S_RADW    = 4'd9;   // 等 mul 结果 → clamp
    localparam S_OUT     = 4'd10;  // out_valid 输出整点
    localparam S_DONE    = 4'd11;
    reg [3:0] state;

    reg [15:0]     n_reg;
    reg [15:0]     i;             // 锚点序号
    reg [15:0]     j_cur;         // 内循环下一个扫描点
    reg [15:0]     in_flight;     // 在途 distance 数（保序）
    reg [31:0]     anchor_x, anchor_y;
    reg [31:0]     min_reg;       // min 链结果（= spacing）
    reg [31:0]     mul_r;

    // 子模块互连
    wire subx_rdy, suby_rdy, subx_v, suby_v;
    wire [31:0] subx_r, suby_r;
    wire hypot_rdy, hypot_v;
    wire [31:0] hypot_r;
    wire mul_rdy, mul_v;
    wire [31:0] mul_out;
    wire mul_fire_w = (state == S_RAD) && mul_rdy;
    wire feed_fire;

    assign feed_fire = (state == S_IFEED) && (j_cur != i) &&
                       subx_rdy && suby_rdy && hypot_rdy;

    //--------------------------------------------------------------------
    // 子模块例化
    //--------------------------------------------------------------------
    fp32_sub u_subx (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (feed_fire),
        .in_ready (subx_rdy),
        .in_a     (anchor_x),
        .in_b     (rd_x),
        .out_valid(subx_v),
        .out_ready(hypot_rdy),
        .out_r    (subx_r)
    );
    fp32_sub u_suby (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (feed_fire),
        .in_ready (suby_rdy),
        .in_a     (anchor_y),
        .in_b     (rd_y),
        .out_valid(suby_v),
        .out_ready(hypot_rdy),
        .out_r    (suby_r)
    );
    fp32_hypot u_hypot (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (subx_v && suby_v),
        .in_ready (hypot_rdy),
        .in_a     (subx_r),
        .in_b     (suby_r),
        .out_valid(hypot_v),
        .out_ready(1'b1),          // min 链恒收
        .out_r    (hypot_r)
    );
    fp32_mul u_mul (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mul_fire_w),
        .in_ready (mul_rdy),
        .in_a     (min_reg),
        .in_b     (C_022F),
        .out_valid(mul_v),
        .out_ready(1'b1),
        .out_r    (mul_out)
    );

    //--------------------------------------------------------------------
    // 主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            i            <= 16'd0;
            j_cur        <= 16'd0;
            rd_en        <= 1'b0;
            rd_addr      <= {N_ADDR_W{1'b0}};
            anchor_x     <= 32'd0;
            anchor_y     <= 32'd0;
            min_reg      <= FLT_MAX;
            out_valid    <= 1'b0;
            out_spacing  <= 32'd0;
            out_radius   <= 32'd0;
            mul_r        <= 32'd0;
            n_reg        <= 16'd0;
        end else begin
            rd_en <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy    <= 1'b1;
                        done     <= 1'b0;
                        i        <= 16'd0;
                        j_cur    <= 16'd0;
                        out_valid<= 1'b0;
                        n_reg    <= n_in;
                        state    <= S_AREQ;
                    end
                end
                S_AREQ: begin
                    if (i >= n_reg) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= S_DONE;
                    end else begin
                        rd_en  <= 1'b1;
                        rd_addr <= i[N_ADDR_W-1:0];
                        state  <= S_ALAT;
                    end
                end
                S_ALAT: begin
                    state <= S_AFEED;
                end
                S_AFEED: begin
                    anchor_x <= rd_x;
                    anchor_y <= rd_y;
                    min_reg  <= FLT_MAX;
                    j_cur    <= 16'd0;
                    in_flight<= 16'd0;
                    state    <= S_IREQ;
                end
                S_IREQ: begin
                    if (j_cur >= n_reg) begin
                        state <= S_DRAIN;          // 内循环为空（单点锚）
                    end else begin
                        rd_en   <= 1'b1;
                        rd_addr <= j_cur[N_ADDR_W-1:0];
                        state   <= S_ILAT;
                    end
                end
                S_ILAT: begin
                    state <= S_IFEED;
                end
                S_IFEED: begin
                    if (j_cur == i) begin
                        // 跳过自身：数据已读，不喂
                        j_cur <= j_cur + 16'd1;
                        state <= S_IREQ;
                    end else if (feed_fire) begin
                        j_cur <= j_cur + 16'd1;
                        state <= S_IREQ;
                    end
                end
                S_DRAIN: begin
                    if (in_flight == 16'd0)
                        state <= S_RAD;
                end
                S_RAD: begin
                    if (mul_rdy) state <= S_RADW;
                end
                S_RADW: begin
                    if (mul_v) begin
                        mul_r <= mul_out;
                        state <= S_OUT;
                    end
                end
                S_OUT: begin
                    out_valid <= 1'b1;
                    out_spacing <= min_reg;
                    out_radius  <= clamp_comb(mul_r);
                    if (out_valid && out_ready) begin
                        out_valid <= 1'b0;
                        if (i + 16'd1 >= n_reg) begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_DONE;
                        end else begin
                            i <= i + 16'd1;
                            state <= S_AREQ;
                        end
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // clamp：min(max(v,4.0f),18.0f)，全部非负 → 无符号位比较
    function automatic [31:0] clamp_comb(input [31:0] v);
        reg [31:0] c1;
        begin
            c1 = (v < C_4F) ? C_4F : v;
            clamp_comb = (c1 > C_18F) ? C_18F : c1;
        end
    endfunction

    //--------------------------------------------------------------------
    // 在途 distance 计数（feed +1 / hypot 结果 -1，min 链恒收）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            in_flight <= 16'd0;
        else if (state == S_AFEED)
            in_flight <= 16'd0;
        else if (feed_fire && hypot_v)
            in_flight <= in_flight;
        else if (feed_fire)
            in_flight <= in_flight + 16'd1;
        else if (hypot_v)
            in_flight <= in_flight - 16'd1;
    end

    //--------------------------------------------------------------------
    // min 链：std::min(result,d) = (d<result)?d:result（非负 → 无符号比较）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            min_reg <= FLT_MAX;
        else if (state == S_AFEED)
            min_reg <= FLT_MAX;
        else if (hypot_v)
            min_reg <= (hypot_r < min_reg) ? hypot_r : min_reg;
    end

endmodule
