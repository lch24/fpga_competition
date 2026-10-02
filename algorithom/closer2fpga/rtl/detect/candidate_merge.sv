`timescale 1ns / 1ps
//==============================================================================
// candidate_merge.sv — M3 候选去重核（逐位复刻 candidates.cpp::merge_duplicates）
//------------------------------------------------------------------------------
// 语义（与 C++ 权威逐位一致，不得改动）：
//   外循环 i=0..N-1：used[i] 则跳过；否则锚点=points[i]，sum=points[i]，n=1
//   内循环 j=i+1..N-1：!used[j] && dist(points[i],points[j]) < radius
//       → used[j]=1、sum.x+=points[j].x、sum.y+=points[j].y（按 j 增序
//         fp32_add 链式累加，与 C++ 舍入顺序一致）、n++
//   输出 {sum.x/n, sum.y/n}（fp32_div(sum, s32_to_f32(n))）；输出序 = i 增序。
//   dist = fp32_hypot(fp32_sub(ax,bx), fp32_sub(ay,by))（double 路径，
//         位级 = std::hypot(float,float)）。
//
// 流水组织（读口 registered，地址提前一拍）：
//   内循环扫描每点 3 拍：REQ(发 rd_addr) → LATCH(锁存点) → FEED(喂 sub，
//   坐标+序号入 coord FIFO)；distance 全流水，结果按 j 增序返回（在途计数
//   in_flight 保序）。命中才进 add 累加链（accx/accy_busy 时回压 dist 结果，
//   保证 sum 串行链与 C++ 逐次舍入位级一致）。
//   背压全程弹性：coord FIFO 满→sub 不喂；hypot 输出被回压→整链冻结。
//==============================================================================
module candidate_merge #(
    parameter N_ADDR_W = 14,
    parameter DEPTH    = (1 << N_ADDR_W)
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    input  wire [15:0]             n_in,           // 本次输入点数（start 时锁存）
    input  wire [31:0]             radius,         // fp32 位模式（start 时锁存）
    // 点读口（1 拍延迟，连到 candidate_store）
    output reg                     rd_en,
    output reg  [N_ADDR_W-1:0]     rd_addr,
    input  wire [31:0]             rd_x,
    input  wire [31:0]             rd_y,
    // 结果输出（流式握手，{res_x,res_y} 整点输出）
    output reg                     res_valid,
    input  wire                    res_ready,
    output reg  [31:0]             res_x,
    output reg  [31:0]             res_y,
    output reg  [14:0]             res_count       // 已输出点数
);

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE       = 4'd0;
    localparam S_ANCHOR_REQ = 4'd1;   // 发读请求 points[i]
    localparam S_ANCHOR_LAT = 4'd2;   // 等数据（本拍 RAM 采样地址）
    localparam S_ANCHOR_FEED= 4'd3;   // 数据有效：锁存锚点（used 已在此前跳过）
    localparam S_IN_REQ     = 4'd4;   // 发读请求 points[j_cur]
    localparam S_IN_LATCH   = 4'd5;   // 等数据（本拍 RAM 采样地址）
    localparam S_IN_FEED    = 4'd6;   // 数据有效：喂 sub + 坐标入队
    localparam S_IN_DRAIN   = 4'd7;   // 等所有在途 distance 与累加链清空
    localparam S_OUT_N      = 4'd8;   // 喂 s32_to_f32(n)
    localparam S_OUT_DIVX   = 4'd9;   // 喂 div(sum_x, n_f)
    localparam S_OUT_WX     = 4'd10;  // 等 div_x 结果
    localparam S_OUT_DIVY   = 4'd11;  // 喂 div(sum_y, n_f)
    localparam S_OUT_WY     = 4'd12;  // 等 div_y 结果
    localparam S_OUT_RDY    = 4'd13;  // res_valid 输出整点
    reg [3:0] state;

    reg [15:0]         n_reg;
    reg [31:0]         radius_reg;
    reg [15:0]         i;             // 外循环锚点序号
    reg [15:0]         j_cur;         // 内循环下一个扫描点
    reg [15:0]         in_flight;     // 在途 distance 数（保序）
    reg [31:0]         anchor_x, anchor_y;
    reg [31:0]         sum_x_cur, sum_y_cur;
    reg [15:0]         n_cnt;
    reg [DEPTH-1:0]    used_bit;      // used 位图（每轮 start 清零）
    reg                accx_busy, accy_busy;
    reg [31:0]         divx_out, divy_out;
    reg [31:0]         n_f;

    // 子模块互连与控制信号（先声明，避免隐式 net 与重复声明）
    wire subx_rdy, suby_rdy, subx_v, suby_v;
    wire [31:0] subx_r, suby_r;
    wire hypot_rdy, hypot_v;
    wire [31:0] hypot_r;
    wire cfifo_in_ready, cfifo_out_valid;
    wire [77:0] cfifo_out;
    wire addx_rdy, addy_rdy, addx_v, addy_v;
    wire [31:0] addx_r, addy_r;
    wire div_rdy, div_v;
    wire [31:0] div_r;
    wire nf_rdy, nf_valid;
    wire feed_fire, cfifo_pop, addx_fire, addy_fire, nf_fire, div_fire;
    wire dist_scanning, lt_dist, dist_hit, acc_block;
    wire [N_ADDR_W-1:0] cfifo_jj;
    wire [31:0] cfifo_px, cfifo_py, div_a;

    //--------------------------------------------------------------------
    // 子模块例化
    //--------------------------------------------------------------------

    // 距离流水：sub(x) / sub(y) → hypot（double 路径）
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
        .out_ready(cfifo_pop),
        .out_r    (hypot_r)
    );

    // 在途坐标 FIFO（{j, x, y}，与 hypot 输出保序对齐）
    sync_fifo #(.DATA_WIDTH(78), .ADDR_WIDTH(7)) u_cfifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (feed_fire),
        .in_ready (cfifo_in_ready),
        .in_data  ({j_cur[N_ADDR_W-1:0], rd_x, rd_y}),
        .out_valid(cfifo_out_valid),
        .out_ready(cfifo_pop),
        .out_data (cfifo_out),
        .count    (),
        .empty    (),
        .full     ()
    );

    // sum 累加链（x/y 各一条 fp32_add，串行保序 = C++ 逐次舍入）
    fp32_add u_addx (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (addx_fire),
        .in_ready (addx_rdy),
        .in_a     (sum_x_cur),
        .in_b     (cfifo_px),
        .out_valid(addx_v),
        .out_ready(1'b1),
        .out_r    (addx_r)
    );
    fp32_add u_addy (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (addy_fire),
        .in_ready (addy_rdy),
        .in_a     (sum_y_cur),
        .in_b     (cfifo_py),
        .out_valid(addy_v),
        .out_ready(1'b1),
        .out_r    (addy_r)
    );

    // n（int）→ fp32
    s32_to_f32 u_ncvt (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (nf_fire),
        .in_ready (nf_rdy),
        .in_data  ({16'd0, n_cnt}),
        .out_valid(nf_valid),
        .out_ready(1'b1),
        .out_r    (n_f)
    );

    // sum / n（分时复用 1 条 fp32_div）
    fp32_div u_div (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (div_fire),
        .in_ready (div_rdy),
        .in_a     (div_a),
        .in_b     (n_f),
        .out_valid(div_v),
        .out_ready(1'b1),
        .out_r    (div_r)
    );

    //--------------------------------------------------------------------
    // 组合控制信号
    //--------------------------------------------------------------------
    // 内循环扫描中（含在途结果返回期）
    assign dist_scanning =
        (state == S_IN_REQ) || (state == S_IN_LATCH) ||
        (state == S_IN_FEED) || (state == S_IN_DRAIN);

    // distance < radius：hypot 输出非负、radius 为正 → 无符号位比较即数值序
    assign lt_dist = $unsigned(hypot_r) < $unsigned(radius_reg);
    assign dist_hit = hypot_v && cfifo_out_valid && !used_bit[cfifo_jj] && lt_dist;
    // 命中但累加链忙（或 add 输入不 ready）→ 回压该 dist 结果
    assign acc_block = dist_hit &&
                       ((accx_busy || !addx_rdy) || (accy_busy || !addy_rdy));
    assign cfifo_pop = dist_scanning && hypot_v && cfifo_out_valid && !acc_block;

    // 内循环喂 sub：S_IN_FEED（数据拍）且 sub 与 coord FIFO 均就绪
    // （读口 rd_en=0 时 rd_x/rd_y 保持，背压期间数据稳定）
    assign feed_fire = (state == S_IN_FEED) &&
                       subx_rdy && suby_rdy && cfifo_in_ready;

    assign addx_fire = dist_hit && cfifo_pop;
    assign addy_fire = dist_hit && cfifo_pop;

    assign nf_fire  = (state == S_OUT_N) && nf_rdy;
    assign div_fire = ((state == S_OUT_DIVX) && nf_valid && div_rdy) ||
                      ((state == S_OUT_DIVY) && div_rdy);

    // sum / n 的除法输入（分时复用：先 x 后 y）
    assign div_a = (state == S_OUT_DIVX) ? sum_x_cur : sum_y_cur;
    assign cfifo_jj = cfifo_out[77:64];
    assign cfifo_px = cfifo_out[63:32];
    assign cfifo_py = cfifo_out[31:0];

    //--------------------------------------------------------------------
    // 块 A：主状态机（state/busy/done/i/j_cur/读口/buf/锚点/输出）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            i           <= 16'd0;
            j_cur       <= 16'd0;
            rd_en       <= 1'b0;
            rd_addr     <= {N_ADDR_W{1'b0}};
            anchor_x    <= 32'd0;
            anchor_y    <= 32'd0;
            res_valid   <= 1'b0;
            res_x       <= 32'd0;
            res_y       <= 32'd0;
            divx_out    <= 32'd0;
            divy_out    <= 32'd0;
            n_reg       <= 16'd0;
            radius_reg  <= 32'd0;
        end else begin
            rd_en <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy       <= 1'b1;
                        done        <= 1'b0;
                        i           <= 16'd0;
                        j_cur       <= 16'd0;
                        res_valid   <= 1'b0;
                        n_reg       <= n_in;
                        radius_reg  <= radius;
                        state       <= S_ANCHOR_REQ;
                    end
                end
                S_ANCHOR_REQ: begin
                    if (i >= n_reg) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        rd_en <= 1'b1;
                        rd_addr <= i[N_ADDR_W-1:0];
                        state <= S_ANCHOR_LAT;
                    end
                end
                S_ANCHOR_LAT: begin
                    // 本拍 RAM 采样地址；下一拍（S_ANCHOR_FEED）数据有效
                    if (used_bit[i[N_ADDR_W-1:0]]) begin
                        i <= i + 16'd1;
                        state <= S_ANCHOR_REQ;
                    end else
                        state <= S_ANCHOR_FEED;
                end
                S_ANCHOR_FEED: begin
                    // rd_x/rd_y = points[i]（读口 1 拍延迟，rd_en=0 保持）
                    anchor_x <= rd_x;
                    anchor_y <= rd_y;
                    j_cur    <= i + 16'd1;
                    state    <= S_IN_REQ;
                end
                S_IN_REQ: begin
                    if (j_cur >= n_reg) begin
                        state <= S_OUT_N;          // 内循环为空（单点锚）
                    end else begin
                        rd_en <= 1'b1;
                        rd_addr <= j_cur[N_ADDR_W-1:0];
                        state <= S_IN_LATCH;
                    end
                end
                S_IN_LATCH: begin
                    // 本拍 RAM 采样地址；下一拍（S_IN_FEED）数据有效
                    state <= S_IN_FEED;
                end
                S_IN_FEED: begin
                    if (feed_fire) begin
                        if (j_cur + 16'd1 < n_reg) begin
                            j_cur <= j_cur + 16'd1;
                            state <= S_IN_REQ;
                        end else
                            state <= S_IN_DRAIN;
                    end
                end
                S_IN_DRAIN: begin
                    if (in_flight == 0 && !accx_busy && !accy_busy)
                        state <= S_OUT_N;
                end
                S_OUT_N: begin
                    if (nf_rdy) state <= S_OUT_DIVX;
                end
                S_OUT_DIVX: begin
                    if (nf_valid && div_rdy) state <= S_OUT_WX;
                end
                S_OUT_WX: begin
                    if (div_v) begin
                        divx_out <= div_r;
                        state <= S_OUT_DIVY;
                    end
                end
                S_OUT_DIVY: begin
                    if (div_rdy) state <= S_OUT_WY;
                end
                S_OUT_WY: begin
                    if (div_v) begin
                        divy_out <= div_r;
                        res_valid <= 1'b1;
                        res_x     <= divx_out;
                        res_y     <= div_r;
                        state     <= S_OUT_RDY;
                    end
                end
                S_OUT_RDY: begin
                    if (res_ready) begin
                        res_valid <= 1'b0;
                        if (i + 16'd1 >= n_reg) begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_IDLE;
                        end else begin
                            i <= i + 16'd1;
                            state <= S_ANCHOR_REQ;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 块 B：sum 累加链 + n 计数
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            sum_x_cur <= 32'd0;
            sum_y_cur <= 32'd0;
            accx_busy <= 1'b0;
            accy_busy <= 1'b0;
            n_cnt     <= 16'd0;
        end else begin
            if (start) begin
                accx_busy <= 1'b0;
                accy_busy <= 1'b0;
            end
            if (state == S_ANCHOR_FEED) begin
                sum_x_cur <= rd_x;                 // sum 初值 = 锚点
                sum_y_cur <= rd_y;
                n_cnt     <= 16'd1;
            end else if (dist_hit && cfifo_pop) begin
                n_cnt <= n_cnt + 16'd1;
            end
            if (addx_v) begin
                sum_x_cur <= addx_r;
                accx_busy <= 1'b0;
            end
            if (addy_v) begin
                sum_y_cur <= addy_r;
                accy_busy <= 1'b0;
            end
            if (addx_fire) accx_busy <= 1'b1;
            if (addy_fire) accy_busy <= 1'b1;
        end
    end

    //--------------------------------------------------------------------
    // 块 C：used 位图
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            used_bit <= {DEPTH{1'b0}};
        else if (start)
            used_bit <= {DEPTH{1'b0}};
        else if (dist_hit && cfifo_pop)
            used_bit[cfifo_jj] <= 1'b1;
    end

    //--------------------------------------------------------------------
    // 块 D：在途 distance 计数（feed +1 / 结果 pop -1）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            in_flight <= 16'd0;
        else if (start)
            in_flight <= 16'd0;
        else if (feed_fire && cfifo_pop)
            in_flight <= in_flight;
        else if (feed_fire)
            in_flight <= in_flight + 16'd1;
        else if (cfifo_pop)
            in_flight <= in_flight - 16'd1;
    end

    //--------------------------------------------------------------------
    // 块 E：输出计数
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            res_count <= 15'd0;
        else if (start)
            res_count <= 15'd0;
        else if (res_valid && res_ready)
            res_count <= res_count + 15'd1;
    end

endmodule
