`timescale 1ns / 1ps
//==============================================================================
// window3x3.v — 流式 3×3 窗口生成器（行RAM轮换 + 列移位 + 输出FIFO）
//------------------------------------------------------------------------------
// 输入：CH 通道像素流（光栅序，从 (0,0) 开始），每拍 CH*DW 位。
// 输出：每个像素 (cx,cy) 的 3×3 窗口（9 组 CH 通道像素），光栅序，共 W*H 个。
//
// 结构（对应 VERILOG_DESIGN_PLAN 5.2：两行历史 RAM、横向寄存器）：
//   - 两张独立 read-first 行 RAM 按行奇偶轮换：奇数行写 RAM_A、偶数行写
//     RAM_B。当前节拍写"本行 RAM"的同址旧值 = 两行前像素（顶行源）；
//     另一张 RAM 只读 = 一行前像素（中行源）；底行 = 当前输入延迟 1 拍。
//   - 每通道三组 3 抽头列移位寄存器形成横向窗口。
//   - 扫描节拍流含虚拟节拍：每行末 1 拍（x=W，补最右列窗口）、帧末 1 个
//     虚拟行（row=H，补最后一行窗口），共 (W+1)*(H+1) 节拍，产出 W*H 个
//     窗口。虚拟节拍不消耗输入（in_ready=0 背压上游）。
//
// 流控架构（历多次反压丢窗口 bug 后的最终定型）：
//   生成核内部为"无反压直出"——所有流水级（sr 移位、元数据延迟）的推进
//   使能统一源自 scan_fire 的延迟脉冲（fire_d/fire_d2），fire 一停则全
//   流水整体冻结，不存在"数据冻结而元数据推进"的错位。
//   下游背压由输出侧 sync_fifo（深度 8）吸收：FIFO 水位 ≥ 6 时冻结
//   fire（在途窗口最多 2 个：fire→fire_d→fire_d2 后入队）。
//   任意 out_ready 波形下不丢窗口、保序、可背靠背。
//
// 边界语义（两种模式严格分开，规划 5.2 禁止含糊的 padding 开关）：
//   BORDER_CLAMP=1（Sobel 灰度窗用）：复制边缘像素。
//   BORDER_CLAMP=0（张量窗用）：图外贡献为零。
//   越界位置四种：row=1 顶行 / row=H 底行 / x=1 左列 / x=W 右列。
//
// 窗口像素序（out_data 拼接，k = r*3+c，r/c∈[0,2]，r=0 顶行）：
//   k0=左上 k1=中上 k2=右上 / k3=左中 k4=中心 k5=右中 / k6=左下 k7=中下 k8=右下
//   每组 CH*DW 位，通道 i 在组内 [i*DW +: DW]。
//==============================================================================
module window3x3 #(
    parameter CH           = 1,
    parameter DW           = 8,
    parameter IMG_W        = 1280,
    parameter IMG_H        = 720,
    parameter BORDER_CLAMP = 1
) (
    input  wire               clk,
    input  wire               rst_n,
    // 像素输入（光栅序；空闲态 in_valid 拉起即开始新帧）
    input  wire               in_valid,
    output wire               in_ready,
    input  wire [CH*DW-1:0]   in_data,
    // 窗口输出
    output wire               out_valid,
    input  wire               out_ready,
    output wire [9*CH*DW-1:0] out_data,
    output wire [10:0]        out_x,     // 窗口中心 cx
    output wire [10:0]        out_y      // 窗口中心 cy
);

    localparam CW = CH * DW;
    localparam AW = $clog2(IMG_W + 1);
    localparam RAM_DEPTH = (1 << AW);
    localparam FIFO_AW = 3;                       // FIFO 深度 8
    localparam [FIFO_AW:0] FREEZE_TH = FIFO_AW'(6); // 水位≥6 冻结（在途≤2）

    //--------------------------------------------------------------------
    // 输出 FIFO（吸收下游背压；count 用于冻结阈值）
    //--------------------------------------------------------------------
    wire                    fifo_in_ready;
    wire [FIFO_AW:0]        fifo_count;
    wire                    out_v, fifo_empty;
    wire [9*CW+21:0]        fifo_out;

    // fire 冻结：水位高时停止产生新节拍（在途窗口最多 2 个）
    wire fifo_freeze = (fifo_count >= FREEZE_TH);

    //--------------------------------------------------------------------
    // 扫描节拍状态机（L0）：节拍网格 (row_t, x_t) ∈ [0,H]×[0,W]，行优先
    //--------------------------------------------------------------------
    localparam S_IDLE = 2'd0, S_SCAN = 2'd1, S_DRAIN = 2'd2, S_DONE = 2'd3;
    reg [1:0]  state;
    reg [10:0] row_t, x_t;

    wire is_virtual = (x_t == IMG_W) || (row_t == IMG_H);
    // IDLE 态即绪：直接成交第一个像素（避免与带延迟上游互相等待死锁）
    assign in_ready = (state == S_IDLE) ||
                      ((state == S_SCAN) && !is_virtual && !fifo_freeze);
    wire pix_fire   = in_valid && in_ready;
    wire scan_fire  = ((state == S_IDLE) && in_valid) ||
                      ((state == S_SCAN) && !fifo_freeze && (is_virtual || pix_fire));

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            row_t <= 11'd0;
            x_t   <= 11'd0;
        end else begin
            case (state)
                S_IDLE: if (in_valid) begin
                    // 第一个像素 (0,0) 本拍已成交，下一节拍为 (0,1)
                    state <= S_SCAN;
                    row_t <= 11'd0;
                    x_t   <= 11'd1;
                end
                S_SCAN: if (scan_fire) begin
                    if (x_t == IMG_W) begin
                        x_t <= 11'd0;
                        if (row_t == IMG_H)
                            state <= S_DRAIN;
                        else
                            row_t <= row_t + 11'd1;
                    end else begin
                        x_t <= x_t + 11'd1;
                    end
                end
                S_DRAIN: state <= S_DONE;   // 流水残余由延迟链+FIFO 自行排空
                S_DONE:  state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 行 RAM（两张独立，read-first 同址读写）
    //   奇数行写 RAM_A；偶数行写 RAM_B
    //   顶行源 = 本行 RAM 同址旧值（两行前）；中行源 = 另一张 RAM（一行前）
    //--------------------------------------------------------------------
    reg [CW-1:0] ram_a [0:RAM_DEPTH-1];
    reg [CW-1:0] ram_b [0:RAM_DEPTH-1];
    reg [CW-1:0] ram_a_rd, ram_b_rd;

    wire sel_a = row_t[0];
    wire we_a  = pix_fire &&  sel_a;
    wire we_b  = pix_fire && !sel_a;
    wire [AW-1:0] raddr = x_t[AW-1:0];

    always @(posedge clk) begin
        if (we_a) ram_a[raddr] <= in_data;
    end
    always @(posedge clk) begin
        if (we_b) ram_b[raddr] <= in_data;
    end
    always @(posedge clk) begin
        ram_a_rd <= ram_a[raddr];   // read-first：写同拍读旧值
    end
    always @(posedge clk) begin
        ram_b_rd <= ram_b[raddr];
    end

    // 顶行源（两行前）与中行源（一行前）
    wire [CW-1:0] top_src = sel_a ? ram_a_rd : ram_b_rd;
    wire [CW-1:0] mid_src = sel_a ? ram_b_rd : ram_a_rd;

    //--------------------------------------------------------------------
    // 节拍数据延迟级：地址/fire 脉冲/输入像素（fire 拍 → 下一拍就绪）
    //--------------------------------------------------------------------
    reg [10:0]   x_t_d, row_t_d;
    reg          fire_d, fire_d2;
    reg [CW-1:0] p_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            x_t_d  <= 11'd0;
            row_t_d <= 11'd0;
            fire_d  <= 1'b0;
            fire_d2 <= 1'b0;
            p_d     <= {CW{1'b0}};
        end else begin
            x_t_d   <= x_t;
            row_t_d <= row_t;
            fire_d  <= scan_fire;
            fire_d2 <= fire_d;
            if (pix_fire)
                p_d <= in_data;
        end
    end

    // 行边界替换值（CLAMP=中行值 / ZERO=0）
    wire [CW-1:0] row_repl = BORDER_CLAMP ? mid_src : {CW{1'b0}};
    // fire_d 拍的三行新像素（该节拍的列）
    wire [CW-1:0] top_new = (row_t_d == 11'd1) ? row_repl : top_src;  // cy=0 顶行
    wire [CW-1:0] mid_new = mid_src;
    wire [CW-1:0] bot_new = (row_t_d == IMG_H) ? row_repl : p_d;      // cy=H-1 底行

    //--------------------------------------------------------------------
    // 元数据 L1（fire_d 装载）：节拍坐标（与 sr 移位同拍，天然同步）
    //--------------------------------------------------------------------
    reg [10:0] x_d1, row_d1;

    always @(posedge clk) begin
        if (!rst_n) begin
            x_d1   <= 11'd0;
            row_d1 <= 11'd0;
        end else if (fire_d) begin
            x_d1   <= x_t_d;
            row_d1 <= row_t_d;
        end
    end

    //--------------------------------------------------------------------
    // 列移位寄存器（fire_d 脉冲移位；fire 冻结时 fire_d=0，整体冻结）
    //--------------------------------------------------------------------
    reg [CW-1:0] sr_top [0:2];
    reg [CW-1:0] sr_mid [0:2];
    reg [CW-1:0] sr_bot [0:2];

    always @(posedge clk) begin
        if (!rst_n) begin
            sr_top[0] <= {CW{1'b0}}; sr_top[1] <= {CW{1'b0}}; sr_top[2] <= {CW{1'b0}};
            sr_mid[0] <= {CW{1'b0}}; sr_mid[1] <= {CW{1'b0}}; sr_mid[2] <= {CW{1'b0}};
            sr_bot[0] <= {CW{1'b0}}; sr_bot[1] <= {CW{1'b0}}; sr_bot[2] <= {CW{1'b0}};
        end else if (fire_d) begin
            sr_top[0] <= sr_top[1]; sr_top[1] <= sr_top[2]; sr_top[2] <= top_new;
            sr_mid[0] <= sr_mid[1]; sr_mid[1] <= sr_mid[2]; sr_mid[2] <= mid_new;
            sr_bot[0] <= sr_bot[1]; sr_bot[1] <= sr_bot[2]; sr_bot[2] <= bot_new;
        end
    end

    //--------------------------------------------------------------------
    // 窗口组合（fire_d2 拍有效：sr 含列 [x_d1-2, x_d1-1, x_d1]）
    //--------------------------------------------------------------------
    wire left_oob  = (x_d1 == 11'd1);      // cx=0：左列越界
    wire right_oob = (x_d1 == IMG_W);      // cx=W-1：右列越界
    wire is_window = (x_d1 >= 11'd1) && (row_d1 >= 11'd1);

    // 列边界替换值：CLAMP=同行中列 / ZERO=0
    wire [CW-1:0] edge0 = BORDER_CLAMP ? sr_top[1] : {CW{1'b0}};
    wire [CW-1:0] edge1 = BORDER_CLAMP ? sr_mid[1] : {CW{1'b0}};
    wire [CW-1:0] edge2 = BORDER_CLAMP ? sr_bot[1] : {CW{1'b0}};

    wire [CW-1:0] w_top_l = left_oob  ? edge0 : sr_top[0];
    wire [CW-1:0] w_top_c = sr_top[1];
    wire [CW-1:0] w_top_r = right_oob ? edge0 : sr_top[2];
    wire [CW-1:0] w_mid_l = left_oob  ? edge1 : sr_mid[0];
    wire [CW-1:0] w_mid_c = sr_mid[1];
    wire [CW-1:0] w_mid_r = right_oob ? edge1 : sr_mid[2];
    wire [CW-1:0] w_bot_l = left_oob  ? edge2 : sr_bot[0];
    wire [CW-1:0] w_bot_c = sr_bot[1];
    wire [CW-1:0] w_bot_r = right_oob ? edge2 : sr_bot[2];

    wire [9*CW-1:0] win_comb = {w_top_l, w_top_c, w_top_r,
                                w_mid_l, w_mid_c, w_mid_r,
                                w_bot_l, w_bot_c, w_bot_r};

    // 入队（填充节拍不入队；fire 冻结由水位保证不入队拍不满）
    wire win_push = fire_d2 && is_window;

    sync_fifo #(
        .DATA_WIDTH (9*CW + 22),
        .ADDR_WIDTH (FIFO_AW)
    ) u_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (win_push),
        .in_ready (fifo_in_ready),
        .in_data  ({win_comb, x_d1 - 11'd1, row_d1 - 11'd1}),
        .out_valid(out_v),
        .out_ready(out_ready),
        .out_data (fifo_out),
        .count    (fifo_count),
        .empty    (fifo_empty),
        .full     ()
    );

    assign out_valid = out_v;
    assign out_data  = fifo_out[9*CW+21:22];       // 高位：窗口数据
    assign out_x     = fifo_out[21:11];            // 低 22 位：x 在 [21:11]
    assign out_y     = fifo_out[10:0];             // y 在 [10:0]

endmodule
