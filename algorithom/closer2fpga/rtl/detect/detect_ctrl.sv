`timescale 1ns / 1ps
//==============================================================================
// detect_ctrl.sv — M6.2 单图多尺度检测调度（detect_chessboard 金字塔编排）
//------------------------------------------------------------------------------
// 架构（层槽位链 + 恢复编排）：
//
//   层 d（generate 槽位，d=0..DEPTH-1，W_d=W0>>d, H_d=H0>>d）：
//     gray_reader（灰度 RAM 光栅读器，registered 读、2 拍/像素、反压安全）
//       → window3x3(CH=1,DW=8,CLAMP=1) → sobel_core → tensor_core
//       → tensor_window_sum → s32_to_f32×3 → min_eigen_core
//       → resp sync_fifo → shi_tomasi_ctrl(d)（thr 由 detect_ctrl 共享 rmax 链）
//       → cand 流 → candidate_filter_ctrl(d)（gray_rd 相对 base_d）
//       → inner 流 → grid_order_ctrl(d) → 40 点流 → corner RAM_d（写口 1）
//       → grid_refine_ctrl(d)（pt_rd 读 corner RAM_d，out 流写回 corner RAM_d，
//                              gray_rd 相对 base_d）→ child_valid[d]=valid_out
//
//   基址布局（确定性）：base_d = cfg_base0 + Σ_{k<d} W_k*H_k；
//   层灰度由外部预生成在对应 base（本模块只消费，不生成缩图）。
//
//   C++ 语义对应（chessboard.cpp::detect_chessboard，export_m6.cpp 权威）：
//     - 先 d=DEPTH-1（最深）：native 全链（shi_tomasi→filter→order→refine）。
//     - 逐层向上 d=DEPTH-2..0：
//         child_valid[d+1] ? map（读 corner RAM_{d+1} 40 点，x'=2x+0.5f、
//                            y'=2y+0.5f，写 corner RAM_d）→ refine(d)：
//                            valid_out → child_valid[d]=1；失败 → native(d)
//                          : native(d)（grid_order 失败 → child_valid[d]=0）
//     - 完成：child_valid[0] → status=01，输出 corner RAM_0 40 点流；
//              否则 status=10，无输出。
//
//   thr 计算：对喂给 shi_tomasi_ctrl(d) 的 resp 流（fifo out 握手）逐拍 max
//     （位变换无符号比较，与 response_store_max 同算法）；每层 native 收满
//     PIXELS_d 个后 fp32_mul(rmax, 0x3DA3D70A) 闩存 thr（活动层互斥，共享一套）。
//
//   gray_rd 汇总：所有槽位 gray_rd 经 2 级选通（活动层 + 活动阶段：
//     native 期间 reader / filter 互斥，refine 期间 refine），
//     最终 gray_rd_addr = base_d + 槽位相对地址。
//
//   corner RAM（每层一个，CORNER_N×{x,y} fp32，dual_port_ram 1 拍读延迟）：
//     写口 3 选 1：grid_order out / grid_refine out / map 写入（阶段互斥）；
//     读口 3 选 1：refine pt_rd（读 RAM_d）/ map 读取（读 RAM_{d+1}）/
//                  S_OUT 输出（读 RAM_0）。
//
//   接口时序：
//     start：busy=0 时单拍启动（全状态复位，连续帧复用）。
//     done ：电平保持（下次 start 清零）。
//     out 流：out_valid/out_ready 握手，40 点行列序 fp32，out_grid_ok=1；
//     status：01=有角点 10=无角点。
//==============================================================================

//------------------------------------------------------------------------------
// gray_reader — 灰度 RAM 光栅读器（registered 读语义：rd_en=1 下一拍 rd_data）
//------------------------------------------------------------------------------
// 时序（2 拍/像素，反压安全）：
//   R_REQ：呈现地址 r_cnt（rd_en=1）→ RAM 下一拍出 mem[r_cnt]
//   R_ABS ：in_valid=1 且数据有效；in_ready=0 时保持（地址不变 → 数据不变）
//   吸收拍后 r_cnt+1 → R_REQ。光栅序共 PIXELS 个像素后置 R_DONE（in_valid=0）。
//------------------------------------------------------------------------------
module gray_reader #(
    parameter PIXELS = 921600,
    parameter ADDR_W = 20
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,              // IDLE 时单拍启动
    input  wire [7:0] gray_rd_data, // 共享灰度 RAM 读数据（1 拍延迟）
    output wire in_valid,
    input  wire in_ready,
    output wire [7:0] in_data,      // = gray_rd_data（组合）
    output wire rd_en,
    output wire [ADDR_W-1:0] rd_addr
);

    localparam R_IDLE = 2'd0, R_REQ = 2'd1, R_ABS = 2'd2, R_DONE = 2'd3;
    reg [1:0]   rs;
    reg [ADDR_W:0] r_cnt;           // 当前像素号（0..PIXELS-1）

    assign in_valid = (rs == R_ABS);
    assign in_data  = gray_rd_data;
    assign rd_en    = (rs == R_REQ) || (rs == R_ABS);
    assign rd_addr  = r_cnt[ADDR_W-1:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            rs    <= R_IDLE;
            r_cnt <= {ADDR_W{1'b0}};
        end else begin
            case (rs)
                R_IDLE: begin
                    r_cnt <= {ADDR_W{1'b0}};
                    if (start) rs <= R_REQ;
                end
                R_REQ: begin
                    rs <= R_ABS;
                end
                R_ABS: begin
                    if (in_valid && in_ready) begin
                        if (r_cnt >= PIXELS - 1) rs <= R_DONE;
                        else begin
                            r_cnt <= r_cnt + 1'b1;
                            rs    <= R_REQ;
                        end
                    end
                end
                default: begin   // R_DONE
                    rs <= R_IDLE;
                end
            endcase
        end
    end

endmodule

//------------------------------------------------------------------------------
// detect_ctrl — 顶层调度
//------------------------------------------------------------------------------
module detect_ctrl #(
    parameter W0          = 1280,
    parameter H0          = 720,
    parameter DEPTH       = 2,           // 层数（含 L0；DEPTH=1 无金字塔）
    parameter GRAY_ADDR_W = 26,          // ≥ $clog2(Σ_{d<DEPTH} W_d*H_d + base0)
    parameter ROWS        = 5,
    parameter COLS        = 8,
    parameter CORNER_N    = ROWS*COLS,
    parameter CORNER_AW   = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,        // busy=0 时单拍启动
    input  wire [GRAY_ADDR_W-1:0]  cfg_base0,    // L0 灰度基址（通常 0）
    output reg                     busy,
    output reg                     done,         // 电平保持（下次 start 清零）
    output reg  [1:0]              status,       // 01=有角点 10=无角点
    // 灰度 RAM 读口（registered 读：rd_en=1 下一拍出数据）
    output wire                    gray_rd_en,
    output wire [GRAY_ADDR_W-1:0]  gray_rd_addr,
    input  wire [7:0]              gray_rd_data,
    // 响应流探针（Pass1 native 期间当前层 resp 流直出，供响应图写 DDR 等）
    output wire                    resp_tap_valid,
    output wire [31:0]             resp_tap_data,
    // 响应图帧级导出（M7.3：detect 有角点路径 ST_OUT 后，读回最深层槽位
    //   resp RAM 全图 PIXELS fp32 → 外部写 DDR；cfg_resp_dump_en=0 时行为不变）
    input  wire                    cfg_resp_dump_en,
    output wire                    resp_dump_valid,
    input  wire                    resp_dump_ready,
    output wire [31:0]             resp_dump_data,
    output reg                     resp_dump_done,   // 电平（离开 ST_DUMP / 下次 start 清零）
    // 有序角点输出（40 点流）
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_x,
    output reg  [31:0]             out_y,
    output reg  [15:0]             out_total,
    output reg                     out_grid_ok
);

    //--------------------------------------------------------------------
    // 常量与层尺寸函数
    //--------------------------------------------------------------------
    localparam [31:0] C_2F   = 32'h40000000;   // 2.0f（map x2）
    localparam [31:0] C_HALF = 32'h3F000000;   // 0.5f（map +0.5）
    localparam [31:0] C_008  = 32'h3DA3D70A;   // 0.08f（thr 链）
    localparam [31:0] NEGINF = 32'hFF800000;   // -inf（rmax 初值）

    // 层尺寸（可折叠常数函数）
    function automatic integer lay_w(input integer d);
        lay_w = W0 >> d;
    endfunction
    function automatic integer lay_h(input integer d);
        lay_h = H0 >> d;
    endfunction
    function automatic integer pix_of(input integer d);
        pix_of = (W0 >> d) * (H0 >> d);
    endfunction
    function automatic integer lay_off(input integer d);
        integer k, acc;
        begin
            acc = 0;
            for (k = 0; k < d; k = k + 1)
                acc = acc + ((W0 >> k) * (H0 >> k));
            lay_off = acc;
        end
    endfunction

    // fp32 → 整数序位变换（无符号比较即 IEEE 顺序，同 response_store_max）
    function automatic [31:0] fp_ord(input [31:0] a);
        fp_ord = a[31] ? ~a : (a | 32'h8000_0000);
    endfunction

    //--------------------------------------------------------------------
    // 顶层状态
    //--------------------------------------------------------------------
    localparam ST_IDLE   = 4'd0;
    localparam ST_NATIVE = 4'd1;   // 层 dl native：shi/filter/order/reader 运行
    localparam ST_ORDER  = 4'd2;   // 收 grid_order(dl) 40 点流 → corner RAM_dl
    localparam ST_REFINE = 4'd3;   // refine(dl) 运行 + 收 out 流写回 corner RAM_dl
    localparam ST_MAP    = 4'd4;   // map：读 RAM_{dl+1} 40 点 → 2p+0.5 → 写 RAM_dl
    localparam ST_NEXT   = 4'd5;   // 层推进
    localparam ST_FIN    = 4'd6;   // 收尾：status 判定
    localparam ST_OUT    = 4'd7;   // 输出 corner RAM_0 40 点流
    localparam ST_DUMP   = 4'd8;   // M7.3：帧级响应图导出（读回最深层 resp RAM）
    reg [3:0] stage;

    localparam M_IDLE = 3'd0, M_RD = 3'd1, M_RD2 = 3'd2, M_MULW = 3'd3,
               M_ADDW = 3'd4, M_WR = 3'd5, M_DONE = 3'd6;
    reg [2:0] m_st;

    localparam O_IDLE = 2'd0, O_RD = 2'd1, O_EMIT = 2'd2;
    reg [1:0] o_st;

    //--------------------------------------------------------------------
    // 控制寄存器
    //--------------------------------------------------------------------
    reg [7:0]  dl;                 // 当前层
    reg        nat_entry, ref_entry;  // native/refine 启动脉冲（1 拍）
    reg [DEPTH-1:0] child_valid;   // 各层检测有效标志
    reg [15:0] oc, rc;             // order/refine 输出收集写指针
    reg [15:0] m_i;                // map 循环索引
    reg [31:0] map_x_r, map_y_r;   // map 读点锁存
    reg [31:0] mulx_r, muly_r;     // map mul 结果锁存
    reg [31:0] map_wrx_r, map_wry_r;  // map add 结果锁存（写 RAM）
    reg        m_fire, a_fire;     // map mul/add 启动脉冲
    reg        map_done;           // map 完成（电平）
    reg [15:0] out_oc;             // 输出流索引
    reg [31:0] rmax_r, thr_r;      // resp max / 闩存 thr
    reg [19:0] p1_cnt;             // 当前 native 已收 resp 数
    reg        thr_mul_fire;
    // M7.3 帧级响应图导出（块 4；声明在 generate 之前供 slot_dump_addr 引用）
    reg [1:0]  d_st;
    reg [19:0] d_addr;
    reg        d_v;

    //--------------------------------------------------------------------
    // 槽位互连 wire 数组（generate 赋值）
    //--------------------------------------------------------------------
    wire        slot_filter_en  [0:DEPTH-1];
    wire [GRAY_ADDR_W-1:0] slot_filter_addr [0:DEPTH-1];
    wire        slot_refine_en  [0:DEPTH-1];
    wire [GRAY_ADDR_W-1:0] slot_refine_addr [0:DEPTH-1];
    wire        slot_gray_en    [0:DEPTH-1];
    wire [GRAY_ADDR_W-1:0] slot_gray_addr [0:DEPTH-1];
    wire        slot_order_v    [0:DEPTH-1];
    wire [31:0] slot_order_x    [0:DEPTH-1];
    wire [31:0] slot_order_y    [0:DEPTH-1];
    wire        slot_order_done [0:DEPTH-1];
    wire        slot_order_gok  [0:DEPTH-1];
    wire        slot_refine_v   [0:DEPTH-1];
    wire [31:0] slot_refine_x   [0:DEPTH-1];
    wire [31:0] slot_refine_y   [0:DEPTH-1];
    wire        slot_refine_done[0:DEPTH-1];
    wire        slot_refine_val [0:DEPTH-1];
    wire        slot_refine_pt_en  [0:DEPTH-1];
    wire [CORNER_AW-1:0] slot_refine_pt_addr [0:DEPTH-1];
    wire        slot_resp_fire   [0:DEPTH-1];
    wire [31:0] slot_resp_data   [0:DEPTH-1];
    // M7.3 dump 读回（ST_DUMP 期间只驱动最深层槽位）
    wire        slot_dump_en     [0:DEPTH-1];
    wire [19:0] slot_dump_addr   [0:DEPTH-1];
    wire [31:0] slot_dump_data   [0:DEPTH-1];
    wire [63:0] cram_rd64        [0:DEPTH-1];
    wire [31:0] cram_rd_x        [0:DEPTH-1];
    wire [31:0] cram_rd_y        [0:DEPTH-1];
    wire        cram_wr_en       [0:DEPTH-1];
    wire [CORNER_AW-1:0] cram_wr_addr [0:DEPTH-1];
    wire [63:0] cram_wr_data     [0:DEPTH-1];
    wire [GRAY_ADDR_W-1:0] base_d [0:DEPTH-1];

    // map / out 组合读口
    wire map_rd_en  = (m_st == M_RD);
    wire map_wr_en  = (m_st == M_WR);
    wire out_rd_en  = (o_st == O_RD);

    // thr 链组合
    wire thr_fire    = (stage == ST_NATIVE) && slot_resp_fire[dl];
    wire [31:0] thr_data = slot_resp_data[dl];
    wire [19:0] p1_lim   = pix_of(dl);
    wire thr_mul_rdy, thr_mul_v;
    wire [31:0] thr_mul_r;

    // resp 探针（Pass1 native 期间当前层 resp 流直出）
    assign resp_tap_valid = (stage == ST_NATIVE) ? slot_resp_fire[dl] : 1'b0;
    assign resp_tap_data  = (stage == ST_NATIVE) ? slot_resp_data[dl] : 32'd0;

    // map 算术互连
    wire mul_rdy_x, mul_rdy_y, mul_v_x, mul_v_y, add_rdy_x, add_rdy_y;
    wire add_v_x, add_v_y;
    wire [31:0] mul_r_x, mul_r_y, add_r_x, add_r_y;

    //--------------------------------------------------------------------
    // 层槽位 generate
    //--------------------------------------------------------------------
    genvar d;
    generate
        for (d = 0; d < DEPTH; d = d + 1) begin : g_slot
            localparam integer WD = W0 >> d;
            localparam integer HD = H0 >> d;
            localparam integer PIX = WD * HD;
            localparam integer LAY_OFF = lay_off(d);
            localparam integer R_AW = $clog2(PIX);

            wire [GRAY_ADDR_W-1:0] base_w = cfg_base0 + LAY_OFF[GRAY_ADDR_W-1:0];

            //---- 前端链 ----
            wire        rdr_v, rdr_rdy, rdr_en;
            wire [7:0]  rdr_data;
            wire [R_AW-1:0] rdr_addr;
            wire        wv, wrdy;  wire [71:0] wd; wire [10:0] wx, wy;
            wire        sv_, srdy; wire [15:0] sgx, sgy; wire [10:0] sx, sy;
            wire        tv, trdy;  wire [31:0] txx, txy, tyy; wire [10:0] tx, ty;
            wire        av, mrdy;  wire [31:0] s_a, s_b_, s_c;
            wire        fa_v, fb_v, fc_v; wire [31:0] fa_r, fb_r, fc_r;
            wire        me_v, me_rdy; wire [31:0] me_resp;
            wire        fv, frdy;  wire [31:0] fd;
            //---- shi→filter→order→refine ----
            wire        cand_v, cand_rdy; wire [10:0] cand_x, cand_y;
            wire [15:0] cand_total;
            wire        shi_done;
            wire        inner_v, inner_rdy; wire [31:0] inner_x, inner_y;
            wire [15:0] inner_total;
            wire        filter_done;
            wire        order_v, order_rdy; wire [31:0] order_x, order_y;
            wire [15:0] order_total; wire order_gok;
            wire        order_done_d;
            wire        refine_v, refine_rdy; wire [31:0] refine_x, refine_y;
            wire        refine_done_d, refine_valid_d;
            wire        cram_rd_en_d;
            wire [CORNER_AW-1:0] cram_rd_addr_d;

            //---- 启动脉冲（组合，nat_entry/ref_entry 1 拍）----
            wire sh_start    = (stage == ST_NATIVE) && nat_entry && (dl == d);
            wire filter_start= sh_start;
            wire order_start = sh_start;
            wire reader_start= sh_start;
            wire refine_start= (stage == ST_REFINE) && ref_entry && (dl == d);

            gray_reader #(.PIXELS(PIX), .ADDR_W(R_AW)) u_reader (
                .clk(clk), .rst_n(rst_n),
                .start(reader_start),
                .gray_rd_data(gray_rd_data),
                .in_valid(rdr_v), .in_ready(rdr_rdy), .in_data(rdr_data),
                .rd_en(rdr_en), .rd_addr(rdr_addr)
            );

            window3x3 #(.CH(1), .DW(8), .IMG_W(WD), .IMG_H(HD),
                        .BORDER_CLAMP(1'b1)) u_win (
                .clk(clk), .rst_n(rst_n),
                .in_valid(rdr_v), .in_ready(rdr_rdy), .in_data(rdr_data),
                .out_valid(wv), .out_ready(wrdy), .out_data(wd),
                .out_x(wx), .out_y(wy)
            );
            sobel_core u_sob (
                .clk(clk), .rst_n(rst_n),
                .in_valid(wv), .in_ready(wrdy), .in_win(wd), .in_x(wx), .in_y(wy),
                .out_valid(sv_), .out_ready(srdy), .out_gx(sgx), .out_gy(sgy),
                .out_x(sx), .out_y(sy)
            );
            tensor_core u_tc (
                .clk(clk), .rst_n(rst_n),
                .in_valid(sv_), .in_ready(srdy), .in_gx(sgx), .in_gy(sgy),
                .in_x(sx), .in_y(sy),
                .out_valid(tv), .out_ready(trdy), .out_xx(txx), .out_xy(txy),
                .out_yy(tyy), .out_x(tx), .out_y(ty)
            );
            tensor_window_sum #(.IMG_W(WD), .IMG_H(HD)) u_tws (
                .clk(clk), .rst_n(rst_n),
                .in_valid(tv), .in_ready(trdy), .in_xx(txx), .in_xy(txy), .in_yy(tyy),
                .in_x(tx), .in_y(ty),
                .out_valid(av), .out_ready(mrdy), .out_a(s_a), .out_b(s_b_),
                .out_c(s_c), .out_x(), .out_y()
            );
            s32_to_f32 u_fa (
                .clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_a),
                .out_valid(fa_v), .out_ready(mrdy), .out_r(fa_r)
            );
            s32_to_f32 u_fb (
                .clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_b_),
                .out_valid(fb_v), .out_ready(mrdy), .out_r(fb_r)
            );
            s32_to_f32 u_fc (
                .clk(clk), .rst_n(rst_n), .in_valid(av), .in_ready(), .in_data(s_c),
                .out_valid(fc_v), .out_ready(mrdy), .out_r(fc_r)
            );
            min_eigen_core u_me (
                .clk(clk), .rst_n(rst_n),
                .in_valid(fa_v), .in_ready(mrdy),
                .in_a(fa_r), .in_b(fb_r), .in_c(fc_r),
                .out_valid(me_v), .out_ready(me_rdy), .out_resp(me_resp)
            );
            // me → ctrl 桥接 FIFO（同 tb_resp 接线：out_ready 反驱 me）
            sync_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(5)) u_fifo (
                .clk(clk), .rst_n(rst_n),
                .in_valid(me_v), .in_ready(me_rdy), .in_data(me_resp),
                .out_valid(fv), .out_ready(frdy), .out_data(fd),
                .count(), .empty(), .full()
            );
            shi_tomasi_ctrl #(.IMG_W(WD), .IMG_H(HD)) u_shi (
                .clk(clk), .rst_n(rst_n),
                .start(sh_start), .busy(), .thr(thr_r),
                .resp_valid(fv), .resp_rdy(frdy), .resp_data(fd),
                .resp_in_ready(me_rdy),
                .mem_addr(), .mem_data(),
                .dump_en(slot_dump_en[d]), .dump_addr(slot_dump_addr[d]),
                .dump_data(slot_dump_data[d]),
                .done(shi_done), .status(),
                .cand_valid(cand_v), .cand_ready(cand_rdy),
                .cand_x(cand_x), .cand_y(cand_y), .cand_total(cand_total)
            );
            candidate_filter_ctrl #(
                .IMG_W(WD), .IMG_H(HD), .GRAY_ADDR_W(GRAY_ADDR_W)
            ) u_filter (
                .clk(clk), .rst_n(rst_n),
                .start(filter_start), .busy(), .done(filter_done), .status(),
                .cand_valid(cand_v), .cand_ready(cand_rdy),
                .cand_x(cand_x), .cand_y(cand_y), .cand_done(shi_done),
                .gray_rd_en(slot_filter_en[d]), .gray_rd_addr(slot_filter_addr[d]),
                .gray_rd_data(gray_rd_data),
                .inner_valid(inner_v), .inner_ready(inner_rdy),
                .inner_x(inner_x), .inner_y(inner_y), .inner_total(inner_total),
                .probe_valid(), .probe_hi(), .probe_lo(), .probe_thr(),
                .probe_ntrans(), .probe_opp_err(), .probe_sector_ok(), .probe_pass()
            );
            grid_order_ctrl #(.ROWS(ROWS), .COLS(COLS)) u_order (
                .clk(clk), .rst_n(rst_n),
                .start(order_start), .busy(), .done(order_done_d), .status(),
                .pts_valid(inner_v), .pts_ready(inner_rdy),
                .pts_x(inner_x), .pts_y(inner_y), .pts_done(filter_done),
                .out_valid(order_v), .out_ready(1'b1),
                .out_x(order_x), .out_y(order_y),
                .out_total(order_total), .out_grid_ok(order_gok)
            );
            grid_refine_ctrl #(
                .ROWS(ROWS), .COLS(COLS), .N_ADDR_W(CORNER_AW),
                .IMG_W(WD), .IMG_H(HD), .GRAY_ADDR_W(GRAY_ADDR_W)
            ) u_refine (
                .clk(clk), .rst_n(rst_n),
                .start(refine_start), .busy(), .done(refine_done_d),
                .valid_out(refine_valid_d),
                .pt_rd_en(slot_refine_pt_en[d]),
                .pt_rd_addr(slot_refine_pt_addr[d]),
                .pt_rd_x(cram_rd_x[d]), .pt_rd_y(cram_rd_y[d]),
                .gray_rd_en(slot_refine_en[d]), .gray_rd_addr(slot_refine_addr[d]),
                .gray_rd_data(gray_rd_data),
                .out_valid(refine_v), .out_ready(1'b1),
                .out_x(refine_x), .out_y(refine_y), .out_valid_flag()
            );

            //---- corner RAM_d ----
            dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(CORNER_AW)) u_cram (
                .clk(clk), .rst_n(rst_n),
                .wr_en(cram_wr_en[d]), .wr_addr(cram_wr_addr[d]),
                .wr_data(cram_wr_data[d]),
                .rd_en(cram_rd_en_d), .rd_addr(cram_rd_addr_d),
                .rd_data(cram_rd64[d])
            );

            //---- 槽位汇出（灰色/响应/序/精定位）----
            // gray 2 级选通第 1 级：活动层 + 活动阶段
            assign slot_gray_en[d] = (dl == d) && (
                ((stage == ST_NATIVE) && (rdr_en || slot_filter_en[d])) ||
                ((stage == ST_REFINE) && slot_refine_en[d]));
            assign slot_gray_addr[d] =
                ((dl == d) && (stage == ST_NATIVE)) ?
                    (slot_filter_en[d] ? slot_filter_addr[d] :
                     {{(GRAY_ADDR_W-R_AW){1'b0}}, rdr_addr}) :
                ((dl == d) && (stage == ST_REFINE)) ? slot_refine_addr[d] :
                {GRAY_ADDR_W{1'b0}};

            assign slot_order_v[d]    = order_v;
            assign slot_order_x[d]    = order_x;
            assign slot_order_y[d]    = order_y;
            assign slot_order_done[d] = order_done_d;
            assign slot_order_gok[d]  = order_gok;
            assign slot_refine_v[d]   = refine_v;
            assign slot_refine_x[d]   = refine_x;
            assign slot_refine_y[d]   = refine_y;
            assign slot_refine_done[d]= refine_done_d;
            assign slot_refine_val[d] = refine_valid_d;
            assign slot_resp_fire[d]  = fv && frdy;
            assign slot_resp_data[d]  = fd;
            assign slot_dump_en[d]    = (stage == ST_DUMP) && (d == DEPTH - 1);
            assign slot_dump_addr[d]  = d_addr;
            assign base_d[d]          = base_w;

            // corner RAM 读口 3 选 1（refine pt_rd / map 读 / S_OUT 输出）
            assign cram_rd_en_d =
                ((stage == ST_REFINE) && (dl == d)) ? slot_refine_pt_en[d] :
                ((stage == ST_OUT)    && (dl == d)) ? out_rd_en :
                ((stage == ST_MAP)    && (dl + 1 == d)) ? map_rd_en : 1'b0;
            assign cram_rd_addr_d =
                ((stage == ST_REFINE) && (dl == d)) ? slot_refine_pt_addr[d] :
                ((stage == ST_OUT)    && (dl == d)) ? out_oc[CORNER_AW-1:0] :
                ((stage == ST_MAP)    && (dl + 1 == d)) ? m_i[CORNER_AW-1:0] :
                {CORNER_AW{1'b0}};

            // corner RAM 写口 3 选 1（order out / refine out / map 写）
            assign cram_wr_en[d] = (dl == d) && (
                ((stage == ST_ORDER)  && order_v) ||
                ((stage == ST_REFINE) && refine_v) ||
                ((stage == ST_MAP)    && map_wr_en));
            assign cram_wr_addr[d] = (dl == d) ?
                ((stage == ST_ORDER)  ? oc[CORNER_AW-1:0] :
                 (stage == ST_REFINE) ? rc[CORNER_AW-1:0] :
                 m_i[CORNER_AW-1:0]) : {CORNER_AW{1'b0}};
            assign cram_wr_data[d] =
                ((stage == ST_ORDER)  ? {order_y, order_x} :
                 (stage == ST_REFINE) ? {refine_y, refine_x} :
                 {map_wry_r, map_wrx_r});
        end
    endgenerate

    // corner RAM 读数据拆分
    genvar rd;
    generate
        for (rd = 0; rd < DEPTH; rd = rd + 1) begin : g_cramrd
            assign cram_rd_x[rd] = cram_rd64[rd][31:0];
            assign cram_rd_y[rd] = cram_rd64[rd][63:32];
        end
    endgenerate

    //--------------------------------------------------------------------
    // gray_rd 汇总（第 2 级：活动层槽位 → base_d 偏移）
    //--------------------------------------------------------------------
    wire [GRAY_ADDR_W-1:0] gray_mux [0:DEPTH-1];
    genvar gm;
    generate
        for (gm = 0; gm < DEPTH; gm = gm + 1) begin : g_gmux
            assign gray_mux[gm] =
                slot_gray_en[gm] ? (base_d[gm] + slot_gray_addr[gm]) :
                {GRAY_ADDR_W{1'b0}};
        end
    endgenerate
    wire [GRAY_ADDR_W-1:0] gray_or [0:DEPTH-1];
    generate
        for (gm = 0; gm < DEPTH; gm = gm + 1) begin : g_gor
            if (gm == 0) assign gray_or[0] = gray_mux[0];
            else         assign gray_or[gm] = gray_or[gm-1] | gray_mux[gm];
        end
    endgenerate
    assign gray_rd_addr = gray_or[DEPTH-1];

    wire [DEPTH-1:0] gray_ens;
    generate
        for (gm = 0; gm < DEPTH; gm = gm + 1) begin : g_gen
            assign gray_ens[gm] = slot_gray_en[gm];
        end
    endgenerate
    assign gray_rd_en = |gray_ens;

    //--------------------------------------------------------------------
    // thr 链：fp32_mul(rmax, 0.08f) 闩存 thr
    //--------------------------------------------------------------------
    fp32_mul u_thrmul (
        .clk(clk), .rst_n(rst_n),
        .in_valid(thr_mul_fire), .in_ready(thr_mul_rdy),
        .in_a(rmax_r), .in_b(C_008),
        .out_valid(thr_mul_v), .out_ready(1'b1), .out_r(thr_mul_r)
    );

    //--------------------------------------------------------------------
    // map 算术（x/y 各一条 mul+add；共享，活动层互斥）
    //--------------------------------------------------------------------
    fp32_mul u_mmulx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m_fire), .in_ready(mul_rdy_x),
        .in_a(map_x_r), .in_b(C_2F),
        .out_valid(mul_v_x), .out_ready(1'b1), .out_r(mul_r_x)
    );
    fp32_mul u_mmuly (
        .clk(clk), .rst_n(rst_n),
        .in_valid(m_fire), .in_ready(mul_rdy_y),
        .in_a(map_y_r), .in_b(C_2F),
        .out_valid(mul_v_y), .out_ready(1'b1), .out_r(mul_r_y)
    );
    fp32_add u_maddx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a_fire), .in_ready(add_rdy_x),
        .in_a(mulx_r), .in_b(C_HALF),
        .out_valid(add_v_x), .out_ready(1'b1), .out_r(add_r_x)
    );
    fp32_add u_maddy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(a_fire), .in_ready(add_rdy_y),
        .in_a(muly_r), .in_b(C_HALF),
        .out_valid(add_v_y), .out_ready(1'b1), .out_r(add_r_y)
    );

    //--------------------------------------------------------------------
    // 块 1：map 子状态机（读 RAM_{dl+1} → 2p+0.5 → 写 RAM_dl）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            m_st       <= M_IDLE;
            m_i        <= 16'd0;
            m_fire     <= 1'b0;
            a_fire     <= 1'b0;
            map_done   <= 1'b0;
            map_x_r    <= 32'd0;
            map_y_r    <= 32'd0;
            mulx_r     <= 32'd0;
            muly_r     <= 32'd0;
            map_wrx_r  <= 32'd0;
            map_wry_r  <= 32'd0;
        end else begin
            m_fire     <= 1'b0;
            a_fire     <= 1'b0;
            map_done   <= 1'b0;
            if (stage != ST_MAP) begin
                m_st <= M_IDLE;
            end else begin
                case (m_st)
                    M_IDLE: begin
                        m_i  <= 16'd0;
                        m_st <= M_RD;
                    end
                    M_RD: begin
                        m_st <= M_RD2;
                    end
                    M_RD2: begin
                        // 数据本拍到达（M_RD 请求、registered 读）
                        if (mul_rdy_x && mul_rdy_y) begin
                            map_x_r <= cram_rd_x[dl + 1];
                            map_y_r <= cram_rd_y[dl + 1];
                            m_fire  <= 1'b1;
                            m_st    <= M_MULW;
                        end
                    end
                    M_MULW: begin
                        if (mul_v_x && mul_v_y) begin
                            mulx_r  <= mul_r_x;
                            muly_r  <= mul_r_y;
                            a_fire  <= 1'b1;
                            m_st    <= M_ADDW;
                        end
                    end
                    M_ADDW: begin
                        if (add_v_x && add_v_y) begin
                            map_wrx_r <= add_r_x;
                            map_wry_r <= add_r_y;
                            m_st      <= M_WR;
                        end
                    end
                    M_WR: begin
                        // 本拍组合 wr_en=map_wr_en、addr=m_i、data=锁存 add 结果
                        if (m_i + 1 >= CORNER_N) begin
                            map_done <= 1'b1;
                            m_st     <= M_DONE;
                        end else begin
                            m_i  <= m_i + 16'd1;
                            m_st <= M_RD;
                        end
                    end
                    M_DONE: begin
                        map_done <= 1'b1;
                    end
                    default: m_st <= M_IDLE;
                endcase
            end
        end
    end

    //--------------------------------------------------------------------
    // 块 2：rmax/thr 跟踪（当前 native 的 resp 流）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            rmax_r    <= NEGINF;
            p1_cnt    <= 20'd0;
            thr_mul_fire <= 1'b0;
            thr_r     <= 32'd0;
        end else begin
            thr_mul_fire <= 1'b0;
            if (thr_mul_v) thr_r <= thr_mul_r;
            if (thr_fire) begin
                if ($unsigned(fp_ord(thr_data)) > $unsigned(fp_ord(rmax_r)))
                    rmax_r <= thr_data;
                if (p1_cnt == p1_lim - 1)
                    thr_mul_fire <= 1'b1;
                p1_cnt <= p1_cnt + 20'd1;
            end
        end
    end

    //--------------------------------------------------------------------
    // 块 3：主状态机
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            stage      <= ST_IDLE;
            busy       <= 1'b0;
            done       <= 1'b0;
            status     <= 2'b00;
            out_valid  <= 1'b0;
            out_x      <= 32'd0;
            out_y      <= 32'd0;
            out_total  <= 16'd0;
            out_grid_ok<= 1'b0;
            dl         <= 8'd0;
            nat_entry  <= 1'b0;
            ref_entry  <= 1'b0;
            child_valid<= {DEPTH{1'b0}};
            oc         <= 16'd0;
            rc         <= 16'd0;
            o_st       <= O_IDLE;
            out_oc     <= 16'd0;
        end else begin
            nat_entry <= 1'b0;
            ref_entry <= 1'b0;
            case (stage)
                //-------------------------------------------- IDLE
                ST_IDLE: begin
                    if (start) begin
                        busy        <= 1'b1;
                        done        <= 1'b0;
                        status      <= 2'b00;
                        out_valid   <= 1'b0;
                        out_grid_ok <= 1'b0;
                        out_total   <= 16'd0;
                        child_valid <= {DEPTH{1'b0}};
                        dl          <= DEPTH - 1;
                        nat_entry   <= 1'b1;
                        rmax_r      <= NEGINF;
                        p1_cnt      <= 20'd0;
                        stage       <= ST_NATIVE;
                    end
                end
                //-------------------------------------------- NATIVE（等 grid_order done）
                ST_NATIVE: begin
                    if (!nat_entry && slot_order_done[dl]) begin
                        if (slot_order_gok[dl]) begin
                            oc    <= 16'd0;
                            stage <= ST_ORDER;
                        end else begin
                            child_valid[dl] <= 1'b0;
                            stage <= ST_NEXT;
                        end
                    end
                end
                //-------------------------------------------- ORDER（收 40 点写 corner RAM_dl）
                ST_ORDER: begin
                    if (slot_order_v[dl]) begin
                        if (oc + 16'd1 >= CORNER_N[15:0]) begin
                            rc        <= 16'd0;
                            ref_entry <= 1'b1;
                            stage     <= ST_REFINE;
                        end else begin
                            oc <= oc + 16'd1;
                        end
                    end
                end
                //-------------------------------------------- REFINE（精定位 + 收流写回）
                ST_REFINE: begin
                    if (!ref_entry) begin
                        if (slot_refine_done[dl]) begin
                            child_valid[dl] <= slot_refine_val[dl];
                            stage <= ST_NEXT;
                        end else if (slot_refine_v[dl]) begin
                            if (rc + 16'd1 < CORNER_N[15:0])
                                rc <= rc + 16'd1;
                        end
                    end
                end
                //-------------------------------------------- MAP（子层映射）
                ST_MAP: begin
                    if (map_done) begin
                        rc        <= 16'd0;
                        ref_entry <= 1'b1;
                        stage     <= ST_REFINE;
                    end
                end
                //-------------------------------------------- NEXT（层推进）
                ST_NEXT: begin
                    if (dl == 8'd0) begin
                        stage <= ST_FIN;
                    end else if (child_valid[dl]) begin
                        dl    <= dl - 8'd1;
                        stage <= ST_MAP;
                    end else begin
                        dl       <= dl - 8'd1;
                        nat_entry<= 1'b1;
                        rmax_r   <= NEGINF;
                        p1_cnt   <= 20'd0;
                        stage    <= ST_NATIVE;
                    end
                end
                //-------------------------------------------- FIN（收尾判定）
                ST_FIN: begin
                    if (child_valid[0]) begin
                        status      <= 2'b01;
                        out_total   <= CORNER_N[15:0];
                        out_grid_ok <= 1'b1;
                        out_oc      <= 16'd0;
                        o_st        <= O_RD;
                        stage       <= ST_OUT;
                    end else begin
                        status <= 2'b10;
                        done   <= 1'b1;
                        busy   <= 1'b0;
                        stage  <= ST_IDLE;
                    end
                end
                //-------------------------------------------- OUT（输出 corner RAM_0）
                ST_OUT: begin
                    case (o_st)
                        O_IDLE: o_st <= O_RD;
                        O_RD:   o_st <= O_EMIT;
                        O_EMIT: begin
                            if (!out_valid) begin
                                out_valid <= 1'b1;
                                out_x     <= cram_rd_x[dl];
                                out_y     <= cram_rd_y[dl];
                            end else if (out_ready) begin
                                out_valid <= 1'b0;
                                if (out_oc + 16'd1 >= CORNER_N[15:0]) begin
                                    if (cfg_resp_dump_en) begin
                                        o_st  <= O_IDLE;
                                        stage <= ST_DUMP;
                                    end else begin
                                        done  <= 1'b1;
                                        busy  <= 1'b0;
                                        o_st  <= O_IDLE;
                                        stage <= ST_IDLE;
                                    end
                                end else begin
                                    out_oc <= out_oc + 16'd1;
                                    o_st   <= O_RD;
                                end
                            end
                        end
                        default: o_st <= O_RD;
                    endcase
                end
                //-------------------------------------------- DUMP（M7.3 帧级响应图导出）
                ST_DUMP: begin
                    if (resp_dump_done) begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        stage <= ST_IDLE;
                    end
                end
                default: stage <= ST_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------
    // 块 4：帧级响应图导出（ST_DUMP：读回最深层槽位 resp RAM 全图）
    //   dump_addr = d_addr（registered 读语义：地址须稳定 1 拍，下一拍出数据）
    //   时序：D_PRE 呈现地址 0（d_v=0）→ D_RUN 数据=mem[0] 且 d_v=1；
    //   接受后进 D_NXT 气泡拍（d_v=0、d_addr+1、呈现新地址），下一拍
    //   D_RUN 数据=mem[新地址]——**必须**隔拍推进：若在接受同拍推进，
    //   接受拍采到的是上一地址的旧数据，字重复/错位（曾导致角点簇写 0）。
    //   收满 D_PIX 个字 → resp_dump_done=1。
    //--------------------------------------------------------------------
    localparam D_IDLE = 3'd0, D_PRE = 3'd1, D_RUN = 3'd2, D_NXT = 3'd3, D_END = 3'd4;
    localparam [19:0] D_PIX = pix_of(DEPTH - 1);

    assign resp_dump_valid = d_v;
    assign resp_dump_data  = slot_dump_data[DEPTH - 1];

    always @(posedge clk) begin
        if (!rst_n) begin
            d_st           <= D_IDLE;
            d_addr         <= 20'd0;
            d_v            <= 1'b0;
            resp_dump_done <= 1'b0;
        end else if (stage != ST_DUMP) begin
            d_st           <= D_IDLE;
            d_addr         <= 20'd0;
            d_v            <= 1'b0;
            resp_dump_done <= 1'b0;
        end else begin
            case (d_st)
                D_IDLE: begin
                    d_addr <= 20'd0;
                    d_v    <= 1'b0;
                    d_st   <= D_PRE;
                end
                D_PRE: begin
                    d_v  <= 1'b1;
                    d_st <= D_RUN;
                end
                D_RUN: begin
                    if (d_v && resp_dump_ready) begin
                        if (d_addr >= D_PIX - 20'd1) begin
                            resp_dump_done <= 1'b1;
                            d_v            <= 1'b0;
                            d_st           <= D_END;
                        end else begin
                            d_v    <= 1'b0;
                            d_addr <= d_addr + 20'd1;
                            d_st   <= D_NXT;
                        end
                    end
                end
                D_NXT: begin
                    d_v  <= 1'b1;
                    d_st <= D_RUN;
                end
                D_END: begin
                    resp_dump_done <= 1'b1;
                    d_v            <= 1'b0;
                end
                default: d_st <= D_IDLE;
            endcase
        end
    end

endmodule
