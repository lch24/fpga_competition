`timescale 1ns / 1ps
//==============================================================================
// grid_order_ctrl.sv — M4 网格排序主控（复刻 organize_grid_det + cost_ref）
//------------------------------------------------------------------------------
// 从 M3 inner 点流组织 rows×cols 角点网格，输出行列序坐标。
// 位级对拍权威 = export_m4.cpp（确定性排序变体）。流程：
//   90 方向角度循环（cos/sin ROM，degree=-90+2*ak）：
//     A_PROJ   ：v = -x·si + y·co（f2o 单调 key）→ 填 keyv/idxv/vv
//     A_VSORT  ：index_sort 按 v 排序 → order[0..N)
//     A_GAP    ：gap = v(order[i+1])-v(order[i])；gapk=f2o(gap)，
//                gapi=(N-2)-i（同值位置小者排末尾 → 升序取尾）
//     A_GSORT  ：gap 排序取末尾 rows-1 个 → 恢复位置 gap_pos[]
//     A_GPSORT ：位置升序 → 行边界 gaps_arr[]（gaps_arr[rows-1]=N-1）
//     A_ROW    ：行循环：
//                 R_RSORT：段 [begin,end) 按 u = x·co + y·si 排序（index_sort）
//                           排序流同时写回 order[begin+c2] 并复制 win_order[]
//                 R_WIN  ：连续 COLS 点窗口枚举：
//                           W_STEP：7 间距 hypot(x[c]-x[c-1], y[c]-y[c-1])
//                           W_MED ：index_sort n=7 取第 4 个 → spacing；
//                                   spacing<4 跳过（W_SKIP 推进）
//                           W_SCORE：Σ((step-spacing)/spacing)² → 选最小
//                 R_GCPY ：grid[r*COLS+c] = pts[win_order[start_best+c]]
//                 R_NEXT ：行推进
//     A_COST   ：grid_validate（cost_ref 复刻）→ cost
//     A_BEST   ：cost < best_cost → best ← grid（40 点）+ 记录
//     A_ORIGIN ：4 角 x+y 最小定 origin；buf 重排写回 best/buf
//     A_OUT    ：过渡
//     A_NEXT   ：ak 推进 / angle_finish（90 角度完）
//   原点规范化后 S_DONE 从 best_buf 输出 40 点流。
//
// 存储：pts/keyv/idxv/vv/order/gapk/gapi/keyu/idxu/grid/best 用 dual_port_ram
//   （1 拍延迟读）；gaps_arr/gap_pos/win_order/wstep_r/wmed_key/best_buf 用 reg。
// 子模块：index_sort（排序）、grid_validate（代价）、fp32 算术（mul/add/sub/
//   div/hypot）。
//
// 时序关键：
//   - dual_port_ram 读延迟 1 拍：请求拍 → 下一拍数据组合可见。
//   - fp32 算术：fire 拍接受，2 拍后 out_valid。
//   - index_sort 的 rd_en/rd_addr 为其输出（wire），直接驱动 key/idx RAM 读口；
//     主控只喂 s_start/s_n，收 out 流（st_out_valid/st_out_ready/st_out_idx）。
//   - grid_validate 的 rd_en/rd_addr 为其输出（wire），经 gv_busy 多路选通
//     u_grid 读口（A_COST 期间归 gv，其余归主控 A_BEST 复制用）。
//   - 输出流 out_x/out_y 为组合（best_buf[oc_idx2]），out_valid 为 reg。
//==============================================================================
module grid_order_ctrl #(
    parameter ROWS        = 5,
    parameter COLS        = 8,
    parameter N_ADDR_W    = 8,          // 点容量 256
    parameter GAP_ADDR_W  = 8,
    parameter ROM_FILE    = "../tests/build/vectors/grid_cos_sin.mem"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    output reg  [1:0]              status,         //01=成功 10=点不足/失败
    // inner 点流（fp32 坐标）
    input  wire                    pts_valid,
    output reg                     pts_ready,
    input  wire [31:0]             pts_x,
    input  wire [31:0]             pts_y,
    input  wire                    pts_done,       // 点流结束
    // 输出：行列序角点（fp32）
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_x,
    output reg  [31:0]             out_y,
    output reg  [15:0]             out_total,
    output reg                     out_grid_ok
);

    localparam NCELL = ROWS * COLS;                 // 40
    localparam C_1E30F = 32'h7149f2ca;              // 1e30f
    localparam C_4    = 32'h40800000;               // 4.0f

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE  = 2'd0, S_CAP  = 2'd1, S_ANGLE = 2'd2, S_DONE = 2'd3;
    reg [1:0] state;

    localparam A_PROJ=0, A_VSORT=1, A_GAP=2, A_GSORT=3, A_GPSORT=4,
               A_ROW=5, A_COST=6, A_BEST=7, A_ORIGIN=8, A_OUT=9, A_NEXT=10;
    reg [3:0] ast;

    localparam R_RSORT=0, R_WIN=1, R_GCPY=2, R_NEXT=3;
    reg [1:0] rst;

    localparam W_STEP=0, W_MED=1, W_SCORE=2, W_SKIP=3;
    reg [1:0] wst;

    //--------------------------------------------------------------------
    // 控制寄存器（块 A：顶层 state）
    //--------------------------------------------------------------------
    reg [15:0] cnt_pts;
    reg        go_angle;              // S_CAP→S_ANGLE 时通知块 B 初始化（脉冲）
    reg [15:0] oc_cnt, oc_idx2;       // S_DONE 输出流计数

    //--------------------------------------------------------------------
    // 控制寄存器（块 B：角度 ast）
    //--------------------------------------------------------------------
    reg        run_angle;             // go_angle 后置 1，直到下一次 go_angle
    reg        angle_finish;          // 90 角度完（块 A 读）
    reg [7:0]  ak;                    // 角度索引 0..89（degree=-90+2*ak）
    reg [15:0] ai;                    // 投影/gap 循环索引
    reg [15:0] N_reg;                 // 点数（块 A 写、块 B 读）
    reg [15:0] r, s, c;               // 行号 / 窗口 start / 列
    reg [15:0] begin_idx, end_idx, seg_len;
    reg [31:0] best_cost, cur_cost;
    reg [31:0] row_best, cur_spacing, score_acc;
    reg [15:0] start_best;
    reg        found_best, grid_full;
    reg [1:0]  origin_k;
    reg [3:0]  pp;                    // 投影相位
    reg [15:0] ov_addr;               // order 写指针（v 排序）
    reg [15:0] gr_cnt;                // gap 步进
    reg [15:0] gcnt;                  // gsort 收集计数
    reg [15:0] gpos_cnt;              // gpsort 收集计数
    reg [15:0] ku_addr;               // keyu/idxu 写指针
    reg [15:0] ks_cnt;                // R_RSORT 步进（0..7）
    reg [15:0] on_cnt;                // order 覆盖写指针（行 u 排序流）
    reg [3:0]  wc;                    // 窗口内步 c=1..COLS-1
    reg [3:0]  sd_cnt;                // W_STEP 步进
    reg [3:0]  med_cnt;               // W_MED 填充/收流计数
    reg [3:0]  sc_idx;                // W_SCORE 元素索引
    reg [2:0]  scp;                   // W_SCORE 步进（0..5）
    reg [3:0]  rc;                    // R_GCPY 步进 / A_ORIGIN 重排行
    reg [15:0] bc_cnt;                // A_BEST 复制计数 / A_ORIGIN 读计数
    reg [3:0]  oo;                    // A_BEST/A_ORIGIN 步进
    reg [3:0]  mc;                    // A_ORIGIN 4 角循环

    // 数据暂存
    reg [31:0] u1r;                   // x*co
    reg [31:0] px_reg, py_reg;        // 点 x/y 锁存
    reg [N_ADDR_W-1:0] pr_ord, po_ord;
    reg [31:0] gv_v1, gv_v2;          // gap 的 v1/v2（投影 -x*si 复用 gv_v1）
    reg [31:0] dxr, dyr;              // 窗口 dx/dy
    reg [31:0] gv_gap;
    reg [31:0] sum4 [0:3];            // 4 角 x+y

    // 数组
    reg [N_ADDR_W-1:0] gap_pos  [0:15];
    reg [N_ADDR_W-1:0] gaps_arr [0:15];
    reg [N_ADDR_W-1:0] win_order [0:255];
    reg [31:0] wstep_r [0:COLS-2];
    reg [31:0] wmed_key [0:7];
    reg [63:0] best_buf [0:63];       // {y,x}

    //--------------------------------------------------------------------
    // 子模块互连
    //--------------------------------------------------------------------
    wire [63:0] pts_rd64, grid_rd64, best_rd64;
    wire [31:0] keyv_rd, idxv_rd, vv_rd, order_rd, gapk_rd, gapi_rd;
    wire [31:0] keyu_rd, idxu_rd;
    wire        st_busy, st_done, st_out_valid;
    wire [N_ADDR_W-1:0] st_out_idx;
    wire        gv_busy, gv_done, gv_valid;
    wire [31:0] gv_cost;
    wire        mul_rdy, mul_v, add_rdy, add_v, sub_rdy, sub_v, hyp_rdy, hyp_v, dv_rdy, dv_v;
    wire [31:0] mul_r, add_r, sub_r, hyp_r, dv_r;

    // 存储（dual_port_ram 1 拍延迟读）
    reg pts_wr_en; reg [N_ADDR_W-1:0] pts_wr_addr; reg [31:0] pts_wr_x, pts_wr_y;
    reg pts_rd_en; reg [N_ADDR_W-1:0] pts_rd_addr;
    reg keyv_wr_en; reg [31:0] keyv_wr;
    reg idxv_wr_en;
    reg [N_ADDR_W-1:0] proj_wr;        // 投影写地址锁存（写口 posedge 采样时 ai 已递增）
    reg vv_wr_en; reg [31:0] vv_wr;
    reg vv_rd_en; reg [N_ADDR_W-1:0] vv_rd_addr;
    reg order_wr_en; reg [N_ADDR_W-1:0] order_wr_addr, order_wr;
    reg order_rd_en; reg [N_ADDR_W-1:0] order_rd_addr;
    reg gapk_wr_en; reg [31:0] gapk_wr;
    reg gapi_wr_en; reg [GAP_ADDR_W-1:0] gapi_wr;
    reg [GAP_ADDR_W-1:0] gap_wr;       // gap 写地址锁存
    reg keyu_wr_en; reg [31:0] keyu_wr;
    reg idxu_wr_en; reg [N_ADDR_W-1:0] idxu_wr;
    reg [N_ADDR_W-1:0] ku_wr;          // 行 u 写地址锁存
    reg grid_wr_en; reg [5:0] grid_wr_addr; reg [31:0] grid_wr_x, grid_wr_y;
    reg best_wr_en; reg [5:0] best_wr_addr; reg [31:0] best_wr_x, best_wr_y;
    reg best_rd_en; reg [5:0] best_rd_addr;
    reg grd_rd_en; reg [5:0] grd_rd_addr;   // A_BEST 复制用 grid 读口（gv 空闲时）

    wire [31:0] pts_rd_x = pts_rd64[31:0];
    wire [31:0] pts_rd_y = pts_rd64[63:32];
    wire [31:0] grid_rd_x = grid_rd64[31:0];
    wire [31:0] grid_rd_y = grid_rd64[63:32];
    wire [31:0] best_rd_x = best_rd64[31:0];
    wire [31:0] best_rd_y = best_rd64[63:32];

    // index_sort 输入（rd_en/rd_addr 是它的输出，wire）
    reg s_start; reg [15:0] s_n;
    reg s_out_ready;
    wire [N_ADDR_W-1:0] s_rd_addr;
    wire s_rd_en;
    wire [31:0] s_rd_key;
    wire [N_ADDR_W-1:0] s_rd_idx;

    // grid_validate 输入（rd_en/rd_addr 是它的输出，wire）
    reg gv_start;
    wire [5:0] gv_rd_addr;

    // 算术 fire/输入
    reg mul_fire, add_fire, sub_fire, hyp_fire, dv_fire;
    reg [31:0] mul_a, mul_b, add_a, add_b, sub_a, sub_b, hyp_a, hyp_b, dv_a, dv_b;

    // ROM cos/sin（行 2k=cos、2k+1=sin，k ↔ degree=-90+2k）
    reg [31:0] csm [0:179];
    initial $readmemh(ROM_FILE, csm);

    //--------------------------------------------------------------------
    // 存储例化
    //--------------------------------------------------------------------
    dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(N_ADDR_W)) u_pts (
        .clk(clk), .rst_n(rst_n),
        .wr_en(pts_wr_en), .wr_addr(pts_wr_addr), .wr_data({pts_wr_y, pts_wr_x}),
        .rd_en(pts_rd_en), .rd_addr(pts_rd_addr), .rd_data(pts_rd64)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_ADDR_W)) u_keyv (
        .clk(clk), .rst_n(rst_n),
        .wr_en(keyv_wr_en), .wr_addr(proj_wr[N_ADDR_W-1:0]), .wr_data(keyv_wr),
        .rd_en(s_rd_en && (ast == A_VSORT)), .rd_addr(s_rd_addr), .rd_data(keyv_rd)
    );
    dual_port_ram #(.DATA_WIDTH(N_ADDR_W), .ADDR_WIDTH(N_ADDR_W)) u_idxv (
        .clk(clk), .rst_n(rst_n),
        .wr_en(idxv_wr_en), .wr_addr(proj_wr[N_ADDR_W-1:0]), .wr_data(proj_wr[N_ADDR_W-1:0]),
        .rd_en(s_rd_en && (ast == A_VSORT)), .rd_addr(s_rd_addr), .rd_data(idxv_rd)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_ADDR_W)) u_vv (
        .clk(clk), .rst_n(rst_n),
        .wr_en(vv_wr_en), .wr_addr(proj_wr[N_ADDR_W-1:0]), .wr_data(vv_wr),
        .rd_en(vv_rd_en), .rd_addr(vv_rd_addr), .rd_data(vv_rd)
    );
    dual_port_ram #(.DATA_WIDTH(N_ADDR_W), .ADDR_WIDTH(N_ADDR_W)) u_order (
        .clk(clk), .rst_n(rst_n),
        .wr_en(order_wr_en), .wr_addr(order_wr_addr), .wr_data(order_wr),
        .rd_en(order_rd_en), .rd_addr(order_rd_addr), .rd_data(order_rd)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(GAP_ADDR_W)) u_gapk (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gapk_wr_en), .wr_addr(gap_wr[GAP_ADDR_W-1:0]), .wr_data(gapk_wr),
        .rd_en(s_rd_en && (ast == A_GSORT)), .rd_addr(s_rd_addr), .rd_data(gapk_rd)
    );
    dual_port_ram #(.DATA_WIDTH(GAP_ADDR_W), .ADDR_WIDTH(GAP_ADDR_W)) u_gapi (
        .clk(clk), .rst_n(rst_n),
        .wr_en(gapi_wr_en), .wr_addr(gap_wr[GAP_ADDR_W-1:0]), .wr_data(gapi_wr),
        .rd_en(s_rd_en && (ast == A_GSORT)), .rd_addr(s_rd_addr), .rd_data(gapi_rd)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_ADDR_W)) u_keyu (
        .clk(clk), .rst_n(rst_n),
        .wr_en(keyu_wr_en), .wr_addr(ku_wr[N_ADDR_W-1:0]), .wr_data(keyu_wr),
        .rd_en(s_rd_en && (ast == A_ROW) && (rst == R_RSORT)), .rd_addr(s_rd_addr), .rd_data(keyu_rd)
    );
    dual_port_ram #(.DATA_WIDTH(N_ADDR_W), .ADDR_WIDTH(N_ADDR_W)) u_idxu (
        .clk(clk), .rst_n(rst_n),
        .wr_en(idxu_wr_en), .wr_addr(ku_wr[N_ADDR_W-1:0]), .wr_data(idxu_wr),
        .rd_en(s_rd_en && (ast == A_ROW) && (rst == R_RSORT)), .rd_addr(s_rd_addr), .rd_data(idxu_rd)
    );
    dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(6)) u_grid (
        .clk(clk), .rst_n(rst_n),
        .wr_en(grid_wr_en), .wr_addr(grid_wr_addr), .wr_data({grid_wr_y, grid_wr_x}),
        .rd_en(gv_busy ? gv_rd_en : grd_rd_en), .rd_addr(gv_busy ? gv_rd_addr : grd_rd_addr),
        .rd_data(grid_rd64)
    );
    dual_port_ram #(.DATA_WIDTH(64), .ADDR_WIDTH(6)) u_best (
        .clk(clk), .rst_n(rst_n),
        .wr_en(best_wr_en), .wr_addr(best_wr_addr), .wr_data({best_wr_y, best_wr_x}),
        .rd_en(best_rd_en), .rd_addr(best_rd_addr), .rd_data(best_rd64)
    );

    //--------------------------------------------------------------------
    // 算术例化（mul/add/sub/hypot/div）
    //--------------------------------------------------------------------
    fp32_mul u_mul (
        .clk(clk), .rst_n(rst_n),
        .in_valid(mul_fire), .in_ready(mul_rdy),
        .in_a(mul_a), .in_b(mul_b),
        .out_valid(mul_v), .out_ready(1'b1), .out_r(mul_r)
    );
    fp32_add u_add (
        .clk(clk), .rst_n(rst_n),
        .in_valid(add_fire), .in_ready(add_rdy),
        .in_a(add_a), .in_b(add_b),
        .out_valid(add_v), .out_ready(1'b1), .out_r(add_r)
    );
    fp32_sub u_sub (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_fire), .in_ready(sub_rdy),
        .in_a(sub_a), .in_b(sub_b),
        .out_valid(sub_v), .out_ready(1'b1), .out_r(sub_r)
    );
    fp32_hypot u_hyp (
        .clk(clk), .rst_n(rst_n),
        .in_valid(hyp_fire), .in_ready(hyp_rdy),
        .in_a(hyp_a), .in_b(hyp_b),
        .out_valid(hyp_v), .out_ready(1'b1), .out_r(hyp_r)
    );
    fp32_div u_div (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dv_fire), .in_ready(dv_rdy),
        .in_a(dv_a), .in_b(dv_b),
        .out_valid(dv_v), .out_ready(1'b1), .out_r(dv_r)
    );

    index_sort #(.N_ADDR_W(N_ADDR_W)) u_sort (
        .clk(clk), .rst_n(rst_n),
        .start(s_start), .busy(st_busy), .done(st_done),
        .n_in(s_n),
        .rd_en(s_rd_en), .rd_addr(s_rd_addr),
        .rd_key(s_rd_key), .rd_idx(s_rd_idx),
        .out_valid(st_out_valid), .out_ready(s_out_ready), .out_idx(st_out_idx)
    );
    grid_validate #(.ROWS(ROWS), .COLS(COLS), .N_ADDR_W(6)) u_gv (
        .clk(clk), .rst_n(rst_n),
        .start(gv_start), .busy(gv_busy), .done(gv_done),
        .n_in(16'd40),
        .rd_en(gv_rd_en), .rd_addr(gv_rd_addr), .rd_x(grid_rd_x), .rd_y(grid_rd_y),
        .valid_out(gv_valid), .cost_out(gv_cost)
    );

    // gap_pos/wmed_key 延迟读寄存器（A_GPSORT/W_MED 排序 key/idx；请求拍锁存，
    // 模拟 dual_port_ram 1 拍读延迟——index_sort 期望请求拍+1 数据有效）
    reg [31:0] gpos_kd;
    reg [N_ADDR_W-1:0] gpos_id;
    reg [31:0] wmed_kd;
    reg [N_ADDR_W-1:0] wmed_id;

    // sort key/idx 多路（v 排序 / gap 排序 / 位置排序 / 行 u 排序 / 中位数）
    assign s_rd_key = (ast == A_VSORT)  ? keyv_rd :
                      (ast == A_GSORT)  ? gapk_rd :
                      (ast == A_GPSORT) ? gpos_kd :
                      ((ast == A_ROW) && (rst == R_RSORT)) ? keyu_rd :
                      ((ast == A_ROW) && (rst == R_WIN) && (wst == W_MED)) ? wmed_kd :
                      32'd0;
    assign s_rd_idx = (ast == A_VSORT)  ? idxv_rd :
                      (ast == A_GSORT)  ? gapi_rd :
                      (ast == A_GPSORT) ? gpos_id :
                      ((ast == A_ROW) && (rst == R_RSORT)) ? idxu_rd :
                      ((ast == A_ROW) && (rst == R_WIN) && (wst == W_MED)) ? wmed_id :
                      {N_ADDR_W{1'b0}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpos_kd <= 32'd0; gpos_id <= {N_ADDR_W{1'b0}};
        end else if (s_rd_en) begin
            gpos_kd <= {{24{1'b0}}, gap_pos[s_rd_addr[3:0]]};
            gpos_id <= gap_pos[s_rd_addr[3:0]];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wmed_kd <= 32'd0; wmed_id <= {N_ADDR_W{1'b0}};
        end else if (s_rd_en) begin
            wmed_kd <= wmed_key[s_rd_addr[2:0]];
            wmed_id <= s_rd_addr[2:0];
        end
    end

    //--------------------------------------------------------------------
    // f2o 组合
    //--------------------------------------------------------------------
    function automatic [31:0] f2o_f(input [31:0] v);
        f2o_f = v[31] ? ~v : (v | 32'h80000000);
    endfunction

    // fp32 有符号数值比较 a < b（含符号，无 NaN）
    function automatic logic fp_lt(input logic [31:0] a, input logic [31:0] b);
        if (a[31] && !b[31])      fp_lt = 1'b1;
        else if (!a[31] && b[31]) fp_lt = 1'b0;
        else if (a[31])           fp_lt = (a > b);
        else                      fp_lt = (a < b);
    endfunction

    // 4 角索引（corner_ids = {0, COLS-1, (ROWS-1)*COLS, ROWS*COLS-1}）
    function automatic [5:0] cid_f(input [3:0] k);
        case (k)
            4'd0: cid_f = 6'd0;
            4'd1: cid_f = COLS - 6'd1;
            4'd2: cid_f = (ROWS - 6'd1) * COLS;
            default: cid_f = ROWS * COLS - 6'd1;
        endcase
    endfunction

    //--------------------------------------------------------------------
    // 块 A：顶层主状态机（IDLE/CAP/ANGLE/DONE）
    //--------------------------------------------------------------------
    assign pts_ready = (state == S_CAP);
    assign pts_wr_en = pts_valid && pts_ready;
    assign pts_wr_addr = cnt_pts[N_ADDR_W-1:0];
    assign pts_wr_x = pts_x;
    assign pts_wr_y = pts_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; busy <= 1'b0; done <= 1'b0; status <= 2'b00;
            cnt_pts <= 16'd0; go_angle <= 1'b0; N_reg <= 16'd0;
            oc_cnt <= 16'd0; oc_idx2 <= 16'd0;
            out_total <= 16'd0; out_grid_ok <= 1'b0;
        end else begin
            go_angle <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    busy <= 1'b1; done <= 1'b0; status <= 2'b00;
                    cnt_pts <= 16'd0; out_grid_ok <= 1'b0;
                    state <= S_CAP;
                end
                S_CAP: begin
                    if (pts_valid && pts_ready)
                        cnt_pts <= cnt_pts + 16'd1;
                    if (cnt_pts > 16'd255) begin
                        status <= 2'b10; done <= 1'b1; busy <= 1'b0; state <= S_DONE;
                    end else if (pts_done) begin
                        N_reg <= cnt_pts + (pts_valid ? 16'd1 : 16'd0);
                        if (cnt_pts + (pts_valid ? 16'd1 : 16'd0) < NCELL[15:0]) begin
                            status <= 2'b10; done <= 1'b1; busy <= 1'b0; state <= S_DONE;
                        end else begin
                            go_angle <= 1'b1;
                            state <= S_ANGLE;
                        end
                    end
                end
                S_ANGLE: begin
                    if (angle_finish) begin
                        if (!found_best) begin
                            status <= 2'b10; done <= 1'b1; busy <= 1'b0; state <= S_DONE;
                        end else begin
                            status <= 2'b01;
                            out_grid_ok <= 1'b1;
                            out_total <= NCELL[15:0];
                            oc_cnt <= 16'd0; oc_idx2 <= 16'd0;
                            done <= 1'b1; busy <= 1'b0;
                            state <= S_DONE;
                        end
                    end
                end
                S_DONE: begin
                    if (oc_cnt < NCELL[15:0]) begin
                        if (out_valid && out_ready) begin
                            oc_cnt <= oc_cnt + 16'd1;
                            oc_idx2 <= oc_idx2 + 16'd1;
                        end
                    end
                    if (start) begin
                        out_valid <= 1'b0;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // S_DONE 输出：out_valid 与 out_x/out_y
    // out_x/out_y 组合直读 best_buf[oc_idx2]（重排后最终序），与握手同步无滞后。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
        end else begin
            if (state == S_DONE && out_grid_ok && (oc_cnt < NCELL[15:0]))
                out_valid <= 1'b1;
            else
                out_valid <= 1'b0;
        end
    end
    always_comb begin
        out_x = best_buf[oc_idx2[5:0]][31:0];
        out_y = best_buf[oc_idx2[5:0]][63:32];
    end

    // 原点规范化源索引（A_ORIGIN 重排用）
    wire [15:0] src_idx_c =
        ((origin_k[1] ? ROWS[15:0] - 1 - rc : rc) * COLS[15:0]) +
        (origin_k[0] ? COLS[15:0] - 1 - c : c);
    wire [63:0] src_buf_d = best_buf[src_idx_c[5:0]];

    //--------------------------------------------------------------------
    // 块 B：角度子状态机（A_PROJ…A_NEXT）
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_angle <= 1'b0; angle_finish <= 1'b0;
            ast <= A_PROJ; rst <= R_RSORT; wst <= W_STEP;
            ak <= 8'd0; ai <= 16'd0;
            pp <= 4'd0; ov_addr <= 16'd0; gr_cnt <= 16'd0; gcnt <= 16'd0;
            gpos_cnt <= 16'd0; ku_addr <= 16'd0; ks_cnt <= 16'd0; on_cnt <= 16'd0;
            r <= 16'd0; s <= 16'd0; c <= 16'd0;
            begin_idx <= 16'd0; end_idx <= 16'd0; seg_len <= 16'd0;
            best_cost <= 32'd0; cur_cost <= 32'd0;
            row_best <= 32'd0; cur_spacing <= 32'd0; score_acc <= 32'd0;
            start_best <= 16'd0;
            found_best <= 1'b0; grid_full <= 1'b0;
            origin_k <= 2'd0;
            wc <= 4'd0; sd_cnt <= 4'd0; med_cnt <= 4'd0; sc_idx <= 4'd0;
            scp <= 2'd0; rc <= 4'd0; bc_cnt <= 16'd0; oo <= 4'd0; mc <= 4'd0;
            u1r <= 32'd0; px_reg <= 32'd0; py_reg <= 32'd0;
            pr_ord <= 0; po_ord <= 0;
            gv_v1 <= 32'd0; gv_v2 <= 32'd0; gv_gap <= 32'd0;
            dxr <= 32'd0; dyr <= 32'd0;
            pts_rd_en <= 1'b0; pts_rd_addr <= 0;
            vv_rd_en <= 1'b0; vv_rd_addr <= 0;
            order_rd_en <= 1'b0; order_rd_addr <= 0;
            order_wr_en <= 1'b0; order_wr_addr <= 0; order_wr <= 0;
            keyv_wr_en <= 1'b0; idxv_wr_en <= 1'b0; vv_wr_en <= 1'b0;
            proj_wr <= 0;
            gapk_wr_en <= 1'b0; gapi_wr_en <= 1'b0; gap_wr <= 0;
            keyu_wr_en <= 1'b0; idxu_wr_en <= 1'b0; ku_wr <= 0;
            grid_wr_en <= 1'b0; grid_wr_addr <= 0; grid_wr_x <= 32'd0; grid_wr_y <= 32'd0;
            best_wr_en <= 1'b0; best_wr_addr <= 0; best_wr_x <= 32'd0; best_wr_y <= 32'd0;
            best_rd_en <= 1'b0; best_rd_addr <= 0;
            grd_rd_en <= 1'b0; grd_rd_addr <= 0;
            s_start <= 1'b0; s_n <= 16'd0; s_out_ready <= 1'b0;
            gv_start <= 1'b0;
            mul_fire <= 1'b0; add_fire <= 1'b0; sub_fire <= 1'b0;
            hyp_fire <= 1'b0; dv_fire <= 1'b0;
            mul_a <= 32'd0; mul_b <= 32'd0; add_a <= 32'd0; add_b <= 32'd0;
            sub_a <= 32'd0; sub_b <= 32'd0; hyp_a <= 32'd0; hyp_b <= 32'd0;
            dv_a <= 32'd0; dv_b <= 32'd0;
        end else begin
            // 默认清脉冲（s_out_ready 为电平握手信号，不在默认清之列）
            s_start <= 1'b0;
            gv_start <= 1'b0;
            mul_fire <= 1'b0; add_fire <= 1'b0; sub_fire <= 1'b0;
            hyp_fire <= 1'b0; dv_fire <= 1'b0;
            keyv_wr_en <= 1'b0; idxv_wr_en <= 1'b0; vv_wr_en <= 1'b0;
            order_wr_en <= 1'b0;
            gapk_wr_en <= 1'b0; gapi_wr_en <= 1'b0;
            keyu_wr_en <= 1'b0; idxu_wr_en <= 1'b0;
            grid_wr_en <= 1'b0;
            best_wr_en <= 1'b0;

            if (go_angle) begin
                // 新一轮初始化
                run_angle <= 1'b1; angle_finish <= 1'b0;
                ast <= A_PROJ; rst <= R_RSORT; wst <= W_STEP;
                ak <= 8'd0; ai <= 16'd0;
                pp <= 4'd0; ov_addr <= 16'd0; gr_cnt <= 16'd0; gcnt <= 16'd0;
                gpos_cnt <= 16'd0; ku_addr <= 16'd0; ks_cnt <= 16'd0; on_cnt <= 16'd0;
                r <= 16'd0; s <= 16'd0; c <= 16'd0;
                begin_idx <= 16'd0; end_idx <= 16'd0; seg_len <= 16'd0;
                best_cost <= C_1E30F; cur_cost <= 32'd0;
                row_best <= 32'd0; cur_spacing <= 32'd0; score_acc <= 32'd0;
                start_best <= 16'hFFFF;
                found_best <= 1'b0; grid_full <= 1'b0;
                origin_k <= 2'd0;
                wc <= 4'd0; sd_cnt <= 4'd0; med_cnt <= 4'd0; sc_idx <= 4'd0;
                scp <= 2'd0; rc <= 4'd0; bc_cnt <= 16'd0; oo <= 4'd0; mc <= 4'd0;
                pts_rd_en <= 1'b0; vv_rd_en <= 1'b0; order_rd_en <= 1'b0;
                order_wr_en <= 1'b0;
                best_rd_en <= 1'b0; grd_rd_en <= 1'b0;
                s_start <= 1'b0; s_out_ready <= 1'b0; gv_start <= 1'b0;
                mul_fire <= 1'b0; add_fire <= 1'b0; sub_fire <= 1'b0;
                hyp_fire <= 1'b0; dv_fire <= 1'b0;
            end else if (run_angle) begin
                case (ast)
                //============================================ A_PROJ
                A_PROJ: begin
                    if (ai >= N_reg) begin
                        // 投影完成 → v 排序
                        ai <= 16'd0;
                        s_n <= N_reg;
                        s_start <= 1'b1;
                        ov_addr <= 16'd0;
                        ast <= A_VSORT;
                    end else begin
                        case (pp)
                            4'd0: begin
                                pts_rd_en <= 1'b1;
                                pts_rd_addr <= ai[N_ADDR_W-1:0];
                                pp <= 4'd1;
                            end
                            4'd1: begin
                                // pts 数据在途（1 拍读延迟）
                                pp <= 4'd2;
                            end
                            4'd2: begin
                                px_reg <= pts_rd_x;
                                py_reg <= pts_rd_y;
                                mul_a <= pts_rd_x;
                                mul_b <= csm[2 * ak + 1];       // sin
                                mul_fire <= 1'b1;
                                pts_rd_en <= 1'b0;
                                pp <= 4'd3;
                            end
                            4'd3: begin
                                if (mul_v) begin
                                    gv_v1 <= {~mul_r[31], mul_r[30:0]};   // -x*si
                                    mul_a <= py_reg;                     // y
                                    mul_b <= csm[2 * ak];                // cos
                                    mul_fire <= 1'b1;
                                    pp <= 4'd4;
                                end
                            end
                            4'd4: begin
                                if (mul_v) begin
                                    add_a <= gv_v1;
                                    add_b <= mul_r;
                                    add_fire <= 1'b1;
                                    pp <= 4'd5;
                                end
                            end
                            4'd5: begin
                                if (add_v) begin
                                    vv_wr <= add_r;
                                    keyv_wr <= f2o_f(add_r);
                                    proj_wr <= ai[N_ADDR_W-1:0];
                                    vv_wr_en <= 1'b1; keyv_wr_en <= 1'b1;
                                    idxv_wr_en <= 1'b1;
                                    ai <= ai + 16'd1;
                                    pp <= 4'd0;
                                end
                            end
                            default: pp <= 4'd0;
                        endcase
                    end
                end
                //============================================ A_VSORT
                A_VSORT: begin
                    if (st_out_valid && s_out_ready) begin
                        order_wr_en <= 1'b1;
                        order_wr_addr <= ov_addr[N_ADDR_W-1:0];
                        order_wr <= st_out_idx;
                        ov_addr <= ov_addr + 16'd1;
                        s_out_ready <= 1'b0;
                    end else if (ov_addr < N_reg && !st_out_valid) begin
                        s_out_ready <= 1'b1;
                    end
                    if (st_done) begin
                        s_out_ready <= 1'b0;
                        gr_cnt <= 16'd0;
                        ai <= 16'd0;
                        ast <= A_GAP;
                    end
                end
                //============================================ A_GAP
                A_GAP: begin
                    if (ai >= N_reg - 1) begin
                        gcnt <= 16'd0;
                        gpos_cnt <= 16'd0;
                        s_n <= N_reg - 1;
                        s_start <= 1'b1;
                        ai <= 16'd0;
                        ast <= A_GSORT;
                    end else begin
                        case (gr_cnt)
                            16'd0: begin
                                order_rd_en <= 1'b1;
                                order_rd_addr <= ai[N_ADDR_W-1:0];
                                gr_cnt <= 16'd1;
                            end
                            16'd1: begin
                                // order[ai] 数据在途
                                gr_cnt <= 16'd2;
                            end
                            16'd2: begin
                                pr_ord <= order_rd;
                                order_rd_en <= 1'b1;
                                order_rd_addr <= ai + 16'd1;
                                gr_cnt <= 16'd3;
                            end
                            16'd3: begin
                                // order[ai+1] 数据在途
                                gr_cnt <= 16'd4;
                            end
                            16'd4: begin
                                po_ord <= order_rd;
                                vv_rd_en <= 1'b1;
                                vv_rd_addr <= pr_ord[N_ADDR_W-1:0];
                                gr_cnt <= 16'd5;
                            end
                            16'd5: begin
                                // vv[o1] 数据在途
                                gr_cnt <= 16'd6;
                            end
                            16'd6: begin
                                gv_v1 <= vv_rd;
                                vv_rd_en <= 1'b1;
                                vv_rd_addr <= po_ord[N_ADDR_W-1:0];
                                gr_cnt <= 16'd7;
                            end
                            16'd7: begin
                                // vv[o2] 数据在途
                                gr_cnt <= 16'd8;
                            end
                            16'd8: begin
                                gv_v2 <= vv_rd;
                                sub_a <= vv_rd;
                                sub_b <= gv_v1;
                                sub_fire <= 1'b1;
                                gr_cnt <= 16'd9;
                            end
                            16'd9: begin
                                if (sub_v) begin
                                    gv_gap <= sub_r;
                                    gapk_wr <= f2o_f(sub_r);
                                    gapi_wr <= N_reg - 2 - ai;
                                    gap_wr <= ai[GAP_ADDR_W-1:0];
                                    gapk_wr_en <= 1'b1; gapi_wr_en <= 1'b1;
                                    vv_rd_en <= 1'b0;
                                    order_rd_en <= 1'b0;
                                    ai <= ai + 16'd1;
                                    gr_cnt <= 16'd0;
                                end
                            end
                        endcase
                    end
                end
                //============================================ A_GSORT
                A_GSORT: begin
                    if (st_out_valid && s_out_ready) begin
                        // out_idx = gapi = (N-2)-pos；恢复 pos（取末尾 rows-1 个）
                        if (gcnt >= (N_reg - 1) - (ROWS[15:0] - 1)) begin
                            if (gpos_cnt < ROWS[15:0] - 1) begin
                                gap_pos[gpos_cnt] <= (N_reg - 2) - st_out_idx;
                                gpos_cnt <= gpos_cnt + 16'd1;
                            end
                        end
                        gcnt <= gcnt + 16'd1;
                        s_out_ready <= 1'b0;
                    end else if (!st_out_valid && gcnt < N_reg - 1) begin
                        s_out_ready <= 1'b1;
                    end
                    if (st_done) begin
                        s_out_ready <= 1'b0;
                        gpos_cnt <= 16'd0;
                        s_n <= ROWS - 1;
                        s_start <= 1'b1;
                        ai <= 16'd0;
                        ast <= A_GPSORT;
                    end
                end
                //============================================ A_GPSORT
                A_GPSORT: begin
                    if (st_out_valid && s_out_ready) begin
                        if (gpos_cnt < ROWS[15:0]) begin
                            gaps_arr[gpos_cnt] <= st_out_idx;
                            gpos_cnt <= gpos_cnt + 16'd1;
                        end
                        s_out_ready <= 1'b0;
                    end else if (!st_out_valid && gpos_cnt < ROWS[15:0]) begin
                        s_out_ready <= 1'b1;
                    end
                    if (st_done) begin
                        s_out_ready <= 1'b0;
                        gaps_arr[ROWS-1] <= N_reg - 1;
                        r <= 16'd0; begin_idx <= 16'd0;
                        grid_full <= 1'b0;
                        ks_cnt <= 16'd0;
                        rst <= R_RSORT;
                        ast <= A_ROW;
                    end
                end
                //============================================ A_ROW
                A_ROW: begin
                    case (rst)
                    //---------------------------------------- R_RSORT
                    R_RSORT: begin
                        case (ks_cnt)
                            16'd0: begin
                                end_idx <= gaps_arr[r] + 16'd1;
                                ks_cnt <= 16'd1;
                            end
                            16'd1: begin
                                seg_len <= end_idx - begin_idx;
                                if (end_idx - begin_idx < COLS[15:0]) begin
                                    // 段长不足：行循环中止 → 废弃本角度
                                    ast <= A_NEXT;
                                end else begin
                                    order_rd_en <= 1'b1;
                                    order_rd_addr <= begin_idx[N_ADDR_W-1:0];
                                    ku_addr <= 16'd0;
                                    ks_cnt <= 16'd2;
                                end
                            end
                            16'd2: begin
                                // order[begin] 数据在途
                                ks_cnt <= 16'd3;
                            end
                            16'd3: begin
                                pr_ord <= order_rd;
                                pts_rd_en <= 1'b1;
                                pts_rd_addr <= order_rd[N_ADDR_W-1:0];
                                order_rd_en <= 1'b0;
                                ks_cnt <= 16'd4;
                            end
                            16'd4: begin
                                // pts[ord] 数据在途
                                ks_cnt <= 16'd5;
                            end
                            16'd5: begin
                                px_reg <= pts_rd_x;
                                py_reg <= pts_rd_y;
                                mul_a <= pts_rd_x;
                                mul_b <= csm[2 * ak];            // cos
                                mul_fire <= 1'b1;
                                pts_rd_en <= 1'b0;
                                ks_cnt <= 16'd6;
                            end
                            16'd6: begin
                                if (mul_v) begin
                                    u1r <= mul_r;
                                    mul_a <= py_reg;
                                    mul_b <= csm[2 * ak + 1];    // sin
                                    mul_fire <= 1'b1;
                                    ks_cnt <= 16'd7;
                                end
                            end
                            16'd7: begin
                                if (mul_v) begin
                                    add_a <= u1r;
                                    add_b <= mul_r;
                                    add_fire <= 1'b1;
                                    ks_cnt <= 16'd8;
                                end
                            end
                            16'd8: begin
                                if (add_v) begin
                                    keyu_wr <= f2o_f(add_r);
                                    idxu_wr <= pr_ord;
                                    ku_wr <= ku_addr[N_ADDR_W-1:0];
                                    keyu_wr_en <= 1'b1;
                                    idxu_wr_en <= 1'b1;
                                    if (ku_addr + 1 >= seg_len) begin
                                        // 段内 u 全部算完 → 触发排序
                                        s_n <= seg_len;
                                        s_start <= 1'b1;
                                        on_cnt <= 16'd0;
                                        ks_cnt <= 16'd9;
                                    end else begin
                                        ku_addr <= ku_addr + 16'd1;
                                        order_rd_en <= 1'b1;
                                        order_rd_addr <= begin_idx + ku_addr + 16'd1;
                                        ks_cnt <= 16'd2;
                                    end
                                end
                            end
                            16'd9: begin
                                // 收 index_sort 流：写回 order 段 + win_order 副本
                                if (st_out_valid && s_out_ready) begin
                                    order_wr_en <= 1'b1;
                                    order_wr_addr <= begin_idx + on_cnt;
                                    order_wr <= st_out_idx;
                                    win_order[on_cnt] <= st_out_idx;
                                    on_cnt <= on_cnt + 16'd1;
                                    s_out_ready <= 1'b0;
                                end else if (!st_out_valid && on_cnt < seg_len) begin
                                    s_out_ready <= 1'b1;
                                end
                                if (st_done) begin
                                    s_out_ready <= 1'b0;
                                    // 进入窗口枚举
                                    s <= begin_idx;
                                    row_best <= C_1E30F;
                                    start_best <= 16'hFFFF;
                                    wc <= 4'd1;
                                    sd_cnt <= 4'd0;
                                    wst <= W_STEP;
                                    rst <= R_WIN;
                                end
                            end
                        endcase
                    end
                    //---------------------------------------- R_WIN
                    R_WIN: begin
                        case (wst)
                        //------------------------------ W_STEP
                        W_STEP: begin
                            case (sd_cnt)
                                4'd0: begin
                                    pts_rd_en <= 1'b1;
                                    pts_rd_addr <= win_order[s - begin_idx + wc];
                                    sd_cnt <= 4'd1;
                                end
                                4'd1: begin
                                    // 当前点数据在途
                                    sd_cnt <= 4'd2;
                                end
                                4'd2: begin
                                    px_reg <= pts_rd_x;   // 当前点 c
                                    py_reg <= pts_rd_y;
                                    pts_rd_en <= 1'b1;
                                    pts_rd_addr <= win_order[s - begin_idx + wc - 4'd1];
                                    sd_cnt <= 4'd3;
                                end
                                4'd3: begin
                                    // 前点数据在途
                                    sd_cnt <= 4'd4;
                                end
                                4'd4: begin
                                    // 前点 c-1 数据到：dx = x[c]-x[c-1]
                                    sub_a <= px_reg;
                                    sub_b <= pts_rd_x;
                                    sub_fire <= 1'b1;
                                    pts_rd_en <= 1'b0;
                                    sd_cnt <= 4'd5;
                                end
                                4'd5: begin
                                    if (sub_v) begin
                                        dxr <= sub_r;
                                        sub_a <= py_reg;
                                        sub_b <= pts_rd_y;   // dy = y[c]-y[c-1]
                                        sub_fire <= 1'b1;
                                        sd_cnt <= 4'd6;
                                    end
                                end
                                4'd6: begin
                                    if (sub_v) begin
                                        dyr <= sub_r;
                                        hyp_a <= dxr;
                                        hyp_b <= sub_r;
                                        hyp_fire <= 1'b1;
                                        sd_cnt <= 4'd7;
                                    end
                                end
                                4'd7: begin
                                    if (hyp_v) begin
                                        wstep_r[wc - 4'd1] <= hyp_r;
                                        if (wc + 4'd1 > COLS[3:0] - 4'd1) begin
                                            // 7 步完成 → 中位数
                                            med_cnt <= 4'd0;
                                            wst <= W_MED;
                                        end else begin
                                            wc <= wc + 4'd1;
                                            sd_cnt <= 4'd0;
                                        end
                                    end
                                end
                            endcase
                        end
                        //------------------------------ W_MED
                        W_MED: begin
                            if (med_cnt < 4'd7) begin
                                wmed_key[med_cnt] <= f2o_f(wstep_r[med_cnt]);
                                med_cnt <= med_cnt + 4'd1;
                            end else if (med_cnt == 4'd7) begin
                                s_n <= 16'd7;
                                s_start <= 1'b1;
                                med_cnt <= 4'd8;
                            end else begin
                                // 收流：第 4 个输出 = 中位数索引
                                if (st_out_valid && s_out_ready) begin
                                    if (med_cnt == 4'd11)
                                        cur_spacing <= wstep_r[st_out_idx[2:0]];
                                    med_cnt <= med_cnt + 4'd1;
                                    s_out_ready <= 1'b0;
                                end else if (!st_out_valid && med_cnt < 4'd15) begin
                                    s_out_ready <= 1'b1;
                                end
                                if (st_done) begin
                                    s_out_ready <= 1'b0;
                                    if (fp_lt(cur_spacing, C_4)) begin
                                        // spacing<4：跳过本窗口
                                        wst <= W_SKIP;
                                    end else begin
                                        score_acc <= 32'd0;
                                        sc_idx <= 4'd0;
                                        scp <= 2'd0;
                                        wst <= W_SCORE;
                                    end
                                end
                            end
                        end
                        //------------------------------ W_SCORE
                        W_SCORE: begin
                            case (scp)
                                3'd0: begin
                                    sub_a <= wstep_r[sc_idx];
                                    sub_b <= cur_spacing;
                                    sub_fire <= 1'b1;
                                    scp <= 3'd1;
                                end
                                3'd1: begin
                                    if (sub_v) begin
                                        dv_a <= sub_r;
                                        dv_b <= cur_spacing;
                                        dv_fire <= 1'b1;
                                        scp <= 3'd2;
                                    end
                                end
                                3'd2: begin
                                    if (dv_v) begin
                                        mul_a <= dv_r;
                                        mul_b <= dv_r;
                                        mul_fire <= 1'b1;
                                        scp <= 3'd3;
                                    end
                                end
                                3'd3: begin
                                    if (mul_v) begin
                                        add_a <= score_acc;
                                        add_b <= mul_r;
                                        add_fire <= 1'b1;
                                        scp <= 3'd4;
                                    end
                                end
                                3'd4: begin
                                    if (add_v) begin
                                        score_acc <= add_r;
                                        if (sc_idx + 4'd1 >= COLS[3:0] - 4'd1) begin
                                            // 7 个元素完成：比较更新
                                            if (fp_lt(add_r, row_best)) begin
                                                row_best <= add_r;
                                                start_best <= s;
                                            end
                                            scp <= 3'd5;
                                        end else begin
                                            sc_idx <= sc_idx + 4'd1;
                                            scp <= 3'd0;
                                        end
                                    end
                                end
                                3'd5: begin
                                    // 窗口推进（start_best 已更新）
                                    if (s + COLS[15:0] + 16'd1 > end_idx) begin
                                        if (start_best == 16'hFFFF) begin
                                            ast <= A_NEXT;      // 行中止
                                        end else begin
                                            rc <= 4'd0; c <= 16'd0;
                                            rst <= R_GCPY;
                                        end
                                    end else begin
                                        s <= s + 16'd1;
                                        wc <= 4'd1;
                                        sd_cnt <= 4'd0;
                                        wst <= W_STEP;
                                    end
                                end
                            endcase
                        end
                        //------------------------------ W_SKIP
                        W_SKIP: begin
                            // spacing<4 跳过（start_best 未更新）
                            if (s + COLS[15:0] + 16'd1 > end_idx) begin
                                if (start_best == 16'hFFFF) begin
                                    ast <= A_NEXT;              // 行中止
                                end else begin
                                    rc <= 4'd0; c <= 16'd0;
                                    rst <= R_GCPY;
                                end
                            end else begin
                                s <= s + 16'd1;
                                wc <= 4'd1;
                                sd_cnt <= 4'd0;
                                wst <= W_STEP;
                            end
                        end
                        endcase
                    end
                    //---------------------------------------- R_GCPY
                    R_GCPY: begin
                        case (rc)
                            4'd0: begin
                                pts_rd_en <= 1'b1;
                                pts_rd_addr <= win_order[start_best - begin_idx + c];
                                rc <= 4'd1;
                            end
                            4'd1: begin
                                // pts 数据在途
                                rc <= 4'd2;
                            end
                            4'd2: begin
                                grid_wr_x <= pts_rd_x;
                                grid_wr_y <= pts_rd_y;
                                grid_wr_addr <= r * COLS + c;
                                grid_wr_en <= 1'b1;
                                pts_rd_en <= 1'b0;
                                if (c + 16'd1 >= COLS[15:0]) begin
                                    // 本行复制完
                                    if (r + 16'd1 >= ROWS[15:0]) begin
                                        grid_full <= 1'b1;
                                        ast <= A_COST;
                                    end else begin
                                        r <= r + 16'd1;
                                        begin_idx <= end_idx;
                                        ks_cnt <= 16'd0;
                                        rst <= R_NEXT;
                                    end
                                end else begin
                                    c <= c + 16'd1;
                                    rc <= 4'd0;
                                end
                            end
                        endcase
                    end
                    //---------------------------------------- R_NEXT
                    R_NEXT: begin
                        rst <= R_RSORT;
                    end
                    endcase
                end
                //============================================ A_COST
                A_COST: begin
                    if (!grid_full) begin
                        ast <= A_NEXT;
                    end else if (gv_done) begin
                        cur_cost <= gv_cost;
                        if (fp_lt(gv_cost, best_cost))
                            ast <= A_BEST;
                        else
                            ast <= A_NEXT;
                    end else if (!gv_busy) begin
                        gv_start <= 1'b1;
                    end
                end
                //============================================ A_BEST
                A_BEST: begin
                    if (bc_cnt < NCELL[15:0]) begin
                        case (oo)
                            4'd0: begin
                                grd_rd_en <= 1'b1;
                                grd_rd_addr <= bc_cnt[5:0];
                                oo <= 4'd1;
                            end
                            4'd1: begin
                                // grid 数据在途
                                oo <= 4'd2;
                            end
                            4'd2: begin
                                best_wr_x <= grid_rd64[31:0];
                                best_wr_y <= grid_rd64[63:32];
                                best_wr_addr <= bc_cnt[5:0];
                                best_wr_en <= 1'b1;
                                grd_rd_en <= 1'b0;
                                bc_cnt <= bc_cnt + 16'd1;
                                oo <= 4'd0;
                            end
                        endcase
                    end else begin
                        best_cost <= cur_cost;
                        found_best <= 1'b1;
                        bc_cnt <= 16'd0;
                        oo <= 4'd0;
                        ast <= A_ORIGIN;
                    end
                end
                //============================================ A_ORIGIN
                A_ORIGIN: begin
                    case (oo)
                        4'd0: begin
                            if (bc_cnt < NCELL[15:0]) begin
                                best_rd_en <= 1'b1;
                                best_rd_addr <= bc_cnt[5:0];
                                bc_cnt <= bc_cnt + 16'd1;
                                oo <= 4'd1;
                            end else begin
                                mc <= 4'd0;
                                oo <= 4'd2;
                            end
                        end
                        4'd1: begin
                            // best 数据在途
                            oo <= 4'd2;
                        end
                        4'd2: begin
                            best_buf[bc_cnt - 16'd1] <= best_rd64;
                            best_rd_en <= 1'b0;
                            if (bc_cnt >= NCELL[15:0]) begin
                                mc <= 4'd0;
                                oo <= 4'd3;
                            end else
                                oo <= 4'd0;
                        end
                        4'd3: begin
                            add_a <= best_buf[cid_f(mc)][31:0];
                            add_b <= best_buf[cid_f(mc)][63:32];
                            add_fire <= 1'b1;
                            oo <= 4'd4;
                        end
                        4'd4: begin
                            if (add_v) begin
                                sum4[mc] <= add_r;
                                mc <= mc + 4'd1;
                                if (mc + 4'd1 >= 4'd4) begin
                                    origin_k <= 2'd0;
                                    oo <= 4'd5;
                                end else
                                    oo <= 4'd3;
                            end
                        end
                        4'd5: begin
                            mc <= 4'd1;
                            oo <= 4'd6;
                        end
                        4'd6: begin
                            if (mc < 4'd4) begin
                                if (fp_lt(sum4[mc], sum4[origin_k]))
                                    origin_k <= mc;
                                mc <= mc + 4'd1;
                                if (mc + 4'd1 >= 4'd4)
                                    oo <= 4'd7;
                            end else
                                oo <= 4'd7;
                        end
                        4'd7: begin
                            rc <= 4'd0;
                            c <= 16'd0;
                            oo <= 4'd8;
                        end
                        4'd8: begin
                            // 重排：dst=r*COLS+c ← buf[src]（同时更新 buf 供输出）
                            best_wr_x <= src_buf_d[31:0];
                            best_wr_y <= src_buf_d[63:32];
                            best_wr_addr <= rc * COLS + c;
                            best_wr_en <= 1'b1;
                            best_buf[rc * COLS + c] <= src_buf_d;
                            if (c + 16'd1 >= COLS[15:0]) begin
                                c <= 16'd0;
                                rc <= rc + 4'd1;
                            end else
                                c <= c + 16'd1;
                            if (rc * COLS + c + 16'd1 >= NCELL[15:0])
                                oo <= 4'd9;
                        end
                        4'd9: begin
                            ast <= A_OUT;
                        end
                    endcase
                end
                //============================================ A_OUT
                A_OUT: begin
                    ast <= A_NEXT;
                end
                //============================================ A_NEXT
                A_NEXT: begin
                    if (ak >= 8'd89) begin
                        angle_finish <= 1'b1;
                    end else begin
                        ak <= ak + 8'd1;
                        ai <= 16'd0;
                        pp <= 4'd0;
                        ast <= A_PROJ;
                    end
                end
                default: ast <= A_NEXT;
                endcase
            end
        end
    end

endmodule
