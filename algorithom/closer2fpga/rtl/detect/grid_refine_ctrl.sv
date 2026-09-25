`timescale 1ns / 1ps
//==============================================================================
// grid_refine_ctrl.sv — M5 网格精定位（复刻 validation.cpp::refine_grid）
//------------------------------------------------------------------------------
// 流程：
//   1. min_step：遍历 40 点所有横/纵相邻点求最短边长（fp32_hypot，float）
//       横边：r∈[0,ROWS)、c∈[0,COLS-1)：dist(corners[r*COLS+c], [..+1])
//       纵边：r∈[0,ROWS-1)、c∈[0,COLS)：dist(corners[r*COLS+c], [..+COLS])
//   2. half_win = clamp(int(min_step * 0.15f), 2, 10)
//       （float 乘法 → 截断为 int → clamp；注意顺序与 C++ 一致）
//   3. refine_subpixel(gray, corners, half_win)：40 点亚像素更新
//       → 复用 subpixel_ctrl（rtl/detect/subpixel_ctrl.sv，M5.1 交付）
//   4. valid = grid_validate(corners) < 1e30f（复用 rtl/detect/grid_validate.sv）
//       → 无效则 corners 清空（valid=0，无输出流）
// 位级权威 = tests/rtl/export_m5.cpp（refine_grid_ref，M5.2 对拍）。
//
// 实现要点（与 C++ 位级一致）：
//   - 中间缓冲：40 点 {x,y} 双口 RAM（x/y 各一个 dual_port_ram，1 拍读延迟）。
//     写口：S_MINSTEP 装载（grid 输入）/ S_SUBPX 回写（subpixel 输出流）2 选 1；
//     读口：S_SUBPX（subpixel pt_rd）/ S_VALIDATE（grid_validate rd）/
//           S_OUT（输出流）3 选 1（阶段互斥）。
//   - S_MINSTEP：先装载 40 点（pt 读口 请求→转移→吸收 2 拍），再逐边
//     （共 ROWS*(COLS-1)+(ROWS-1)*COLS = 67 条）读两端点 → fp32_sub(dx/dy)
//     → fp32_hypot → 与 min_step 取 min（全正，无符号比较即可）。
//   - S_HALFWIN：fp32_mul(min_step, 0.15f) → fp32 截断 s32（正数=floor）
//     → clamp(2,10)。
//   - S_SUBPX：subpixel_ctrl（n_in=40，half_win 上述），其 pt/gray 读口
//     经本模块透传；out 流（x,y）写回中间 RAM（reliable 不看，C++ 语义
//     失败保留原值）→ done → S_VALIDATE。
//   - S_VALIDATE：grid_validate start（n_in=40，其 rd 口驱动中间 RAM）
//     → cost_out < 1e30f 判有效 → S_OUT / S_DONE（清空语义）。
//   - S_OUT/S_DONE：valid=1 时 40 点 {x,y} 输出流 + out_valid_flag=1；
//     valid=0 时无输出流直接 S_DONE。
//
// 读口（与 candidate_filter_ctrl / subpixel_ctrl 同款）：
//   - pt_rd_*：点读口，1 拍延迟（请求拍 → 转移拍 → 吸收拍）
//   - gray_rd_*：灰度读口，1 拍延迟（透传 subpixel_ctrl）
//==============================================================================
module grid_refine_ctrl #(
    parameter ROWS        = 5,
    parameter COLS        = 8,
    parameter N_ADDR_W    = 8,         // 点容量 256（≥ROWS*COLS）
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter GRAY_ADDR_W = 20,
    parameter ROM_FILE    = "../tests/build/vectors/gaussian_weights.mem"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    output reg                     valid_out,      // 最终网格有效标志（无效则清空）
    // 点读口（外部存储，1 拍延迟读）
    output reg                     pt_rd_en,
    output reg  [N_ADDR_W-1:0]     pt_rd_addr,
    input  wire [31:0]             pt_rd_x,
    input  wire [31:0]             pt_rd_y,
    // 灰度读口（外部存储，1 拍延迟读；组合透传 subpixel_ctrl）
    output wire                    gray_rd_en,
    output wire [GRAY_ADDR_W-1:0]  gray_rd_addr,
    input  wire [7:0]              gray_rd_data,
    // 输出：更新后 40 点流 + 有效标志
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_x,
    output reg  [31:0]             out_y,
    output reg                     out_valid_flag
);

    //--------------------------------------------------------------------
    // 状态
    //--------------------------------------------------------------------
    localparam S_IDLE = 3'd0, S_MINSTEP = 3'd1, S_HALFWIN = 3'd2,
               S_SUBPX = 3'd3, S_VALIDATE = 3'd4, S_OUT = 3'd5, S_DONE = 3'd6;
    reg [2:0] state;

    // MINSTEP 子状态（装载 + 边扫描）
    localparam M_LOAD = 3'd0, M_RD1 = 3'd1, M_RD2 = 3'd2, M_RD3 = 3'd3,
               M_RD4 = 3'd4, M_HYPW = 3'd5, M_HYPW2 = 3'd6, M_MINW = 3'd7;
    reg [2:0] mst;

    // HALFWIN / SUBPX / VALIDATE / OUT 子状态
    localparam H_MUL = 1'b0, H_W = 1'b1;
    reg hst;
    localparam P_START = 1'b0, P_RUN = 1'b1;
    reg pst;
    localparam V_START = 1'b0, V_RUN = 1'b1;
    reg vst;
    localparam O_RD = 1'b0, O_EMIT = 1'b1;
    reg ost;

    //--------------------------------------------------------------------
    // 常量（位模式，g++ 同机导出）
    //--------------------------------------------------------------------
    localparam [31:0] FLT_MAX = 32'h7F7FFFFF;
    localparam [31:0] C_015   = 32'h3E19999A;   // 0.15f
    localparam [31:0] C_1E30  = 32'h7149F2CA;   // 1e30f
    localparam [15:0] N_PTS   = ROWS * COLS;    // 40
    localparam [15:0] N_EDGE  = ROWS*(COLS-1) + (ROWS-1)*COLS;   // 67
    localparam [15:0] N_HEDGE = ROWS*(COLS-1);  // 35

    //--------------------------------------------------------------------
    // 控制寄存器
    //--------------------------------------------------------------------
    reg [31:0] min_step;            // 最短边（fp32）
    reg [7:0]  half_win;            // clamp(int(min_step*0.15f),2,10)
    reg [15:0] ld_cnt;              // 装载计数 0..41
    reg [15:0] scan_cnt;            // 边扫描计数 0..66
    reg [31:0] xa_r, ya_r, xb_r, yb_r;   // 边两端点
    reg [31:0] dx_r, dy_r;          // fp32_sub 结果锁存（喂 hypot）
    reg [15:0] spx_idx;             // subpixel 输出回写地址
    reg [15:0] oc_cnt;              // 输出流计数
    reg        grid_valid;

    //--------------------------------------------------------------------
    // 组合函数：fp32 → s32 截断（向零；正数 = floor，与 C++ int() 一致）
    //--------------------------------------------------------------------
    function automatic [31:0] f32_to_s32_trunc(input [31:0] v);
        integer e;
        reg [31:0] mag;
        begin
            if (v[30:23] == 8'hFF || v[30:23] == 8'h00) begin
                f32_to_s32_trunc = 32'd0;             // NaN/Inf/0（本域不出现）
            end else begin
                e = v[30:23] - 127;
                if (e < 0) begin
                    f32_to_s32_trunc = 32'd0;         // |v|<1 → 0
                end else if (e > 30) begin
                    f32_to_s32_trunc = 32'h7FFFFFFF;  // 饱和（本域不出现）
                end else if (e > 23) begin
                    mag = (32'd1 << e) | (v[22:0] << (e - 23));
                    f32_to_s32_trunc = v[31] ? -mag : mag;
                end else begin
                    mag = (32'd1 << e) | (v[22:0] >> (23 - e));
                    f32_to_s32_trunc = v[31] ? -mag : mag;
                end
            end
        end
    endfunction

    // clamp(int(v), 2, 10)
    function automatic [7:0] hw_clamp(input [31:0] v);
        reg [31:0] t;
        begin
            t = f32_to_s32_trunc(v);
            hw_clamp = (t < 32'd2)  ? 8'd2  :
                       (t > 32'd10) ? 8'd10 : t[7:0];
        end
    endfunction

    //--------------------------------------------------------------------
    // 边地址（scan_cnt → 端点 a/b；先横边后纵边，min 与顺序无关）
    //--------------------------------------------------------------------
    wire [15:0] h_r = scan_cnt / N_HEDGE;
    wire [15:0] h_c = scan_cnt % N_HEDGE;
    wire [15:0] h_a = h_r * COLS + h_c;
    wire [15:0] v_k = scan_cnt - N_HEDGE;
    wire [15:0] v_r = v_k / COLS;
    wire [15:0] v_c = v_k % COLS;
    wire [15:0] v_a = v_r * COLS + v_c;
    wire [15:0] ed_a = (scan_cnt < N_HEDGE) ? h_a : v_a;
    wire [15:0] ed_b = (scan_cnt < N_HEDGE) ? (h_a + 16'd1) : (v_a + COLS[15:0]);

    //--------------------------------------------------------------------
    // 子模块互连（先声明，避免隐式 net）
    //--------------------------------------------------------------------
    // fp32_sub ×2（dx/dy）→ fp32_hypot → fp32_mul（half_win 链）
    wire sdx_v, sdy_v, hyp_v, mul_v;
    wire [31:0] sdx_r, sdy_r, hyp_r, mul_r;
    wire sdx_go  = (state == S_MINSTEP) && (mst == M_HYPW);
    wire sdy_go  = (state == S_MINSTEP) && (mst == M_HYPW);
    wire hyp_go  = (state == S_MINSTEP) && (mst == M_HYPW2);
    wire mul_go  = (state == S_HALFWIN) && (hst == H_MUL);

    // subpixel_ctrl 互连
    wire spx_busy, spx_done;
    wire spx_pt_rd_en;
    wire [N_ADDR_W-1:0] spx_pt_rd_addr;
    wire spx_gray_rd_en;
    wire [GRAY_ADDR_W-1:0] spx_gray_rd_addr;
    wire spx_out_valid;
    wire [31:0] spx_out_x, spx_out_y;
    wire spx_start      = (state == S_SUBPX) && (pst == P_START);
    wire spx_out_ready  = (state == S_SUBPX);   // 输出流恒收

    // grid_validate 互连
    wire gv_busy, gv_done;
    wire gv_rd_en;
    wire [N_ADDR_W-1:0] gv_rd_addr;
    wire gv_valid_out;
    wire [31:0] gv_cost_out;
    wire gv_start = (state == S_VALIDATE) && (vst == V_START);

    // 中间 RAM 读口多路（SUBPX / VALIDATE / OUT 阶段互斥）
    wire iram_rd_en = (state == S_SUBPX)    ? spx_pt_rd_en :
                      (state == S_VALIDATE) ? gv_rd_en   :
                      (state == S_OUT)      ? 1'b1       : 1'b0;
    wire [N_ADDR_W-1:0] iram_rd_addr =
        (state == S_SUBPX)    ? spx_pt_rd_addr          :
        (state == S_VALIDATE) ? gv_rd_addr              :
        oc_cnt[N_ADDR_W-1:0];
    wire [31:0] iram_rd_x, iram_rd_y;

    // 中间 RAM 写口多路（MINSTEP 装载 / SUBPX 回写）
    wire iram_wr_en =
        ((state == S_MINSTEP) && (mst == M_LOAD) && (ld_cnt >= 16'd2)) ||
        ((state == S_SUBPX) && spx_out_valid && spx_out_ready);
    wire [15:0] ld_wr = ld_cnt - 16'd2;      // 装载写入地址（= 本拍到达点号）
    wire [N_ADDR_W-1:0] iram_wr_addr =
        (state == S_MINSTEP) ? ld_wr[N_ADDR_W-1:0] :
        spx_idx[N_ADDR_W-1:0];
    wire [31:0] iram_wr_x = (state == S_MINSTEP) ? pt_rd_x : spx_out_x;
    wire [31:0] iram_wr_y = (state == S_MINSTEP) ? pt_rd_y : spx_out_y;

    // 灰度读口：组合透传 subpixel_ctrl（其余阶段关断）
    assign gray_rd_en   = (state == S_SUBPX) ? spx_gray_rd_en   : 1'b0;
    assign gray_rd_addr = (state == S_SUBPX) ? spx_gray_rd_addr : {GRAY_ADDR_W{1'b0}};

    //--------------------------------------------------------------------
    // 子模块例化
    //--------------------------------------------------------------------
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_ADDR_W)) u_ramx (
        .clk(clk), .rst_n(rst_n),
        .wr_en(iram_wr_en), .wr_addr(iram_wr_addr), .wr_data(iram_wr_x),
        .rd_en(iram_rd_en), .rd_addr(iram_rd_addr), .rd_data(iram_rd_x)
    );
    dual_port_ram #(.DATA_WIDTH(32), .ADDR_WIDTH(N_ADDR_W)) u_ramy (
        .clk(clk), .rst_n(rst_n),
        .wr_en(iram_wr_en), .wr_addr(iram_wr_addr), .wr_data(iram_wr_y),
        .rd_en(iram_rd_en), .rd_addr(iram_rd_addr), .rd_data(iram_rd_y)
    );

    fp32_sub u_sdx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sdx_go), .in_ready(),
        .in_a(xa_r), .in_b(xb_r),
        .out_valid(sdx_v), .out_ready(1'b1), .out_r(sdx_r)
    );
    fp32_sub u_sdy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sdy_go), .in_ready(),
        .in_a(ya_r), .in_b(yb_r),
        .out_valid(sdy_v), .out_ready(1'b1), .out_r(sdy_r)
    );
    fp32_hypot u_hyp (
        .clk(clk), .rst_n(rst_n),
        .in_valid(hyp_go), .in_ready(),
        .in_a(dx_r), .in_b(dy_r),
        .out_valid(hyp_v), .out_ready(1'b1), .out_r(hyp_r)
    );
    fp32_mul u_mul (
        .clk(clk), .rst_n(rst_n),
        .in_valid(mul_go), .in_ready(),
        .in_a(min_step), .in_b(C_015),
        .out_valid(mul_v), .out_ready(1'b1), .out_r(mul_r)
    );

    subpixel_ctrl #(
        .IMG_W(IMG_W), .IMG_H(IMG_H),
        .GRAY_ADDR_W(GRAY_ADDR_W),
        .N_ADDR_W(N_ADDR_W),
        .PATCH_HW(16),
        .ROM_FILE(ROM_FILE)
    ) u_subpx (
        .clk(clk), .rst_n(rst_n),
        .start(spx_start), .busy(spx_busy), .done(spx_done),
        .n_in(N_PTS), .half_win(half_win),
        .pt_rd_en(spx_pt_rd_en), .pt_rd_addr(spx_pt_rd_addr),
        .pt_rd_x(iram_rd_x), .pt_rd_y(iram_rd_y),
        .gray_rd_en(spx_gray_rd_en), .gray_rd_addr(spx_gray_rd_addr),
        .gray_rd_data(gray_rd_data),
        .out_valid(spx_out_valid), .out_ready(spx_out_ready),
        .out_x(spx_out_x), .out_y(spx_out_y),
        .out_reliable()
    );

    grid_validate #(
        .ROWS(ROWS), .COLS(COLS), .N_ADDR_W(N_ADDR_W)
    ) u_gval (
        .clk(clk), .rst_n(rst_n),
        .start(gv_start), .busy(gv_busy), .done(gv_done),
        .n_in(N_PTS),
        .rd_en(gv_rd_en), .rd_addr(gv_rd_addr),
        .rd_x(iram_rd_x), .rd_y(iram_rd_y),
        .valid_out(gv_valid_out), .cost_out(gv_cost_out)
    );

    //--------------------------------------------------------------------
    // 状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            busy <= 1'b0; done <= 1'b0; valid_out <= 1'b0;
            out_valid <= 1'b0; out_valid_flag <= 1'b0;
            out_x <= 32'd0; out_y <= 32'd0;
            pt_rd_en <= 1'b0; pt_rd_addr <= {N_ADDR_W{1'b0}};
            min_step <= FLT_MAX;
            half_win <= 8'd0;
            ld_cnt <= 16'd0; scan_cnt <= 16'd0;
            mst <= M_LOAD; hst <= H_MUL; pst <= P_START;
            vst <= V_START; ost <= O_RD;
            xa_r <= 32'd0; ya_r <= 32'd0; xb_r <= 32'd0; yb_r <= 32'd0;
            dx_r <= 32'd0; dy_r <= 32'd0;
            spx_idx <= 16'd0; oc_cnt <= 16'd0;
            grid_valid <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        min_step <= FLT_MAX;
                        ld_cnt <= 16'd0;
                        scan_cnt <= 16'd0;
                        mst <= M_LOAD; pst <= P_START;
                        vst <= V_START; ost <= O_RD;
                        spx_idx <= 16'd0; oc_cnt <= 16'd0;
                        grid_valid <= 1'b0;
                        state <= S_MINSTEP;
                    end
                end

                //--------------------------------------------------------
                S_MINSTEP: begin
                    case (mst)
                        M_LOAD: begin
                            // 装载 40 点：请求 f(k) → 转移 → 吸收（2 拍）
                            if (ld_cnt == 16'd0) begin
                                pt_rd_en <= 1'b1;
                                pt_rd_addr <= {N_ADDR_W{1'b0}};
                                ld_cnt <= 16'd1;
                            end else if (ld_cnt == 16'd1) begin
                                pt_rd_addr <= ld_cnt[N_ADDR_W-1:0];
                                ld_cnt <= 16'd2;
                            end else begin
                                // 数据 f(ld_cnt-2) 本拍到达；组合写中间 RAM
                                if (ld_cnt == N_PTS) begin
                                    pt_rd_en <= 1'b0;
                                    ld_cnt <= N_PTS + 16'd1;
                                end else if (ld_cnt > N_PTS) begin
                                    // 最后一点已写入 → 边扫描
                                    scan_cnt <= 16'd0;
                                    mst <= M_RD1;
                                end else begin
                                    pt_rd_addr <= ld_cnt[N_ADDR_W-1:0];
                                    ld_cnt <= ld_cnt + 16'd1;
                                end
                            end
                        end
                        M_RD1: begin
                            pt_rd_en <= 1'b1;
                            pt_rd_addr <= ed_a[N_ADDR_W-1:0];
                            mst <= M_RD2;
                        end
                        M_RD2: begin
                            // 转移拍（addr 呈现给 RAM）；同拍发 b 地址
                            pt_rd_addr <= ed_b[N_ADDR_W-1:0];
                            mst <= M_RD3;
                        end
                        M_RD3: begin
                            // 吸收拍：锁存端点 a；关读口
                            xa_r <= pt_rd_x;
                            ya_r <= pt_rd_y;
                            pt_rd_en <= 1'b0;
                            mst <= M_RD4;
                        end
                        M_RD4: begin
                            // 吸收拍：锁存端点 b
                            xb_r <= pt_rd_x;
                            yb_r <= pt_rd_y;
                            mst <= M_HYPW;
                        end
                        M_HYPW: begin
                            // 组合发 fp32_sub(dx/dy)（持续至首结果）；锁存结果
                            if (sdx_v && sdy_v) begin
                                dx_r <= sdx_r;
                                dy_r <= sdy_r;
                                mst <= M_HYPW2;
                            end
                        end
                        M_HYPW2: begin
                            // 组合发 fp32_hypot（1 拍，用锁存 dx/dy）
                            mst <= M_MINW;
                        end
                        default: begin   // M_MINW
                            if (hyp_v) begin
                                if ($unsigned(hyp_r) < $unsigned(min_step))
                                    min_step <= hyp_r;
                                if (scan_cnt >= N_EDGE - 16'd1) begin
                                    hst <= H_MUL;
                                    state <= S_HALFWIN;
                                end else begin
                                    scan_cnt <= scan_cnt + 16'd1;
                                    mst <= M_RD1;
                                end
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------
                S_HALFWIN: begin
                    case (hst)
                        H_MUL: begin
                            // 组合发 fp32_mul(min_step, 0.15f)
                            hst <= H_W;
                        end
                        default: begin   // H_W
                            if (mul_v) begin
                                half_win <= hw_clamp(mul_r);
                                pst <= P_START;
                                state <= S_SUBPX;
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------
                S_SUBPX: begin
                    case (pst)
                        P_START: begin
                            // 本拍组合 spx_start=1（subpixel 启动）；初始化回写指针
                            spx_idx <= 16'd0;
                            pst <= P_RUN;
                        end
                        default: begin   // P_RUN
                            // 回写：spx 输出流（x,y）→ 中间 RAM（组合写口）
                            if (spx_out_valid && spx_out_ready) begin
                                if (spx_idx + 16'd1 >= N_PTS)
                                    spx_idx <= 16'd0;
                                else
                                    spx_idx <= spx_idx + 16'd1;
                            end
                            if (spx_done) begin
                                pst <= P_START;
                                vst <= V_START;
                                state <= S_VALIDATE;
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------
                S_VALIDATE: begin
                    case (vst)
                        V_START: begin
                            // 本拍组合 gv_start=1（grid_validate 启动）
                            vst <= V_RUN;
                        end
                        default: begin   // V_RUN
                            if (gv_done) begin
                                grid_valid <= ($unsigned(gv_cost_out) < C_1E30);
                                if ($unsigned(gv_cost_out) < C_1E30) begin
                                    oc_cnt <= 16'd0;
                                    ost <= O_RD;
                                    state <= S_OUT;
                                end else begin
                                    state <= S_DONE;   // 无效：无输出流（清空）
                                end
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------
                S_OUT: begin
                    case (ost)
                        O_RD: begin
                            // 请求读中间 RAM[oc_cnt]（数据下一拍到）
                            ost <= O_EMIT;
                        end
                        default: begin   // O_EMIT
                            if (!out_valid) begin
                                out_valid <= 1'b1;
                                out_valid_flag <= 1'b1;
                                out_x <= iram_rd_x;
                                out_y <= iram_rd_y;
                            end else if (out_ready) begin
                                out_valid <= 1'b0;
                                if (oc_cnt + 16'd1 >= N_PTS) begin
                                    state <= S_DONE;
                                end else begin
                                    oc_cnt <= oc_cnt + 16'd1;
                                    ost <= O_RD;
                                end
                            end
                        end
                    endcase
                end

                //--------------------------------------------------------
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    valid_out <= grid_valid;
                    out_valid_flag <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
