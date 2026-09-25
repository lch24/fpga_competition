`timescale 1ns / 1ps
//==============================================================================
// subpixel_ctrl.sv — M5 亚像素精定位主控（复刻 algo/subpixel.cpp::refine_subpixel）
//------------------------------------------------------------------------------
// 对点流逐点做 40 轮以内的高斯加权梯度交点迭代：
//   sample：floor 取整 + 小数部分，bilinear_core 插值（float）
//   gx/gy  ：相邻 1px 差分（fp32_sub，float 精度）→ f32_to_f64 提升（精确）
//   w      ：exp(-(x²+y²)/r²)，离线 ROM（gaussian_weights.mem，radius=2..15）
//            段内 y 外循环 x 内循环（与 export_m5.cpp 累加顺序一致）
//   xx/xy/yy/bx/by：subpixel_accum 五项 FP64 累加（同序逐次舍入）
//   solve  ：tensor_solve（FP64，det/trace 可靠性判定 + 除法）
//   next   ：f64_to_f32(p + dx/dy)；非有限 || hypot(next-original)>radius → 失败
//   converge：dx²+dy²<1e-6（FP64）→ reliable；40 轮未收敛保留 original
// 位级权威 = tests/rtl/export_m5.cpp（refine_subpixel_traj），逐迭代一致。
//
// 三级循环：点（0..n_in-1）→ 迭代（0..39）→ 窗口（y=-r..r 外、x=-r..r 内）。
//
// 存储与读口：
//   - 点读口 pt_rd_*：1 拍延迟（candidate_store 单读口阶段复用；SUBPX 期间归本模块）
//   - 灰度读口 gray_rd_*：1 拍延迟（外部存储/DDR 模型；与 ring_check 同款）
//   - patch RAM：每迭代按当前 floor(p) 预装 [floor(p.x)-r-1, floor(p.x)+r+2]×
//     [floor(p.y)-r-1, floor(p.y)+r+2]（2r+4 行 × 2r+4 列，r=15 时 34×34），
//     组合读（reg 数组），迭代中窗口像素 0 延迟取。
//   - w ROM：gaussian_weights.mem，段基址按 r 查（每 r 一段，(2r+1)² 项）。
//
// 时序契约（不得改动）：
//   - pt/gray 读口：rd_en=1 的下一拍数据有效（请求拍 → 转移拍 → 吸收拍）。
//   - fp32/fp64 算术模块：fire 拍接受，out_valid 下一拍起（弹握手流水）。
//   - bilinear_core 一次 2 拍延迟；每窗口位置需 4 次 bilinear（sample(sx±1,sy)、
//     (sx,sy±1)）+ 2 次 fp32_sub 差分 + f32_to_f64 提升 → 喂 accum 一组。
//   - accum：逐样本流水吸收，n_win=(2r+1)² 收齐后 out_valid 给 5 项。
//   - tensor_solve：start 后 busy/done，出 ok/dx/dy。
//   - 输出流 out_*：reliable=1 收敛更新 / =0 保留 original；out_valid 握手。
//==============================================================================
module subpixel_ctrl #(
    parameter IMG_W        = 1280,
    parameter IMG_H        = 720,
    parameter GRAY_ADDR_W  = 20,        // ≥ $clog2(W*H)
    parameter N_ADDR_W     = 8,         // 点容量 256
    parameter PATCH_HW     = 16,        // 最大半宽 r+1=16 → patch 2r+4=34
    parameter ROM_FILE     = "../tests/build/vectors/gaussian_weights.mem"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,          // IDLE 时启动
    output reg                     busy,
    output reg                     done,
    input  wire [15:0]             n_in,           // 点数（≥0）
    input  wire [7:0]              half_win,       // 半径输入（模块内 clamp 2..15）
    // 点读口（外部存储，1 拍延迟读）
    output reg                     pt_rd_en,
    output reg  [N_ADDR_W-1:0]     pt_rd_addr,
    input  wire [31:0]             pt_rd_x,
    input  wire [31:0]             pt_rd_y,
    // 灰度读口（外部存储，1 拍延迟读）
    output reg                     gray_rd_en,
    output reg  [GRAY_ADDR_W-1:0]  gray_rd_addr,
    input  wire [7:0]              gray_rd_data,
    // 输出：更新后坐标流（fp32）+ 收敛标志
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_x,
    output reg  [31:0]             out_y,
    output reg                     out_reliable
);

    //--------------------------------------------------------------------
    // 常量
    //--------------------------------------------------------------------
    localparam [31:0] ONE      = 32'h3F800000;   // 1.0f
    localparam [31:0] NEG_ONE  = 32'hBF800000;   // -1.0f
    localparam [63:0] C_1E6    = 64'h3EB0C6F7A0B5ED8D;  // 1e-6（m5_const.txt）

    //--------------------------------------------------------------------
    // 状态（顶层）
    //--------------------------------------------------------------------
    localparam S_IDLE = 3'd0, S_LOAD = 3'd1, S_ITER = 3'd2, S_OUT = 3'd3,
               S_DONE = 3'd4;
    reg [2:0] state;

    // 迭代子状态（S_ITER 内）
    localparam I_BOUND = 3'd0, I_PATCH = 3'd1, I_WIN = 3'd2,
               I_SOLVE = 3'd3, I_UPDATE = 3'd4, I_NEXT = 3'd5;
    reg [2:0] ist;

    // 窗口子状态（I_WIN 内 wst）
    localparam W_INIT  = 4'd0,  // 发 sx/sy 基址加（cur + 偏移）
               W_WAIT0 = 4'd1,  // 等基址加 → 锁存 sx_r/sy_r + w_q
               W_ADD   = 4'd2,  // 发本样本变轴加/共享小数减
               W_WAIT1 = 4'd3,  // 等加 → 锁存 s_chg、发变轴小数减
               W_WAIT2 = 4'd4,  // 等变轴小数减 → 锁存 dx_f/dy_f
               W_FIRE  = 4'd5,  // 发 bilinear（像素组合读 + 小数）
               W_BIL   = 4'd6,  // 等 bilinear 出 → 锁存 bilA/B/C/D
               W_DIFF  = 4'd7,  // 发 gx/gy 差分减
               W_DIFFW = 4'd8,  // 等差分 → 提升 f64 + 锁存 accum 输入
               W_FEED  = 4'd9;  // accum 握手喂一组；推进 w_x/w_y
    reg [3:0] wst;

    // 样本相位（0=A: sx+1, 1=B: sx-1, 2=C: sy+1, 3=D: sy-1）
    reg [1:0] smpl;

    // solve 子状态（I_SOLVE 内 nst）
    localparam SV0 = 4'd0, SV1 = 4'd1, SV2 = 4'd2, SV3 = 4'd3,
               SV4 = 4'd4, SV5 = 4'd5, SV6 = 4'd6, SV7 = 4'd7,
               SV8 = 4'd8, SV9 = 4'd9;
    reg [3:0] nst;

    // 更新子状态（I_UPDATE 内 ust）
    localparam U0 = 2'd0, U1 = 2'd1, U2 = 2'd2;
    reg [1:0] ust;

    // 点加载相位（S_LOAD 内）
    reg [1:0] ld_st;

    //--------------------------------------------------------------------
    // 控制寄存器
    //--------------------------------------------------------------------
    reg [15:0] pt_idx;              // 当前点号
    reg [6:0]  iter;                // 0..39
    reg [7:0]  radius;              // clamp 后的半径
    reg [31:0] orig_x, orig_y;      // original（回滚用）
    reg [31:0] cur_x,  cur_y;       // 当前 p
    reg        reliable;
    reg [15:0] w_x, w_y;            // 窗口计数索引
    reg [15:0] n_win;               // (2r+1)²
    reg [11:0] patch_cnt;           // patch 预装计数（≤ (2r+4)² = 1156）
    reg [31:0] next_x, next_y;      // solve 后 next（I_UPDATE 用）
    reg        conv_ok;             // 本迭代收敛标志

    // 窗口数据暂存
    reg [31:0] sx_r, sy_r;          // sx = cur + 偏移（fp32_add 结果）
    reg [31:0] s_chg;               // 变轴坐标（sx±1 / sy±1）
    reg [31:0] dx_f, dy_f;          // 当前样本小数部分（fp32_sub 结果）
    reg [31:0] bilA_r, bilB_r, bilC_r, bilD_r;
    reg [63:0] gx64_r, gy64_r;      // 差分 → f64 提升
    reg [63:0] acc_x_r, acc_y_r, acc_w_r, acc_gx_r, acc_gy_r;  // accum 输入锁存
    reg [63:0] ts_dx_r, ts_dy_r;    // tensor_solve 输出锁存
    reg [63:0] nextx64_r;           // fp64_add(cur_x, dx) 结果
    reg [63:0] cmulx_r, cmuly_r;    // dx², dy²（收敛链）
    reg [63:0] conv_sum_r;          // dx²+dy²
    reg        acc_start_p, ts_start_p;   // 脉冲
    reg        solve_started;       // tensor_solve start 已接受标志

    //--------------------------------------------------------------------
    // patch RAM（2r+4=34 上限；组合读）
    //--------------------------------------------------------------------
    reg [7:0] patch [0:(2*PATCH_HW+2)*(2*PATCH_HW+2)-1];   // 34×34
    reg [15:0] patch_px, patch_py;   // patch 原点（floor(p)-r-1）
    reg [7:0]  patch_rd_a, patch_rd_b, patch_rd_c, patch_rd_d; // 组合读暂存

    //--------------------------------------------------------------------
    // 组合函数：int（|v|<2^24）→ fp32 位模式（精确）
    //--------------------------------------------------------------------
    function automatic [31:0] int_to_f32(input integer v);
        integer mag, i;
        reg [31:0] sh;
        reg [4:0] e;
        begin
            if (v == 0) begin
                int_to_f32 = 32'h00000000;
            end else begin
                mag = (v < 0) ? -v : v;
                e = 5'd0;
                for (i = 30; i >= 0; i = i - 1)
                    if (mag[i]) begin e = i[4:0]; i = -1; end
                sh = mag << (23 - e);
                int_to_f32 = {v[31], 8'd127 + e, sh[22:0]};
            end
        end
    endfunction

    // fp32（非负有限）→ s32 截断（floor 等价；值域 <2^16）
    function automatic [15:0] f32_trunc16(input [31:0] v);
        integer e;
        begin
            if (v[30:23] == 8'hFF || v[30:23] == 8'h00) begin
                f32_trunc16 = 16'd0;                 // NaN/Inf/0 → 0（本域不出现）
            end else begin
                e = v[30:23] - 127;
                if (e < 0)          f32_trunc16 = 16'd0;
                else if (e > 15)    f32_trunc16 = 16'hFFFF;
                else                f32_trunc16 = (16'd1 << e) | (v[22:0] >> (23 - e));
            end
        end
    endfunction

    // fp32 → fp64 提升（精确：±0/次正规/正常全覆盖）
    function automatic [63:0] f32_to_f64_c(input [31:0] v);
        begin
            if (v[30:23] == 8'hFF) begin
                f32_to_f64_c = {v[31], 11'h7FF, v[22:0], 29'd0};   // NaN/Inf 防御
            end else if (v[30:23] == 8'h00) begin
                if (v[22:0] == 23'd0)
                    f32_to_f64_c = {v[31], 11'd0, 52'd0};          // ±0
                else
                    f32_to_f64_c = {v[31], 11'd845, v[22:0], 29'd0}; // 次正规（防御）
            end else begin
                f32_to_f64_c = {v[31], {3'b000, v[30:23]} + 11'd896,
                                v[22:0], 29'd0};
            end
        end
    endfunction

    // fp32 有符号数值比较 a < b（含符号；无 NaN，本域 cur 恒有限）
    function automatic logic fp_lt(input logic [31:0] a, input logic [31:0] b);
        if (a[31] && !b[31])      fp_lt = 1'b1;
        else if (!a[31] && b[31]) fp_lt = 1'b0;
        else if (a[31])           fp_lt = ($unsigned(a) > $unsigned(b));
        else                      fp_lt = ($unsigned(a) < $unsigned(b));
    endfunction

    //--------------------------------------------------------------------
    // 派生常量（半径在 S_IDLE 锁定后稳定）
    //--------------------------------------------------------------------
    wire [7:0] hw_clamped = (half_win < 8'd2) ? 8'd2 :
                            (half_win > 8'd15) ? 8'd15 : half_win;
    wire [31:0] lo_f    = int_to_f32($signed({1'b0, radius}) + 32'sd1);
    wire [31:0] hi_x_f  = int_to_f32($signed(IMG_W) - 32'sd2 - $signed({1'b0, radius}));
    wire [31:0] hi_y_f  = int_to_f32($signed(IMG_H) - 32'sd2 - $signed({1'b0, radius}));
    wire [31:0] radius_f = int_to_f32({1'b0, radius});
    wire [9:0]  pw      = 2 * radius + 4;            // patch 边长 2r+4
    wire [11:0] pw2     = pw * pw;                   // 总字节 (2r+4)²
    wire [7:0]  win_last = 2 * radius;               // 最后窗口索引 2r
    wire signed [15:0] win_ox = $signed(w_x) - $signed({1'b0, radius});
    wire signed [15:0] win_oy = $signed(w_y) - $signed({1'b0, radius});

    //--------------------------------------------------------------------
    // 子模块互连
    //--------------------------------------------------------------------
    // bilinear_core
    wire bil_in_ready, bil_out_valid;
    wire [31:0] bil_out_r;
    wire bil_fire = (ist == I_WIN) && (wst == W_FIRE) && bil_in_ready;

    // fp32_add ×2（基址/变轴）
    wire add_sx_fire = (wst == W_INIT) ||
                       ((wst == W_ADD) && (smpl == 2'd0 || smpl == 2'd1));
    wire [31:0] add_sx_a = (wst == W_INIT) ? cur_x : sx_r;
    wire [31:0] add_sx_b = (wst == W_INIT) ? int_to_f32(win_ox) :
                           ((smpl == 2'd0) ? ONE : NEG_ONE);
    wire add_sx_v;
    wire [31:0] add_sx_r;

    wire add_sy_fire = (wst == W_INIT) ||
                       ((wst == W_ADD) && (smpl == 2'd2 || smpl == 2'd3));
    wire [31:0] add_sy_a = (wst == W_INIT) ? cur_y : sy_r;
    wire [31:0] add_sy_b = (wst == W_INIT) ? int_to_f32(win_oy) :
                           ((smpl == 2'd2) ? ONE : NEG_ONE);
    wire add_sy_v;
    wire [31:0] add_sy_r;

    // fp32_sub ×2（小数部分 x/y）
    wire sub_dx_fire = ((wst == W_ADD)   && (smpl == 2'd2)) ||        // C 共享 dx
                       ((wst == W_WAIT1) && (smpl == 2'd0 || smpl == 2'd1) && add_sx_v);
    wire [31:0] sub_dx_a = ((wst == W_WAIT1) && (smpl == 2'd0 || smpl == 2'd1))
                           ? add_sx_r : sx_r;
    wire [31:0] sub_dx_b = int_to_f32(f32_trunc16(sub_dx_a));
    wire sub_dx_v;
    wire [31:0] sub_dx_r;

    wire sub_dy_fire = ((wst == W_ADD)   && (smpl == 2'd0)) ||        // A 共享 dy
                       ((wst == W_WAIT1) && (smpl == 2'd2 || smpl == 2'd3) && add_sy_v);
    wire [31:0] sub_dy_a = ((wst == W_WAIT1) && (smpl == 2'd2 || smpl == 2'd3))
                           ? add_sy_r : sy_r;
    wire [31:0] sub_dy_b = int_to_f32(f32_trunc16(sub_dy_a));
    wire sub_dy_v;
    wire [31:0] sub_dy_r;

    // fp32_sub ×2（gx/gy 差分；I_UPDATE 复用做 hypot 输入）
    wire sub_gx_fire = ((ist == I_WIN)    && (wst == W_DIFF)) ||
                       ((ist == I_UPDATE) && (ust == U0));
    wire [31:0] sub_gx_a = (ist == I_WIN) ? bilA_r : next_x;
    wire [31:0] sub_gx_b = (ist == I_WIN) ? bilB_r : orig_x;
    wire sub_gx_v;
    wire [31:0] sub_gx_r;

    wire sub_gy_fire = ((ist == I_WIN)    && (wst == W_DIFF)) ||
                       ((ist == I_UPDATE) && (ust == U0));
    wire [31:0] sub_gy_a = (ist == I_WIN) ? bilC_r : next_y;
    wire [31:0] sub_gy_b = (ist == I_WIN) ? bilD_r : orig_y;
    wire sub_gy_v;
    wire [31:0] sub_gy_r;

    // fp64_mul ×2（收敛 dx²/dy²）
    wire cmul_fire = (nst == SV2);
    wire cmul_v1, cmul_v2;
    wire [63:0] cmul_r1, cmul_r2;

    // fp64_add（收敛和 dx²+dy²）
    wire acnv_fire = (ust == U0);
    wire acnv_v;
    wire [63:0] acnv_r;

    // fp64_add（next = cur + dx / cur + dy）
    wire anxtx_fire = (nst == SV2);
    wire anxty_fire = (nst == SV7);
    wire anxt_v;
    wire [63:0] anxt_r;

    // f64_to_f32 ×1（next_x/next_y 串行）
    wire cvtx_fire = (nst == SV5);
    wire cvty_fire = (nst == SV8) && anxt_v;
    wire [63:0] cvt_in = (nst == SV5) ? nextx64_r : anxt_r;
    wire cvt_v;
    wire [31:0] cvt_r;

    // fp32_hypot（|next-orig|）
    wire hyp_fire = (ust == U1) && sub_gx_v && sub_gy_v;
    wire hyp_v;
    wire [31:0] hyp_r;

    // accum 输入 valid（组合：W_FEED 期间持高，握手拍与推进同拍）
    wire acc_in_valid = (ist == I_WIN) && (wst == W_FEED);
    wire acc_in_ready, acc_out_valid, acc_done, acc_busy;
    wire [63:0] acc_out_a, acc_out_b, acc_out_c, acc_out_bx, acc_out_by;

    wire ts_busy, ts_done, ts_out_ok;
    wire [63:0] ts_out_dx, ts_out_dy;

    // anxt 操作数：SV2 做 x（cur_x+dx）、SV7 做 y（cur_y+dy）
    wire [63:0] anxt_a = (nst == SV2) ? f32_to_f64_c(cur_x) : f32_to_f64_c(cur_y);
    wire [63:0] anxt_b = (nst == SV2) ? ts_dx_r : ts_dy_r;

    //--------------------------------------------------------------------
    // 窗口像素组合读（pidx 越界防护：仅 W_FIRE 期间使用，防御性钳 0）
    //--------------------------------------------------------------------
    wire [15:0] cur_ix = (smpl < 2'd2) ? f32_trunc16(s_chg) : f32_trunc16(sx_r);
    wire [15:0] cur_iy = (smpl < 2'd2) ? f32_trunc16(sy_r)   : f32_trunc16(s_chg);
    wire [15:0] cur_col = cur_ix - patch_px;
    wire [15:0] cur_row = cur_iy - patch_py;
    wire        idx_ok = (cur_row < pw) && (cur_col < pw);
    wire [15:0] pidx00 = idx_ok ? (cur_row * pw + cur_col) : 16'd0;
    wire [15:0] pidx10 = pidx00 + 16'd1;
    wire [15:0] pidx01 = pidx00 + pw;
    wire [15:0] pidx11 = pidx00 + pw + 16'd1;
    wire [31:0] bil_p00 = int_to_f32(patch[pidx00]);
    wire [31:0] bil_p10 = int_to_f32(patch[pidx10]);
    wire [31:0] bil_p01 = int_to_f32(patch[pidx01]);
    wire [31:0] bil_p11 = int_to_f32(patch[pidx11]);
    wire [31:0] bil_dx_i = dx_f;
    wire [31:0] bil_dy_i = dy_f;

    //--------------------------------------------------------------------
    // w ROM（gaussian_weights.mem，段内 y 外 x 内；段基址按 r）
    //--------------------------------------------------------------------
    reg [63:0] w_rom [0:5445];      // Σ(2r+1)², r=2..15 = 5446 项
    reg [63:0] w_q;
    reg [12:0] w_addr;
    initial $readmemh(ROM_FILE, w_rom);

    // r 段基址查表（r=2..15 → 段起点；case 全列举）
    function automatic [12:0] w_base(input integer r);
        case (r)
            2:  w_base = 13'd0;
            3:  w_base = 13'd25;
            4:  w_base = 13'd74;
            5:  w_base = 13'd155;
            6:  w_base = 13'd276;
            7:  w_base = 13'd445;
            8:  w_base = 13'd670;
            9:  w_base = 13'd959;
            10: w_base = 13'd1320;
            11: w_base = 13'd1761;
            12: w_base = 13'd2290;
            13: w_base = 13'd2915;
            14: w_base = 13'd3644;
            15: w_base = 13'd4485;
            default: w_base = 13'd0;
        endcase
    endfunction

    //--------------------------------------------------------------------
    // 子模块例化
    //--------------------------------------------------------------------
    bilinear_core u_bil (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bil_fire), .in_ready(bil_in_ready),
        .in_p00(bil_p00), .in_p10(bil_p10), .in_p01(bil_p01), .in_p11(bil_p11),
        .in_dx(bil_dx_i), .in_dy(bil_dy_i),
        .out_valid(bil_out_valid), .out_ready(1'b1), .out_r(bil_out_r)
    );

    tensor_solve u_ts (
        .clk(clk), .rst_n(rst_n),
        .start(ts_start_p), .busy(ts_busy), .done(ts_done),
        .in_a(acc_out_a), .in_b(acc_out_b), .in_c(acc_out_c),
        .in_bx(acc_out_bx), .in_by(acc_out_by),
        .out_ok(ts_out_ok), .out_dx(ts_out_dx), .out_dy(ts_out_dy)
    );

    subpixel_accum #(.N_ADDR_W(N_ADDR_W)) u_acc (
        .clk(clk), .rst_n(rst_n),
        .start(acc_start_p), .busy(acc_busy), .done(acc_done),
        .n_win(n_win),
        .in_valid(acc_in_valid), .in_ready(acc_in_ready),
        .in_x(acc_x_r), .in_y(acc_y_r), .in_w(acc_w_r),
        .in_gx(acc_gx_r), .in_gy(acc_gy_r),
        .out_valid(acc_out_valid), .out_ready(1'b1),
        .out_a(acc_out_a), .out_b(acc_out_b), .out_c(acc_out_c),
        .out_bx(acc_out_bx), .out_by(acc_out_by)
    );

    fp32_add u_add_sx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(add_sx_fire), .in_ready(),
        .in_a(add_sx_a), .in_b(add_sx_b),
        .out_valid(add_sx_v), .out_ready(1'b1), .out_r(add_sx_r)
    );
    fp32_add u_add_sy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(add_sy_fire), .in_ready(),
        .in_a(add_sy_a), .in_b(add_sy_b),
        .out_valid(add_sy_v), .out_ready(1'b1), .out_r(add_sy_r)
    );
    fp32_sub u_sub_dx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_dx_fire), .in_ready(),
        .in_a(sub_dx_a), .in_b(sub_dx_b),
        .out_valid(sub_dx_v), .out_ready(1'b1), .out_r(sub_dx_r)
    );
    fp32_sub u_sub_dy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_dy_fire), .in_ready(),
        .in_a(sub_dy_a), .in_b(sub_dy_b),
        .out_valid(sub_dy_v), .out_ready(1'b1), .out_r(sub_dy_r)
    );
    fp32_sub u_sub_gx (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_gx_fire), .in_ready(),
        .in_a(sub_gx_a), .in_b(sub_gx_b),
        .out_valid(sub_gx_v), .out_ready(1'b1), .out_r(sub_gx_r)
    );
    fp32_sub u_sub_gy (
        .clk(clk), .rst_n(rst_n),
        .in_valid(sub_gy_fire), .in_ready(),
        .in_a(sub_gy_a), .in_b(sub_gy_b),
        .out_valid(sub_gy_v), .out_ready(1'b1), .out_r(sub_gy_r)
    );
    fp64_mul u_cmul1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(cmul_fire), .in_ready(),
        .in_a(ts_dx_r), .in_b(ts_dx_r),
        .out_valid(cmul_v1), .out_ready(1'b1), .out_r(cmul_r1)
    );
    fp64_mul u_cmul2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(cmul_fire), .in_ready(),
        .in_a(ts_dy_r), .in_b(ts_dy_r),
        .out_valid(cmul_v2), .out_ready(1'b1), .out_r(cmul_r2)
    );
    fp64_add u_acnv (
        .clk(clk), .rst_n(rst_n),
        .in_valid(acnv_fire), .in_ready(),
        .in_a(cmulx_r), .in_b(cmuly_r),
        .out_valid(acnv_v), .out_ready(1'b1), .out_r(acnv_r)
    );
    fp64_add u_anxt (
        .clk(clk), .rst_n(rst_n),
        .in_valid(anxtx_fire || anxty_fire), .in_ready(),
        .in_a(anxt_a), .in_b(anxt_b),
        .out_valid(anxt_v), .out_ready(1'b1), .out_r(anxt_r)
    );
    f64_to_f32 u_cvt (
        .clk(clk), .rst_n(rst_n),
        .in_valid(cvtx_fire || cvty_fire), .in_ready(),
        .in_x(cvt_in),
        .out_valid(cvt_v), .out_ready(1'b1), .out_r(cvt_r)
    );
    fp32_hypot u_hyp (
        .clk(clk), .rst_n(rst_n),
        .in_valid(hyp_fire), .in_ready(),
        .in_a(sub_gx_r), .in_b(sub_gy_r),
        .out_valid(hyp_v), .out_ready(1'b1), .out_r(hyp_r)
    );

    //--------------------------------------------------------------------
    // clamp(half_win, 2, 15)
    //--------------------------------------------------------------------

    //--------------------------------------------------------------------
    // 顶层状态机
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; ist <= I_BOUND;
            busy <= 1'b0; done <= 1'b0;
            pt_idx <= 16'd0; iter <= 7'd0; radius <= 8'd0;
            reliable <= 1'b0;
            out_valid <= 1'b0; out_reliable <= 1'b0;
            out_x <= 32'd0; out_y <= 32'd0;
            pt_rd_en <= 1'b0; gray_rd_en <= 1'b0;
            gray_rd_addr <= 0;
            ld_st <= 2'd0;
            wst <= W_INIT; smpl <= 2'd0; nst <= SV0; ust <= U0;
            w_x <= 16'd0; w_y <= 16'd0;
            patch_cnt <= 12'd0;
            patch_px <= 16'd0; patch_py <= 16'd0;
            sx_r <= 32'd0; sy_r <= 32'd0; s_chg <= 32'd0;
            dx_f <= 32'd0; dy_f <= 32'd0;
            bilA_r <= 32'd0; bilB_r <= 32'd0; bilC_r <= 32'd0; bilD_r <= 32'd0;
            gx64_r <= 64'd0; gy64_r <= 64'd0;
            acc_x_r <= 64'd0; acc_y_r <= 64'd0; acc_w_r <= 64'd0;
            acc_gx_r <= 64'd0; acc_gy_r <= 64'd0;
            ts_dx_r <= 64'd0; ts_dy_r <= 64'd0;
            nextx64_r <= 64'd0; cmulx_r <= 64'd0; cmuly_r <= 64'd0;
            conv_sum_r <= 64'd0;
            acc_start_p <= 1'b0; ts_start_p <= 1'b0;
            solve_started <= 1'b0;
            next_x <= 32'd0; next_y <= 32'd0;
            conv_ok <= 1'b0;
        end else begin
            done <= 1'b0;
            acc_start_p <= 1'b0;
            ts_start_p <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        radius <= hw_clamped;
                        n_win <= (2 * hw_clamped + 1) * (2 * hw_clamped + 1);
                        pt_idx <= 16'd0;
                        if (n_in == 16'd0) begin
                            state <= S_DONE;
                        end else begin
                            ld_st <= 2'd0;
                            state <= S_LOAD;
                        end
                    end
                end
                S_LOAD: begin
                    case (ld_st)
                        2'd0: begin
                            pt_rd_en <= 1'b1;
                            pt_rd_addr <= pt_idx[N_ADDR_W-1:0];
                            ld_st <= 2'd1;
                        end
                        2'd1: begin
                            // 点数据在途（1 拍读延迟）
                            ld_st <= 2'd2;
                        end
                        default: begin
                            orig_x <= pt_rd_x;
                            orig_y <= pt_rd_y;
                            cur_x  <= pt_rd_x;
                            cur_y  <= pt_rd_y;
                            pt_rd_en <= 1'b0;
                            reliable <= 1'b0;
                            if (pt_rd_x[30:23] == 8'hFF || pt_rd_y[30:23] == 8'hFF) begin
                                // 非有限原始点：直接输出 original/reliable=0
                                state <= S_OUT;
                            end else begin
                                iter <= 7'd0;
                                ist <= I_BOUND;
                                state <= S_ITER;
                            end
                        end
                    endcase
                end
                S_ITER: begin
                    case (ist)
                        I_BOUND: begin
                            // p.x<r+1 || p.y<r+1 || p.x>=W-r-2 || p.y>=H-r-2 → 失败
                            if (fp_lt(cur_x, lo_f) || fp_lt(cur_y, lo_f) ||
                                !fp_lt(cur_x, hi_x_f) || !fp_lt(cur_y, hi_y_f)) begin
                                reliable <= 1'b0;
                                state <= S_OUT;
                            end else begin
                                acc_start_p <= 1'b1;          // 重启累加器
                                patch_px <= f32_trunc16(cur_x) - {8'd0, radius} - 16'd1;
                                patch_py <= f32_trunc16(cur_y) - {8'd0, radius} - 16'd1;
                                patch_cnt <= 12'd0;
                                ist <= I_PATCH;
                            end
                        end
                        I_PATCH: begin
                            // gray 读口 1 拍延迟：cnt 拍请求 f(cnt)，cnt+1 拍数据到，
                            // 写入 patch[cnt-1]；请求与写入同拍推进（流水）。
                            if (patch_cnt == 12'd0) begin
                                gray_rd_en   <= 1'b1;
                                gray_rd_addr <= patch_py * IMG_W + patch_px;
                                patch_cnt    <= 12'd1;
                            end else if (patch_cnt == 12'd1) begin
                                // 首个请求 f(0) 在途（数据下一拍到）；请求 f(1)
                                gray_rd_addr <= patch_py * IMG_W + patch_px
                                              + (12'd1 / pw) * IMG_W + (12'd1 % pw);
                                patch_cnt    <= 12'd2;
                            end else begin
                                // 写入 f(cnt-2) 数据（cnt-1 拍请求、cnt 拍到达）
                                patch[patch_cnt - 12'd2] <= gray_rd_data;
                                if (patch_cnt == pw2) begin
                                    // 已请求 f(pw2-1)；关读口，数据下一拍到
                                    gray_rd_en <= 1'b0;
                                    patch_cnt  <= patch_cnt + 12'd1;
                                end else if (patch_cnt > pw2) begin
                                    // 最后一项 f(pw2-1) 写入完成
                                    w_x <= 16'd0; w_y <= 16'd0;
                                    wst <= W_INIT; smpl <= 2'd0;
                                    ist <= I_WIN;
                                end else begin
                                    gray_rd_addr <= patch_py * IMG_W + patch_px
                                                  + (patch_cnt / pw) * IMG_W
                                                  + (patch_cnt % pw);
                                    patch_cnt <= patch_cnt + 12'd1;
                                end
                            end
                        end
                        I_WIN: begin
                            case (wst)
                                W_INIT: begin
                                    // 发 sx/sy 基址加 + ROM 地址
                                    w_addr <= w_base(radius) + w_y * (2 * radius + 1) + w_x;
                                    wst <= W_WAIT0;
                                end
                                W_WAIT0: begin
                                    w_q <= w_rom[w_addr];
                                    if (add_sx_v && add_sy_v) begin
                                        sx_r <= add_sx_r;
                                        sy_r <= add_sy_r;
                                        smpl <= 2'd0;
                                        wst <= W_ADD;
                                    end
                                end
                                W_ADD: begin
                                    // 本样本组合发（变轴加 + 共享小数减）
                                    wst <= W_WAIT1;
                                end
                                W_WAIT1: begin
                                    if ((smpl == 2'd0 || smpl == 2'd1) && add_sx_v) begin
                                        s_chg <= add_sx_r;
                                        if (smpl == 2'd0) begin
                                            if (sub_dy_v) begin
                                                dy_f <= sub_dy_r;
                                                wst <= W_WAIT2;
                                            end
                                        end else begin
                                            wst <= W_WAIT2;
                                        end
                                    end else if ((smpl == 2'd2 || smpl == 2'd3) && add_sy_v) begin
                                        s_chg <= add_sy_r;
                                        if (smpl == 2'd2) begin
                                            if (sub_dx_v) begin
                                                dx_f <= sub_dx_r;
                                                wst <= W_WAIT2;
                                            end
                                        end else begin
                                            wst <= W_WAIT2;
                                        end
                                    end
                                end
                                W_WAIT2: begin
                                    if ((smpl == 2'd0 || smpl == 2'd1) && sub_dx_v) begin
                                        dx_f <= sub_dx_r;
                                        wst <= W_FIRE;
                                    end else if ((smpl == 2'd2 || smpl == 2'd3) && sub_dy_v) begin
                                        dy_f <= sub_dy_r;
                                        wst <= W_FIRE;
                                    end
                                end
                                W_FIRE: begin
                                    // 组合像素 + 小数 → bilinear（bil_fire 组合）
                                    if (bil_in_ready)
                                        wst <= W_BIL;
                                end
                                W_BIL: begin
                                    if (bil_out_valid) begin
                                        case (smpl)
                                            2'd0: bilA_r <= bil_out_r;
                                            2'd1: bilB_r <= bil_out_r;
                                            2'd2: bilC_r <= bil_out_r;
                                            default: bilD_r <= bil_out_r;
                                        endcase
                                        if (smpl == 2'd3) begin
                                            wst <= W_DIFF;
                                        end else begin
                                            smpl <= smpl + 2'd1;
                                            wst <= W_ADD;
                                        end
                                    end
                                end
                                W_DIFF: begin
                                    // 发 gx/gy 差分减（组合）
                                    wst <= W_DIFFW;
                                end
                                W_DIFFW: begin
                                    if (sub_gx_v && sub_gy_v) begin
                                        acc_x_r <= f32_to_f64_c(int_to_f32(win_ox));
                                        acc_y_r <= f32_to_f64_c(int_to_f32(win_oy));
                                        acc_w_r <= w_q;
                                        acc_gx_r <= f32_to_f64_c(sub_gx_r);
                                        acc_gy_r <= f32_to_f64_c(sub_gy_r);
                                        wst <= W_FEED;
                                    end
                                end
                                default: begin   // W_FEED
                                    if (acc_in_ready) begin
                                        // accum 本拍采样（acc_in_valid 组合持高）
                                        if (w_x >= {8'd0, win_last}) begin
                                            w_x <= 16'd0;
                                            if (w_y >= {8'd0, win_last}) begin
                                                // 窗口收齐 → solve
                                                nst <= SV0;
                                                ist <= I_SOLVE;
                                            end else begin
                                                w_y <= w_y + 16'd1;
                                                wst <= W_INIT;
                                            end
                                        end else begin
                                            w_x <= w_x + 16'd1;
                                            wst <= W_INIT;
                                        end
                                    end
                                end
                            endcase
                        end
                        I_SOLVE: begin
                            case (nst)
                                SV0: begin
                                    if (acc_out_valid) begin
                                        ts_start_p <= 1'b1;
                                        nst <= SV1;
                                    end
                                end
                                SV1: begin
                                    // 注意：tensor_solve 的 done 在两次 start 之间保持 1，
                                    // 必须先确认 start 已接受（done 拉低）再等本次 done。
                                    if (!solve_started) begin
                                        if (!ts_done) begin
                                            solve_started <= 1'b1;   // start 已接受
                                        end
                                    end else if (ts_done) begin
                                        solve_started <= 1'b0;
                                        if (!ts_out_ok) begin
                                            // solve 失败：回滚出点
                                            reliable <= 1'b0;
                                            state <= S_OUT;
                                        end else begin
                                            ts_dx_r <= ts_out_dx;
                                            ts_dy_r <= ts_out_dy;
                                            nst <= SV2;
                                        end
                                    end
                                end
                                SV2: begin
                                    // 发 next_x 的 fp64_add + 收敛 dx²/dy²（组合）
                                    nst <= SV3;
                                end
                                SV3: nst <= SV4;
                                SV4: begin
                                    if (anxt_v && cmul_v1 && cmul_v2) begin
                                        nextx64_r <= anxt_r;
                                        cmulx_r <= cmul_r1;
                                        cmuly_r <= cmul_r2;
                                        nst <= SV5;
                                    end
                                end
                                SV5: begin
                                    // 发 f64_to_f32(next_x)（组合）
                                    nst <= SV6;
                                end
                                SV6: begin
                                    if (cvt_v) begin
                                        next_x <= cvt_r;
                                        nst <= SV7;
                                    end
                                end
                                SV7: begin
                                    // 发 next_y 的 fp64_add（组合）
                                    nst <= SV8;
                                end
                                SV8: begin
                                    if (anxt_v) begin
                                        // 同拍发 f64_to_f32(next_y)（操作数 anxt_r 有效）
                                        nst <= SV9;
                                    end
                                end
                                default: begin   // SV9
                                    if (cvt_v) begin
                                        next_y <= cvt_r;
                                        ust <= U0;
                                        ist <= I_UPDATE;
                                    end
                                end
                            endcase
                        end
                        I_UPDATE: begin
                            case (ust)
                                U0: begin
                                    // isfinite(next) 检查；发 hypot 差分减 + 收敛和（组合）
                                    if (next_x[30:23] == 8'hFF || next_y[30:23] == 8'hFF) begin
                                        reliable <= 1'b0;
                                        state <= S_OUT;
                                    end else begin
                                        ust <= U1;
                                    end
                                end
                                U1: begin
                                    if (sub_gx_v && sub_gy_v && acnv_v) begin
                                        conv_sum_r <= acnv_r;
                                        ust <= U2;      // 同拍发 hypot（组合）
                                    end
                                end
                                default: begin   // U2
                                    if (hyp_v) begin
                                        if ($unsigned(hyp_r) > $unsigned(radius_f)) begin
                                            // 漂移超半径：失败回滚
                                            reliable <= 1'b0;
                                            state <= S_OUT;
                                        end else begin
                                            cur_x <= next_x;
                                            cur_y <= next_y;
                                            if ($unsigned(conv_sum_r) < C_1E6) begin
                                                // 收敛
                                                reliable <= 1'b1;
                                                conv_ok <= 1'b1;
                                                state <= S_OUT;
                                            end else begin
                                                if (iter >= 7'd39) begin
                                                    // 40 轮未收敛：保留 original
                                                    reliable <= 1'b0;
                                                    state <= S_OUT;
                                                end else begin
                                                    iter <= iter + 7'd1;
                                                    ist <= I_NEXT;
                                                end
                                            end
                                        end
                                    end
                                end
                            endcase
                        end
                        default: begin   // I_NEXT
                            ist <= I_BOUND;
                        end
                    endcase
                end
                S_OUT: begin
                    if (!out_valid) begin
                        out_valid <= 1'b1;
                        out_x <= reliable ? cur_x : orig_x;
                        out_y <= reliable ? cur_y : orig_y;
                        out_reliable <= reliable;
                    end else if (out_ready) begin
                        out_valid <= 1'b0;
                        if (pt_idx + 16'd1 >= n_in) begin
                            state <= S_DONE;
                        end else begin
                            pt_idx <= pt_idx + 16'd1;
                            ld_st <= 2'd0;
                            state <= S_LOAD;
                        end
                    end
                end
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
