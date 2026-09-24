`timescale 1ns / 1ps
//==============================================================================
// nms_candidates.sv — 3×3 resp 窗口上的 Shi-Tomasi 角点判定（单点 NMS）
//------------------------------------------------------------------------------
// 输入：window3x3(BORDER_CLAMP=0,CH=1,DW=32) 的 3×3 resp 窗口流（光栅序）。
// 对每个中心 (cx,cy)：
//   - 中心有效范围：cx∈[2,IMG_W-3], cy∈[2,IMG_H-3]（Pass2 内缩/r=1,nr=1）
//   - v=w11 满足：v>0 && v>=thr && 8 个邻居都满足 v>=邻居（相等不抑制）
// 满足则作为一个候选角点，经内部 FIFO 缓冲后按窗口流顺序（=光栅序）输出。
//
// fp32 比较（位技巧，非算术模块）：ord(a)=a[31]?~a:(a|0x8000_0000)，用
// 无符号比较即可得到 IEEE 浮点顺序，与 C++ 逐位一致。thr>0，仅与正比较。
//==============================================================================
module nms_candidates #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
) (
    input                                  clk,
    input                                  rst_n,
    // resp 窗口流
    input                                  win_valid,
    output                                 win_ready,
    input  [9*32-1:0]                      win_data,  // k=r*3+c, k0=左上
    input  [10:0]                          cx,
    input  [10:0]                          cy,
    input  [31:0]                          thr,
    // 角点输出
    output                                 cand_valid,
    input                                  cand_ready,
    output [10:0]                          cand_x,
    output [10:0]                          cand_y
);

    // fp32 -> 整数序位变换
    function automatic [31:0] fp_ord(input [31:0] a);
        fp_ord = a[31] ? ~a : (a | 32'h8000_0000);
    endfunction

    localparam signed [31:0] CX_HI = IMG_W - 3;
    localparam signed [31:0] CY_HI = IMG_H - 3;

    // 窗口像素序（window3x3 约定，k=r*3+c，k0=左上在最高位）：w00..w22
    wire [31:0] w00 = win_data[287:256];
    wire [31:0] w01 = win_data[255:224];
    wire [31:0] w02 = win_data[223:192];
    wire [31:0] w10 = win_data[191:160];
    wire [31:0] w11 = win_data[159:128];   // 中心
    wire [31:0] w12 = win_data[127:96];
    wire [31:0] w20 = win_data[95:64];
    wire [31:0] w21 = win_data[63:32];
    wire [31:0] w22 = win_data[31:0];

    // 中心有效范围（Pass2 内缩）
    wire center_ok =
        ($signed({1'b0, cx}) >= $signed(32'd2)) && ($signed({1'b0, cx}) <= CX_HI) &&
        ($signed({1'b0, cy}) >= $signed(32'd2)) && ($signed({1'b0, cy}) <= CY_HI);

    // 判据（全部用位比较得到 IEEE 顺序）
    wire v_gt_0    = $unsigned(fp_ord(w11)) >  $unsigned(fp_ord(32'h0000_0000));
    wire v_ge_thr  = $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(thr));
    wire v_ge_nei  = // v >= 全部 8 邻居（相等不抑制）
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w00)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w01)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w02)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w10)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w12)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w20)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w21)) &&
        $unsigned(fp_ord(w11)) >= $unsigned(fp_ord(w22));

    wire push = win_valid && win_ready && center_ok && v_gt_0 && v_ge_thr && v_ge_nei;

    //--------------------------------------------------------------------
    // 输出 FIFO（宽 22 = cand_x[21:11] + cand_y[10:0]），win_ready=!full
    //--------------------------------------------------------------------
    wire fifo_in_ready;
    wire [21:0] canddata;

    sync_fifo #(
        .DATA_WIDTH (22),
        .ADDR_WIDTH (4)                    // 深度 16，承压足够
    ) u_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (push),
        .in_ready (fifo_in_ready),
        .in_data  ({cx, cy}),
        .out_valid(cand_valid),
        .out_ready(cand_ready),
        .out_data (canddata),
        .count    (),
        .empty    (),
        .full     ()
    );

    assign win_ready = fifo_in_ready;
    assign cand_x    = canddata[21:11];
    assign cand_y    = canddata[10:0];

endmodule