`timescale 1ns / 1ps
//==============================================================================
// tensor_window_sum.v — 张量 3×3 窗口累加（A/B/C，图外贡献为零）
//------------------------------------------------------------------------------
// 复刻 shi_tomasi.cpp 的张量窗累加：
//   对每个像素 (x,y)：A = Σxx、B = Σxy、C = Σyy，邻域越界时梯度按 0
//   （注意：与 Sobel 灰度窗的 CLAMP 边界不同，此处为 ZERO 边界——
//   规划 5.2 明确两套规则不能混用）。
//
// 结构：例化 window3x3 #(CH=3, DW=32, BORDER_CLAMP=0)，三通道张量流
//   （in_data = {yy, xy, xx}）生成 3×3 窗口，再对 9 个窗口位置的每通道
//   求和。求和用加法树：所有项为精确整数且部分和 < 2^24，与 C++ 的
//   f32 顺序累加结果逐位一致（f32 在此值域内精确）。
//
// 数值：|Σxx|,|Σyy| ≤ 9×1040400 = 9363600 < 2^24，s32 足够（规划 §10）。
//
// 时序：窗口生成 3 拍 + 求和树 1 拍；接口为标准 valid/ready 流。
//==============================================================================
module tensor_window_sum #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
) (
    input  wire        clk,
    input  wire        rst_n,
    // 张量流输入（tensor_core 输出）
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [31:0] in_xx,
    input  wire [31:0] in_xy,
    input  wire [31:0] in_yy,
    input  wire [10:0] in_x,
    input  wire [10:0] in_y,
    // 窗口和输出
    output wire        out_valid,
    input  wire        out_ready,
    output wire [31:0] out_a,       // Σxx
    output wire [31:0] out_b,       // Σxy
    output wire [31:0] out_c,       // Σyy
    output wire [10:0] out_x,
    output wire [10:0] out_y
);

    localparam CW = 96;   // 3 通道 × 32bit

    //--------------------------------------------------------------------
    // 窗口生成（ZERO 边界）
    //--------------------------------------------------------------------
    wire        win_valid;
    wire [10:0] win_x, win_y;
    wire [9*CW-1:0] win_data;

    window3x3 #(
        .CH           (3),
        .DW           (32),
        .IMG_W        (IMG_W),
        .IMG_H        (IMG_H),
        .BORDER_CLAMP (1'b0)
    ) u_win (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (in_valid),
        .in_ready  (in_ready),
        .in_data   ({in_yy, in_xy, in_xx}),   // ch0=xx, ch1=xy, ch2=yy
        .out_valid (win_valid),
        .out_ready (out_ready),
        .out_data  (win_data),
        .out_x     (win_x),
        .out_y     (win_y)
    );

    //--------------------------------------------------------------------
    // 9 项求和（每通道加法树；k = r*3+c，组内通道 i 在 [i*32 +: 32]）
    //--------------------------------------------------------------------
    function [31:0] ch_sum;
        input [9*CW-1:0] w;
        input integer ch;
        reg [31:0] s0, s1;
        integer k;
        begin
            s0 = 32'd0;
            s1 = 32'd0;
            for (k = 0; k < 9; k = k + 2)
                s0 = s0 + w[k*96 + ch*32 +: 32];
            for (k = 1; k < 9; k = k + 2)
                s1 = s1 + w[k*96 + ch*32 +: 32];
            ch_sum = s0 + s1;
        end
    endfunction

    assign out_a = ch_sum(win_data, 0);
    assign out_b = ch_sum(win_data, 1);
    assign out_c = ch_sum(win_data, 2);
    assign out_x = win_x;
    assign out_y = win_y;
    assign out_valid = win_valid;

endmodule
