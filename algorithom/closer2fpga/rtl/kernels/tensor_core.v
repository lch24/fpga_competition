`timescale 1ns / 1ps
//==============================================================================
// tensor_core.v — 梯度外积核（xx/xy/yy，与 kernels::outer_product 一致）
//------------------------------------------------------------------------------
// 公式：xx = gx*gx，xy = gx*gy，yy = gy*gy（weight=1）
// 数值：|gx|,|gy| ≤ 1020 → |xx|,|yy| ≤ 1040400，|xy| 同；s32 足够（规划 §10）。
// C++ 侧 float 乘法但值为精确整数（< 2^24）→ RTL s32 与 f32 位模式对应。
//
// 时序：1 拍寄存；in_ready = out_ready 透传背压。
//==============================================================================
module tensor_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [15:0] in_gx,
    input  wire [15:0] in_gy,
    input  wire [10:0] in_x,
    input  wire [10:0] in_y,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [31:0] out_xx,
    output reg  [31:0] out_xy,
    output reg  [31:0] out_yy,
    output reg  [10:0] out_x,
    output reg  [10:0] out_y
);

    assign in_ready = out_ready;

    wire signed [15:0] gx = $signed(in_gx);
    wire signed [15:0] gy = $signed(in_gy);

    wire signed [31:0] xx = gx * gx;
    wire signed [31:0] xy = gx * gy;
    wire signed [31:0] yy = gy * gy;

    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_xx    <= 32'd0;
            out_xy    <= 32'd0;
            out_yy    <= 32'd0;
            out_x     <= 11'd0;
            out_y     <= 11'd0;
        end else if (in_valid && in_ready) begin
            out_valid <= 1'b1;
            out_xx    <= xx;
            out_xy    <= xy;
            out_yy    <= yy;
            out_x     <= in_x;
            out_y     <= in_y;
        end else if (out_ready) begin
            out_valid <= 1'b0;
        end
    end

endmodule
