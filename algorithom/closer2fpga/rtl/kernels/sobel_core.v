`timescale 1ns / 1ps
//==============================================================================
// sobel_core.v — Sobel 梯度核（gx/gy，与 kernels::sobel 逐位一致）
//------------------------------------------------------------------------------
// 公式（kernels/gradient.h）：
//   gx = -tl - 2*ml - bl + tr + 2*mr + br
//   gy = -tl - 2*tc - tr + bl + 2*bc + br
// 输入为 CLAMP 边界的 3×3 灰度窗（window3x3 BORDER_CLAMP=1 输出）。
// 数值：输入 0..255，|gx|,|gy| ≤ 1020，s16 足够（规划 §10）。
// C++ 侧 float 运算但值为精确整数 → RTL s16 与 f32 位模式一一对应。
//
// 时序：1 拍寄存；in_ready = out_ready 透传背压（与 bgr_to_gray 同模式）。
//==============================================================================
module sobel_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [71:0] in_win,     // 9×8bit：k=r*3+c，k4=中心
    input  wire [10:0] in_x,       // 窗口中心坐标（透传 +1 拍）
    input  wire [10:0] in_y,
    output reg         out_valid,
    input  wire        out_ready,
    output reg  [15:0] out_gx,
    output reg  [15:0] out_gy,
    output reg  [10:0] out_x,
    output reg  [10:0] out_y
);

    assign in_ready = out_ready;

    // 窗口拼接序（window3x3 文档约定）：k=r*3+c，k0=左上在最高字节
    wire [7:0] tl = in_win[71:64], tc = in_win[63:56], tr = in_win[55:48];
    wire [7:0] ml = in_win[47:40],                      mr = in_win[31:24];
    wire [7:0] bl = in_win[23:16], bc = in_win[15:8],   br = in_win[7:0];

    wire signed [15:0] gx = -$signed({1'b0, tl}) - 2*$signed({1'b0, ml})
                            - $signed({1'b0, bl}) + $signed({1'b0, tr})
                            + 2*$signed({1'b0, mr}) + $signed({1'b0, br});
    wire signed [15:0] gy = -$signed({1'b0, tl}) - 2*$signed({1'b0, tc})
                            - $signed({1'b0, tr}) + $signed({1'b0, bl})
                            + 2*$signed({1'b0, bc}) + $signed({1'b0, br});

    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_gx    <= 16'd0;
            out_gy    <= 16'd0;
            out_x     <= 11'd0;
            out_y     <= 11'd0;
        end else if (in_valid && in_ready) begin
            out_valid <= 1'b1;
            out_gx    <= gx;
            out_gy    <= gy;
            out_x     <= in_x;
            out_y     <= in_y;
        end else if (out_ready) begin
            out_valid <= 1'b0;
        end
    end

endmodule
