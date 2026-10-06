`timescale 1ns / 1ps
//==============================================================================
// bgr_to_gray.v — BGR888 → 灰度（整数精确，与 kernels::bgr_to_gray 逐位一致）
//------------------------------------------------------------------------------
// 公式（kernels/color.h，整数截断）：
//   gray = (299*R + 587*G + 114*B + 500) / 1000
// 输入顺序注意：接口按 B,G,R 三字段给出（DDR 字节流顺序 BGR），
// 权重对应 299*R、587*G、114*B。
//
// 数值范围：299*255+587*255+114*255+500 = 255500 < 2^18，19 位无符号足够。
// 常数除法由综合器实现；行为仿真与 C++ 整数除法（截断）一致。
//
// 接口：valid/ready 直通（1 拍寄存延迟）。in_valid 拍 in_ready=1 时吞入，
// 下一拍 out_valid=1 给出结果。无内部缓冲（ready 组合透传）。
//==============================================================================
module bgr_to_gray (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       in_valid,
    output wire       in_ready,
    input  wire [7:0] in_b,
    input  wire [7:0] in_g,
    input  wire [7:0] in_r,
    output reg        out_valid,
    input  wire       out_ready,
    output reg  [7:0] out_gray
);

    // 无输出反压时 ready 恒 1（调用方保证不溢出；内部仅 1 拍）
    assign in_ready = out_ready;

    wire [18:0] acc = 19'd500
                    + 19'(19'd299 * in_r)
                    + 19'(19'd587 * in_g)
                    + 19'(19'd114 * in_b);
    wire [7:0] gray = acc / 19'd1000;   // 常数除法，截断语义与 C++ 一致

    always @(posedge clk) begin
        if (!rst_n)
            out_valid <= 1'b0;
        else if (in_valid && in_ready)
            out_valid <= 1'b1;
        else if (out_ready)
            out_valid <= 1'b0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            out_gray <= 8'd0;
        end else if (in_valid && in_ready) begin
            out_gray <= gray;
        end
    end

endmodule
