`timescale 1ns / 1ps
//==============================================================================
// gray_scan.v — 灰度扫描基础件（M6.1，赛题2 畸变矫正前级）
//------------------------------------------------------------------------------
// 功能：字节流输入（C=3 时每像素 B,G,R 三字节；C=1 时每像素 1 字节）→
//       灰度像素流（光栅序，共 W*H 个）。
//
// 公式（kernels/color.h，整数截断）：
//   gray = (299*R + 587*G + 114*B + 500) / 1000
// 输入字节序 B,G,R（DDR 字节流顺序；bgr_to_gray 核接口同序）。
// C=1 时输入已是灰度，直接复制。
//
// 接口语义：
//   start    单拍脉冲（busy=0 时才可拉），锁存 cfg_w/cfg_h/cfg_c1，启动一帧。
//   busy     start 拉起到 done 有效。
//   done     电平保持置 1（W*H*C 或 W*H 字节消费完且流水排空），下次 start 清零。
//   in_valid/in_ready  字节流握手；in_ready=0 时源必须停顿（反压不丢数据）。
//   out_valid/out_ready 灰度像素流握手（光栅序）。
//
// 时序：
//   C=3：3 字节收集器——前 2 字节（B,G）无条件吸收（in_ready 恒 1）缓存，
//        第 3 字节（R）阶段 in_ready = 核 in_ready（= 核 out_ready = 模块
//        out_ready，组合直通）；核接受拍后下一拍 out_valid=1 输出灰度。
//   C=1：仿核 out_valid 逻辑——接受拍后下一拍 out_valid=1，out_ready 时
//        清零；in_ready = !pending || out_ready。
//   两路径输出用 cfg_c1 组合选通（out_valid/out_gray 多路选择）。
//   done = (byte_cnt >= W*H*C 或 W*H) && 流水排空（核 out_valid=0 / pending=0）。
//   W=0 或 H=0 时 start 后 1 拍即 done。
//==============================================================================
module gray_scan #(
    parameter MAX_W = 2048,
    parameter MAX_H = 2048
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,       // 单拍脉冲，锁存配置（busy=0 时才可拉）
    input  wire [10:0]       cfg_w,       // 像素宽
    input  wire [10:0]       cfg_h,       // 像素高
    input  wire              cfg_c1,      // 1=C=1 直接复制（1 字节/像素）；0=BGR888 转换（3 字节/像素）
    output reg               busy,
    output reg               done,        // 电平保持置1（下次 start 清零）
    // 字节流输入：C=3 时每像素 3 字节按 B,G,R 顺序；C=1 时每像素 1 字节
    input  wire              in_valid,
    output wire              in_ready,
    input  wire [7:0]        in_byte,
    // 灰度像素流输出（光栅序，共 W*H 个）
    output reg               out_valid,
    input  wire              out_ready,
    output reg  [7:0]        out_gray
);

    //--------------------------------------------------------------------
    // 配置锁存（start 拍采样）与总量计算
    //--------------------------------------------------------------------
    reg [10:0] w_r, h_r;
    reg        c1_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            w_r  <= 11'd0;
            h_r  <= 11'd0;
            c1_r <= 1'b0;
        end else if (start) begin
            w_r  <= cfg_w;
            h_r  <= cfg_h;
            c1_r <= cfg_c1;
        end
    end

    // 宽度：W,H<=2047 → W*H<=4190209(22b)，W*H*3<=12570627(24b)，24 位足够
    wire [23:0] pix_total  = w_r * h_r;                           // W*H
    wire [23:0] byte_total = c1_r ? pix_total : (pix_total * 24'd3); // W*H*C

    //--------------------------------------------------------------------
    // 字节计数（C=3 每像素 3 字节、C=1 每像素 1 字节）
    //--------------------------------------------------------------------
    reg [23:0] byte_cnt;

    wire accept_byte = in_valid && in_ready;

    always @(posedge clk) begin
        if (!rst_n)
            byte_cnt <= 24'd0;
        else if (start)
            byte_cnt <= 24'd0;
        else if (accept_byte)
            byte_cnt <= byte_cnt + 1'b1;
    end

    //--------------------------------------------------------------------
    // C=3：3 字节收集器（phase 0=B 1=G 2=R）
    //--------------------------------------------------------------------
    reg [1:0] phase;
    reg [7:0] b_reg, g_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            phase <= 2'd0;
            b_reg <= 8'd0;
            g_reg <= 8'd0;
        end else if (start) begin
            phase <= 2'd0;
            b_reg <= 8'd0;
            g_reg <= 8'd0;
        end else if (!c1_r && busy) begin
            case (phase)
                2'd0:   if (accept_byte) begin b_reg <= in_byte; phase <= 2'd1; end
                2'd1:   if (accept_byte) begin g_reg <= in_byte; phase <= 2'd2; end
                default: if (accept_byte) phase <= 2'd0;  // 第 3 字节：核接受后回 phase 0
            endcase
        end
    end

    //--------------------------------------------------------------------
    // C=1：pending 1 拍流水（仿 bgr_to_gray 核的 out_valid 逻辑）
    //--------------------------------------------------------------------
    reg        c1_pending;
    reg [7:0]  c1_gray;

    always @(posedge clk) begin
        if (!rst_n) begin
            c1_pending <= 1'b0;
            c1_gray    <= 8'd0;
        end else if (start) begin
            c1_pending <= 1'b0;
            c1_gray    <= 8'd0;
        end else begin
            if (c1_r && busy && accept_byte)
                c1_pending <= 1'b1;        // 接受拍 → 下一拍 out_valid=1
            else if (out_ready)
                c1_pending <= 1'b0;        // out_ready 时清零
            if (c1_r && busy && accept_byte)
                c1_gray <= in_byte;        // 输出数据锁存
        end
    end

    //--------------------------------------------------------------------
    // bgr_to_gray 核信号（C=3 路径）
    //   core_in_valid 要求模块 in_valid（源真有数据）才有效，避免 phase 2
    //   窗口内吞入无效字节；in_ready 组合直通 out_ready（核内 assign）。
    //--------------------------------------------------------------------
    wire bytes_ok     = (byte_cnt < byte_total);
    wire       core_in_valid;
    wire       core_in_ready;
    wire       core_out_valid;
    wire [7:0] core_out_gray;

    assign core_in_valid = in_valid && busy && !c1_r && (phase == 2'd2) && bytes_ok;

    //--------------------------------------------------------------------
    // 输入握手（组合）
    //   C=3：phase 0/1 恒 1；phase 2 = 核 in_ready（=out_ready）
    //   C=1：!pending || out_ready
    //   均叠加 busy 与 bytes_ok（消费完不再接收）
    //--------------------------------------------------------------------
    wire c3_in_ready  = (phase == 2'd2) ? core_in_ready : 1'b1;
    wire c1_in_ready  = (!c1_pending || out_ready);
    assign in_ready   = busy && bytes_ok && (c1_r ? c1_in_ready : c3_in_ready);

    bgr_to_gray u_bgr (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (core_in_valid),
        .in_ready  (core_in_ready),
        .in_b      (b_reg),
        .in_g      (g_reg),
        .in_r      (in_byte),
        .out_valid (core_out_valid),
        .out_ready (out_ready),
        .out_gray  (core_out_gray)
    );

    //--------------------------------------------------------------------
    // 输出选通（cfg_c1 多路选择）
    //--------------------------------------------------------------------
    always @* begin
        if (c1_r) begin
            out_valid = c1_pending;
            out_gray  = c1_gray;
        end else begin
            out_valid = core_out_valid;
            out_gray  = core_out_gray;
        end
    end

    //--------------------------------------------------------------------
    // busy / done
    //   done = 字节消费完 && 流水排空（最后一个输出已被 out_ready 接受）
    //--------------------------------------------------------------------
    wire c3_pipe_empty = !core_out_valid;
    wire c1_pipe_empty = !c1_pending;
    wire pipe_empty    = c1_r ? c1_pipe_empty : c3_pipe_empty;
    wire finish_cond   = busy && (byte_cnt >= byte_total) && pipe_empty;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
        end else if (start) begin
            busy <= 1'b1;
            done <= 1'b0;
        end else if (finish_cond) begin
            busy <= 1'b0;
            done <= 1'b1;
        end
    end

endmodule
