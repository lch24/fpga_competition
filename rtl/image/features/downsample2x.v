`timescale 1ns / 1ps
//==============================================================================
// downsample2x.v — 金字塔 2x 降采样（行缓存 + 列对均值），M6.1
//------------------------------------------------------------------------------
// 公式（chessboard.cpp::detect_chessboard 缩图分支，位级权威）：
//     half(x,y) = (a + b + c + d + 2) >> 2     （整数 floor 除法）
//   a = src(2x, 2y)    b = src(2x+1, 2y)
//   c = src(2x, 2y+1)  d = src(2x+1, 2y+1)
//   输出尺寸 (W/2)×(H/2)；奇数末行/末列不参与（源多余像素被消费但丢弃）。
//
// 接口语义：
//   start        ：单拍脉冲，busy=0 时才可拉，锁存 cfg_w/cfg_h 并置 busy
//   busy         ：start 接受后置 1；全部输入消费完且（如有）最后一个输出
//                  被 out_ready 接受后置 0
//   done         ：电平保持置 1（下次 start 清零），避免单拍脉冲被漏采
//   in_valid/in_ready/in_gray ：源灰度流（光栅序，行内 x 递增，行间 y 递增）
//   out_valid/out_ready/out_gray ：缩图像素流（光栅序，(W/2)×(H/2)）
//
// 架构：单行缓存 line_mem（reg 数组，W 字节，registered 读延迟 1 拍，
// 语义与 rtl/common/dual_port_ram.v 一致：rd_addr 变化后 1 拍 rd_data 出数，
// 同址同拍读写读旧值）。偶数输入行逐像素写缓存，不产生输出；奇数输入行与
// 缓存中上一偶数行按列对合并。行 RAM 为单读口、读延迟 1 拍，每个输出需
// a、b 两个缓存读，因此在奇数行内"偶数列→奇数列"对之间插 1 拍气泡
// （in_ready 拉低 1 拍），节拍如下（E_i/O_i 为第 i 对偶/奇数列接受拍）：
//     E_i      ：c_reg <= in_gray（=c），rd_addr <= 2i+1（发起读 b）
//     O_i      ：d_reg <= in_gray（=d），rd_addr <= 2i（发起读 a）
//     O_i+1    ：气泡拍。b_reg <= rd_data（=b），c_del <= c_reg
//     O_i+2    ：计算 out = (rd_data + b_reg + c_del + d_reg + 2) >> 2
//                （此时 rd_data = a），置 out_valid；若输出挂起未接受则延后
// 奇数行自身不写回缓存（下一对使用的缓存行是新的偶数行）。
//
// 反压：out_ready=0 且输出挂起时 in_ready=0（源流停顿，数据保持）；气泡拍
// in_ready=0。停顿期间 rd_addr 寄存器保持 → rd_data 保持 → 不丢数、不欠速。
// 精确消费 W*H 输入、精确产出 (W/2)*(H/2) 输出；W<2 或 H<2 时产出 0 像素
// 但消费全部输入。done 在全部输入消费完且（如有）最后一个输出被 out_ready
// 接受后拉高电平。
//
// 时序（10ns 周期示意，clk posedge 采样）：
//   [idle] start=1 → busy=1 锁存 cfg_w/cfg_h
//   [run]  偶数行 1 像素/拍写缓存；奇数行约 2 像素/3 拍（偶、奇、气泡），
//          输出在奇数列接受后第 2 拍产生，保持至 out_ready 接受
//   [end]  全部消费 + 输出清空 → busy=0, done=1
//==============================================================================
module downsample2x #(parameter USE_CE=0,
    parameter MAX_W = 2048,
    parameter ADDR_W = $clog2(MAX_W)
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,       // 单拍脉冲，锁存配置（busy=0 时才可拉）
    input  wire [ADDR_W-1:0]  cfg_w,       // 源宽像素
    input  wire [ADDR_W-1:0]  cfg_h,       // 源高像素
    output reg                busy,        // start 接受后置1，完成置0
    output reg                done,        // 电平保持置1（下次 start 清零）
    // 源灰度像素流（光栅序：行内 x 递增，行间 y 递增）
    input  wire               in_valid,
    output wire               in_ready,
    input  wire [7:0]         in_gray,
    // 缩图像素流（光栅序，(W/2)×(H/2)）
    output reg                out_valid,
    input  wire               out_ready,
    output reg  [7:0]         out_gray
);

    //--------------------------------------------------------------------
    // 内部寄存器/信号
    //--------------------------------------------------------------------
    reg [ADDR_W-1:0] w_r, h_r;                  // start 锁存的宽/高
    reg [ADDR_W:0]   x, y;                      // 当前输入坐标（+1bit 防溢出）
    reg [7:0]        b_reg, c_reg, c_del, d_reg;// 列对管线锁存（b 读自缓存）
    reg              pending_out;               // 奇数列已接受，输出待计算
    reg              bub;                       // 奇数列后的气泡拍（in_ready=0）
    reg [ADDR_W-1:0] rd_addr_r;                 // 行 RAM 读地址（停顿保持）
    reg [7:0]        rd_data;                   // registered 读数据（延迟 1 拍）
    reg [7:0]        line_mem [0:(1<<ADDR_W)-1];// 行缓存，W 字节

    wire             feed    = busy && in_valid && in_ready; // 本拍消费 1 输入像素
    wire             row_odd = y[0];                          // 当前输入行是奇数行
    wire             x_odd   = x[0];                          // 当前输入列是奇数列
    wire             all_in  = (y == h_r);                    // 全部 W*H 已消费
    wire [ADDR_W:0]  x_m1    = x - 1'b1;                      // 奇数列 → 读 a
    wire [ADDR_W:0]  x_p1    = x + 1'b1;                      // 偶数列 → 读 b

    // 反压：未运行 / 已消费完 / 气泡拍 / 输出挂起未接受 → 不接输入
    assign in_ready = busy && !all_in && !bub && !(out_valid && !out_ready);

    //--------------------------------------------------------------------
    // 行缓存写口：偶数行逐像素写（读优先模板，同址同拍读写读旧值）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if(!USE_CE || ce) begin
        if (feed && !row_odd)
            line_mem[x[ADDR_W-1:0]] <= in_gray;
    end // synchronous clock enable
    end

    // 行缓存读口：registered，延迟 1 拍
    always @(posedge clk) begin
        if (!rst_n) rd_data <= 8'd0;
        else if(!USE_CE || ce) begin        rd_data <= line_mem[rd_addr_r];
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 列对管线：奇数列锁 d、发起读 a；偶数列锁 c、发起读 b；气泡拍锁 b、存 c
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            b_reg     <= 8'd0;
            c_reg     <= 8'd0;
            c_del     <= 8'd0;
            d_reg     <= 8'd0;
            rd_addr_r <= {ADDR_W{1'b0}};
        end else if(!USE_CE || ce) begin if (feed && row_odd) begin
            if (x_odd) begin
                d_reg     <= in_gray;
                rd_addr_r <= x_m1[ADDR_W-1:0];   // 读 a = mem[2i]
            end else begin
                c_reg     <= in_gray;
                rd_addr_r <= x_p1[ADDR_W-1:0];   // 读 b = mem[2i+1]
            end
        end else if (bub) begin
            b_reg <= rd_data;                    // rd_data = mem[2i+1] = b
            c_del <= c_reg;                      // 保存 c 供输出计算
        end
    end // synchronous clock enable
    end

    // 气泡拍标志：奇数列接受后置 1，下一拍清零
    always @(posedge clk) begin
        if (!rst_n)                      bub <= 1'b0;
        else if(!USE_CE || ce) begin if (feed && row_odd && x_odd) bub <= 1'b1;
        else                             bub <= 1'b0;
    end // synchronous clock enable
    end

    // 输出待计算标志：奇数列接受置 1，气泡后计算完成清零
    always @(posedge clk) begin
        if (!rst_n)                                  pending_out <= 1'b0;
        else if(!USE_CE || ce) begin if (feed && row_odd && x_odd)           pending_out <= 1'b1;
        else if (pending_out && !bub && !(out_valid && !out_ready))
                                                     pending_out <= 1'b0;
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 输出：气泡拍后计算 out = (a+b+c+d+2)>>2（a=rd_data），置 out_valid
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_gray  <= 8'd0;
        end else if(!USE_CE || ce) begin if (pending_out && !bub && !(out_valid && !out_ready)) begin
            out_valid <= 1'b1;                      // 新输出优先于接受清零
            out_gray  <= (rd_data + b_reg + c_del + d_reg + 2) >> 2;
        end else if (out_valid && out_ready) begin
            out_valid <= 1'b0;
        end
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // 坐标计数：光栅序，行尾回绕；最后一行消费完 y==h_r（all_in）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            x <= {ADDR_W{1'b0}};
            y <= {ADDR_W{1'b0}};
        end else if(!USE_CE || ce) begin if (start && !busy) begin
            x <= {ADDR_W{1'b0}};
            y <= {ADDR_W{1'b0}};
        end else if (feed) begin
            if (x == (w_r - 1'b1)) begin            // 行尾
                x <= {ADDR_W{1'b0}};
                if (y == (h_r - 1'b1)) y <= h_r;    // 最后一行：y==h_r 表示消费完毕
                else                   y <= y + 1'b1;
            end else begin
                x <= x + 1'b1;
            end
        end
    end // synchronous clock enable
    end

    //--------------------------------------------------------------------
    // start / busy / done
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            w_r  <= {ADDR_W{1'b0}};
            h_r  <= {ADDR_W{1'b0}};
        end else if(!USE_CE || ce) begin if (start && !busy) begin
            w_r  <= cfg_w;
            h_r  <= cfg_h;
            busy <= 1'b1;
            done <= 1'b0;
            if (cfg_w == {ADDR_W{1'b0}} || cfg_h == {ADDR_W{1'b0}}) begin
                busy <= 1'b0;                       // 0 尺寸：无输入可消费，立即完成
                done <= 1'b1;
            end
        end else if (busy && all_in && !pending_out && !(out_valid && !out_ready)) begin
            busy <= 1'b0;                           // 全部消费 + 输出清空
            done <= 1'b1;
        end
    end // synchronous clock enable
    end

endmodule
