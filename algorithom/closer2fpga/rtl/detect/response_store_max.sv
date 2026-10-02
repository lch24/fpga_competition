`timescale 1ns / 1ps
//==============================================================================
// response_store_max.sv — Pass1 resp 流存储 + 全图实时 max（响应结果 RAM）
//------------------------------------------------------------------------------
// 作用：Pass1 阶段以光栅序顺序写入 PIXELS 个 32 位 resp，同时在写流上进行
//       逐拍 fp32 最大值跟踪（rmax，对应 C++ 的 rmax=max(resp 全图)）。
// Pass2 阶段通过读口（registered read，延迟 1 拍）按地址任意读回 resp。
//
// 说明：本模块不做任何 fp32 乘/加，只做"整数位序变换 + 无符号比较"的 fp32
//       比较（非 NaN 恒等价 IEEE 比较），属于位比较而非算术模块。
//
// fp32 比较位技巧（关键，与 C++ IEEE 逐位一致）：
//     ord(a) = a[31] ? ~a : (a | 32'h8000_0000)
//   以 32 位无符号整数比较 ord 的"大于/大于等于"，即得 IEEE 浮点顺序。
//   这是标准 Radix/fpsort 变换，对正负号、大小关系（含 -0==+0）均成立。
//
// 实时 max 初值取 -inf（32'hFF800000）。
//==============================================================================
module response_store_max #(
    parameter PIXELS = 921600,
    parameter ADDR_W = $clog2(PIXELS)
) (
    input                                   clk,
    input                                   rst_n,
    // Pass1 resp 流
    input                                   in_valid,
    output                                  in_ready,
    input  [31:0]                           in_resp,
    // 帧结束 / rmax
    output reg                              pass1_done,   // 第 PIXELS 个接受后的下一拍脉冲 1
    output reg [31:0]                       rmax,
    // Pass2 读口（registered read，延迟 1 拍）
    input  [ADDR_W-1:0]                     rd_addr,
    output reg [31:0]                       rd_data
);

    // fp32 -> 整数序位变换（无符号比较即 IEEE 顺序）
    function automatic [31:0] fp_ord(input [31:0] a);
        fp_ord = a[31] ? ~a : (a | 32'h8000_0000);
    endfunction

    // 写地址计数（第 0..PIXELS-1 个），宽度 +1 避免满帧溢出混淆
    reg [ADDR_W:0] wcnt;

    assign in_ready = 1'b1;                 // 恒 1（前端直接推流）

    //--------------------------------------------------------------------
    // 双口 RAM：写口 Pass1 顺序写，读口 Pass2 registered 读
    //--------------------------------------------------------------------
    dual_port_ram #(
        .DATA_WIDTH (32),
        .ADDR_WIDTH (ADDR_W)
    ) u_ram (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (in_valid),               // in_ready 恒 1，写即接受
        .wr_addr  (wcnt[ADDR_W-1:0]),
        .wr_data  (in_resp),
        .rd_en    (1'b1),                   // 恒使能 = registered latency-1 读
        .rd_addr  (rd_addr),
        .rd_data  (rd_data)
    );

    //--------------------------------------------------------------------
    // rmax 逐拍更新 + pass1_done 脉冲
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            wcnt       <= {(ADDR_W+1){1'b0}};
            rmax       <= 32'hFF80_0000;    // -inf
            pass1_done <= 1'b0;
        end else begin
            // pass1_done：收满后持续拉高（电平而非单拍脉冲）——单拍脉冲在
            //   ctrl 的 posedge 采样竞态下会被错过（ctrl 读到的是更新前旧值，
            //   下一拍脉冲已回 0），导致 PASS1 永远无法转移的死锁。
            pass1_done <= (wcnt >= PIXELS);

            if (in_valid) begin
                wcnt <= wcnt + 1'b1;
                // 逐拍 max（位比较）
                if ($unsigned(fp_ord(in_resp)) > $unsigned(fp_ord(rmax)))
                    rmax <= in_resp;
            end
        end
    end

endmodule