`timescale 1ns / 1ps
//==============================================================================
// dual_port_ram.v — 同步读双口 RAM 封装（写口 + 读口，同一时钟）
//------------------------------------------------------------------------------
// 用途：候选点存储、行缓存、窗口缓存、矩阵 RAM 等所有需要"一处写、
// 一处读"的片上存储。统一封装，保证各模块对 RAM 语义的理解一致。
//
// 参数：
//   DATA_WIDTH : 数据位宽（bit）
//   ADDR_WIDTH : 地址位宽；深度固定为 2**ADDR_WIDTH（仅支持 2 的幂深度）
//
// 接口：
//   clk     : 时钟（读写同一时钟域）
//   rst_n   : 低有效复位（同步释放，建议来自 reset_sync）。仅清零读数据
//             寄存器；存储阵列内容不受复位影响（BRAM 无法复位）。
//   wr_en / wr_addr / wr_data : 写口
//   rd_en / rd_addr          : 读口
//   rd_data                  : 读数据输出
//
// 时序语义（重要，调用方必须按此设计）：
//   1) 读延迟 = 1 拍：rd_en=1 的下一拍，rd_data 上出现 mem[rd_addr]。
//   2) rd_en=0 时 rd_data 保持上一次的值（不更新）。
//   3) 同址同拍读写（wr_addr==rd_addr 且双使能）：读回"写之前的旧值"
//      （read-first 语义）。这是综合器推断 BRAM 的标准模板，
//      不要在调用方假设能读到当拍新值。
//   4) 写口与读口完全独立，允许同拍各自寻址不同地址。
//
// 无 valid/ready 流控：本模块是纯存储原语，流控由调用方控制器负责。
//==============================================================================
module dual_port_ram #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 10
) (
    input  wire                   clk,
    input  wire                   rst_n,
    // 写口
    input  wire                   wr_en,
    input  wire [ADDR_WIDTH-1:0]  wr_addr,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    // 读口（同步读，1 拍延迟）
    input  wire                   rd_en,
    input  wire [ADDR_WIDTH-1:0]  rd_addr,
    output reg  [DATA_WIDTH-1:0]  rd_data
);

    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    // 写口：写使能时更新存储
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // 读口：read-first 模板（同址读写时读旧值）
    always @(posedge clk) begin
        if (!rst_n)
            rd_data <= {DATA_WIDTH{1'b0}};
        else if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule
