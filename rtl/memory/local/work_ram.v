// 单写、单同步读工作 RAM。读延迟一拍；同址读写的值不作接口保证。
// 不复位存储阵列：调用方必须先写后读，用状态/有效位取消旧事务。
// 无旁路、无隐含多读端口；多个操作数须分拍读取。
module work_ram #(parameter WIDTH=64, ADDR_BITS=6) (
    input wire clk,
    input wire wr_en,
    input wire [ADDR_BITS-1:0] wr_addr,
    input wire [WIDTH-1:0] wr_data,
    input wire rd_en,
    input wire [ADDR_BITS-1:0] rd_addr,
    output reg [WIDTH-1:0] rd_data
);
    reg [WIDTH-1:0] words [0:(1<<ADDR_BITS)-1];
    always @(posedge clk) begin
        if(wr_en) words[wr_addr] <= wr_data;
        if(rd_en) rd_data <= words[rd_addr];
    end
endmodule
