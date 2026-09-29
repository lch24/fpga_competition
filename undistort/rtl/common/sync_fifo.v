module sync_fifo #(parameter WIDTH=32,ADDR_BITS=4)(
 input wire clk,rst_n,input wire in_valid,output wire in_ready,input wire [WIDTH-1:0] in_data,
 output wire out_valid,input wire out_ready,output wire [WIDTH-1:0] out_data
);
 reg [WIDTH-1:0] mem[0:(1<<ADDR_BITS)-1];reg [ADDR_BITS-1:0] wp,rp;reg [ADDR_BITS:0] count;
 assign out_valid=count!=0;assign in_ready=count!=(1<<ADDR_BITS);assign out_data=mem[rp];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin wp<=0;rp<=0;count<=0;end
  else begin
   if(in_valid&&in_ready)begin mem[wp]<=in_data;wp<=wp+1'b1;end
   if(out_valid&&out_ready)rp<=rp+1'b1;
   case({in_valid&&in_ready,out_valid&&out_ready})
    2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:count<=count;
   endcase
  end
 end
endmodule
