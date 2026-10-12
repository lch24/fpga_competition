`timescale 1ns/1ps
module tb_packed_map;
 reg clk=0;always #5 clk=~clk;reg rst=0,cmd=0,rsp_ready=0;
 wire ready,rsp;wire [7:0] status;reg [287:0] params=0;
 wire wv,wready,dv,dready,br;wire [31:0] addr,len,data;wire [15:0] tag;wire [3:0] keep;wire last;
 reg completion=0,fault=0;reg [31:0] address;reg pending=0;integer ticks=0,writes=0,i;
 reg [31:0] memory[0:23];
 assign wready=!pending&&!completion&&ticks%3!=0;assign dready=pending&&ticks%5!=0;
 packed_map_builder dut(.clk(clk),.rst_n(rst),.cmd_valid(cmd),.cmd_ready(ready),.cmd_opcode(2'd1),.cmd_params(params),
  .cmd_width(16'd4),.cmd_height(16'd3),.cmd_map_base(32'h20000),.rsp_valid(rsp),.rsp_ready(rsp_ready),.rsp_status(status),
  .wr_valid(wv),.wr_ready(wready),.wr_addr(addr),.wr_len(len),.wr_tag(tag),
  .w_valid(dv),.w_ready(dready),.w_data(data),.w_keep(keep),.w_last(last),
  .b_valid(completion),.b_ready(br),.b_tag(16'b0),.b_error(fault));
 always @(posedge clk)if(rst)begin
  ticks<=ticks+1;
  if(wv&&wready)begin if(len!=4||tag!=0)$fatal(1,"map command format");address<=addr;pending<=1;end
  if(dv&&dready)begin
   if(keep!=15||!last)$fatal(1,"map write format");memory[(address-32'h20000)/4]<=data;writes<=writes+1;pending<=0;completion<=1;
  end
  if(completion&&br)completion<=0;
 end
 task start;
  begin @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;end
 endtask
 task response(input [7:0] code);
  begin wait(rsp);repeat(3)begin @(negedge clk);if(!rsp||status!=code)$fatal(1,"map response status/hold %d",status);end
   rsp_ready=1;@(negedge clk);rsp_ready=0;end
 endtask
 reg [3:0] mask;reg [63:0] expected;integer x,y;
 initial begin
  params[31:0]=32'h3f800000;params[63:32]=32'h3f800000;
  repeat(4)@(negedge clk);rst=1;start;response(0);
  if(writes!=24)$fatal(1,"duplicate/missing coordinates writes=%d",writes);
  for(i=0;i<12;i=i+1)begin
   x=i%4;y=i/4;mask={y<2&&x<3,y<2,x<3,1'b1};expected={mask,14'(y),14'(x),32'b0};
   if({memory[2*i+1],memory[2*i]}!==expected)$fatal(1,"packed map pixel %d got=%h expected=%h",i,{memory[2*i+1],memory[2*i]},expected);
  end
  params[31:0]=32'h7fc00000;start;response(1);if(writes!=24)$fatal(1,"invalid params wrote map");
  params[31:0]=32'h3f800000;fault=1;start;response(5);
  $display("PASS packed_map identity/border descriptors, single issue, held responses, invalid parameters and write-error drain");$finish;
 end
 initial begin #10000000;$fatal(1,"map timeout state=%d core=%d",dut.state,dut.core.state);end
endmodule
