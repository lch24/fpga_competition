`timescale 1ns/1ps
// Grid-mode packed map builder: identity lens, 16x12 image, 4x4 control grid.
// The grid has GW+1=5 columns and GH+1=4 rows (one extra row/column for the
// final interpolation cell), with a padded 64-point row stride.
module tb_grid_map;
 localparam BASE=32'h10000,GW=5,GH=4,STRIDE=64;
 reg clk=0;always #5 clk=~clk;reg rst=0,cmd=0,rsp_ready=0;
 reg [1:0] opcode=1;reg [287:0] params=0;
 wire ready,rsp;wire [7:0] status;
 wire wv,wready,dv,dready,br;wire [31:0] addr,len,data;wire [15:0] tag;wire [3:0] keep;wire last;
 reg completion=0,fault=0;reg [31:0] address;reg pending=0;integer ticks=0,writes=0,i,x,y;
 reg [31:0] memory[0:4095];
 assign wready=!pending&&!completion&&ticks%3!=0;assign dready=pending&&ticks%5!=0;
 packed_map_builder #(.GRID_MAP(1),.MAP_STEP(4),.GRID_STRIDE(STRIDE),.IMAGE_WIDTH(16),.IMAGE_HEIGHT(12)) dut(
  .clk(clk),.rst_n(rst),.cmd_valid(cmd),.cmd_ready(ready),.cmd_opcode(opcode),.cmd_params(params),
  .cmd_width(16'(GW)),.cmd_height(16'(GH)),.cmd_map_base(BASE),
  .rsp_valid(rsp),.rsp_ready(rsp_ready),.rsp_status(status),
  .wr_valid(wv),.wr_ready(wready),.wr_addr(addr),.wr_len(len),.wr_tag(tag),
  .w_valid(dv),.w_ready(dready),.w_data(data),.w_keep(keep),.w_last(last),
  .b_valid(completion),.b_ready(br),.b_tag(16'b0),.b_error(fault));
 always @(posedge clk)if(rst)begin
  ticks<=ticks+1;
  if(wv&&wready)begin if(len!=4||tag!=0)$fatal(1,"map command format");address<=addr;pending<=1;end
  if(dv&&dready)begin
   if(keep!=15||!last)$fatal(1,"map write format");
   memory[(address-BASE)>>2]<=data;writes<=writes+1;pending<=0;completion<=1;
  end
  if(completion&&br)completion<=0;
 end
 task start;
  begin @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;end
 endtask
 task response(input [7:0] code);
  begin wait(rsp);repeat(3)begin @(negedge clk);if(!rsp||status!=code)$fatal(1,"map response status/hold %0d",status);end
   rsp_ready=1;@(negedge clk);rsp_ready=0;end
 endtask
 initial begin
  params[31:0]=32'h3f800000;params[63:32]=32'h3f800000;
  repeat(4)@(negedge clk);rst=1;start;response(0);
  if(writes!=GW*GH*2)$fatal(1,"grid writes=%0d expected %0d",writes,GW*GH*2);
  for(y=0;y<GH;y=y+1)for(x=0;x<GW;x=x+1)begin
   // Identity lens: control point (x*4,y*4) maps to itself.
   if(memory[(y*STRIDE+x)*2]!==((x*4)<<16))$fatal(1,"grid x y=%0d x=%0d got=%h",y,x,memory[(y*STRIDE+x)*2]);
   if(memory[(y*STRIDE+x)*2+1]!==((y*4)<<16))$fatal(1,"grid y y=%0d x=%0d got=%h",y,x,memory[(y*STRIDE+x)*2+1]);
  end
  // Padding must stay untouched; the reader never consumes it.
  if(memory[(0*STRIDE+GW)*2]!==32'hxxxxxxxx)$fatal(1,"padding written at row0");
  $display("PASS grid map stride/format, %0d control points, identity Q16.16",GW*GH);
  $finish;
 end
 initial begin #2000000;$fatal(1,"grid map timeout state=%0d core=%0d",dut.state,dut.core.state);end
endmodule
