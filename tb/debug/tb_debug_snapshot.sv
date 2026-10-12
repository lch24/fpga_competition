`timescale 1ns/1ps
module tb_debug_snapshot;
 reg clk=0,src=0,rst=0,run_src=1;always #5 clk=~clk;always #7 if(run_src)src=~src;
 reg [31:0] count=0;always @(posedge src)count<=count+1;
 wire [63:0] data;wire update;wire [15:0] arg,tag;
 reg [15:0] requested=0;
 reg finished=0;integer updates=0;reg [63:0] previous;
 debug_snapshot #(.WIDTH(64)) dut(.clk(clk),.rst_n(rst),.poll(1'b1),.argument(requested),
  .data(data),.tag(tag),.update(update),.src_clk(src),.src_data({~count,count}),.src_argument(arg));
 always @(negedge clk)if(update)begin
  if(data[63:32]!==~data[31:0])$fatal(1,"torn snapshot");
  updates=updates+1;requested<=requested+1;
 end
 initial begin
  #20;rst=1;wait(updates==1000);
  @(negedge src);run_src=0;repeat(50)@(posedge clk);previous=data;
  repeat(100)@(posedge clk);if(data!==previous)$fatal(1,"stopped source mutated snapshot");
  run_src=1;wait(updates==1100);
  rst=0;#30;rst=1;wait(updates==1200);
  $display("PASS tb_debug_snapshot: coherent bundled data, stop, recovery, POR");finished=1;$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
