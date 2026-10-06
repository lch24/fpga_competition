`timescale 1ns/1ps
module tb_gray_dma;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;wire [7:0] rsp_status;
 reg [15:0] width=7,height=3;reg [31:0] src_base=31,src_stride=19,dst_base=511,dst_stride=11;
 `include "logical_bus.vh"
 rgb565_gray_dma dut(.*);
 `include "logical_memory.vh"
 integer x,y,i,p,r,g,b,expected,transactions=0;
 always @(posedge clk)if(rd_valid&&rd_ready)transactions<=transactions+1;
 task launch;
  begin @(negedge clk);cmd_valid=1;do @(posedge clk);while(!cmd_ready);@(negedge clk);cmd_valid=0;end
 endtask
 task finish_job(input integer status);
  begin wait(rsp_valid);repeat(9)begin @(negedge clk);if(!rsp_valid||rsp_status!=status)$fatal(1,"gray status/backpressure");end
   rsp_ready=1;@(negedge clk);rsp_ready=0;
  end
 endtask
 initial begin
  for(i=0;i<1024;i=i+1)memory.mem[i]=8'ha5;
  for(y=0;y<3;y=y+1)for(x=0;x<7;x=x+1)begin
   p=(x*9137+y*777)&65535;memory.mem[31+y*19+2*x]=p&255;memory.mem[32+y*19+2*x]=p>>8;
  end
  repeat(4)@(negedge clk);rst_n=1;launch();finish_job(0);
  for(y=0;y<3;y=y+1)for(x=0;x<11;x=x+1)begin
   p=(x*9137+y*777)&65535;r=((p>>11)&31)*8;g=((p>>5)&63)*4;b=(p&31)*8;
   expected=x<7?(299*r+587*g+114*b+500)/1000:8'ha5;
   if(memory.mem[511+y*11+x]!==expected[7:0])$fatal(1,"gray/padding mismatch %d %d",x,y);
  end
  if(transactions!=21||violations!=0)$fatal(1,"transaction count/protocol");
  // Exact overlap rejected without a read. Odd addresses and cross-block pixels
  // above exercise the logical byte boundary independently of physical HMIC.
  dst_base=31;launch();finish_job(1);if(transactions!=21)$fatal(1,"bad config read");dst_base=511;
  memory.inj_en=1;memory.inj_addr_min=31;memory.inj_addr_max=32;launch();finish_job(5);
  memory.inj_addr_min=511;memory.inj_addr_max=511;launch();finish_job(5);
  memory.inj_en=0;launch();finish_job(0);
  if(violations!=0)$fatal(1,"memory protocol");
  $display("PASS gray DMA: pixel reference, odd starts, padding, read/write errors, retry and completion backpressure");$finish;
 end
 initial begin #5000000;$fatal(1,"gray timeout");end
endmodule
