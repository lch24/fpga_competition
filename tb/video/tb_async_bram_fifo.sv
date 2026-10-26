`timescale 1ns/1ps
module tb_async_bram_fifo #(parameter ADDR_BITS=3);
 localparam DEPTH=1<<ADDR_BITS;
 reg finished=0;
 reg wclk=0,rclk=0,rst=0,wvalid=0,rready=0;
 always #3 wclk=~wclk;
 always #5 rclk=~rclk;
 reg [31:0] wdata=0;
 wire wready,rvalid;wire [31:0] rdata;wire [ADDR_BITS:0] level;
 async_bram_fifo #(.WIDTH(32),.ADDR_BITS(ADDR_BITS)) dut(
  .wclk(wclk),.wrst_n(rst),.wvalid(wvalid),.wready(wready),.wdata(wdata),
  .rclk(rclk),.rrst_n(rst),.rvalid(rvalid),.rready(rready),.rdata(rdata),.wlevel(level));
 task put(input [31:0] value);
  begin
   @(negedge wclk);wvalid=1;wdata=value;
   do @(posedge wclk);while(!wready);
   @(negedge wclk);wvalid=0;
  end
 endtask
 task get(input [31:0] value);
  begin
   @(negedge rclk);rready=1;
   do @(posedge rclk);while(!rvalid);
   if(rdata!==value)$fatal(1,"FIFO order got %h expected %h",rdata,value);
   @(negedge rclk);rready=0;
  end
 endtask
 integer round_index,i;
 reg [31:0] held;
 initial begin
  repeat(4)@(negedge wclk);rst=1;
  // Exactly DEPTH accepted writes, full backpressure, and uninterrupted reads.
  // Repeat across pointer wraps; the full flag must never allow a DEPTH+1 write.
  for(round_index=0;round_index<3;round_index=round_index+1)begin
   for(i=0;i<DEPTH;i=i+1)put(round_index*DEPTH+i);
   repeat(5)@(negedge rclk);
   if(wready||!rvalid||level!=DEPTH)$fatal(1,"full/capacity mismatch");
   held=rdata;
   @(negedge wclk);wvalid=1;wdata=32'hdeadbeef;
   repeat(5)begin @(negedge wclk);if(wready||rdata!==held)$fatal(1,"full/stall violation");end
   wvalid=0;
   @(negedge rclk);rready=1;
   for(i=0;i<DEPTH;i=i+1)begin
    @(posedge rclk);
    if(!rvalid||rdata!==round_index*DEPTH+i)$fatal(1,"continuous-read bubble/order at %0d",i);
   end
   @(negedge rclk);rready=0;
   repeat(5)@(negedge wclk);
   if(rvalid||!wready||level!=0)$fatal(1,"empty/release mismatch");
  end
  // Concurrent clocks, sparse arrivals, and output stalls.
  fork
   begin for(integer n=0;n<DEPTH*5;n=n+1)begin repeat(n%4)@(negedge wclk);put(32'h10000+n);end end
   begin for(integer n=0;n<DEPTH*5;n=n+1)begin repeat(n%3)@(negedge rclk);get(32'h10000+n);end end
  join
  put(32'habcd);put(32'hef01);repeat(4)@(negedge rclk);
  // Abort an occupied queue with VALID high; old RAM contents must not escape.
  @(negedge wclk);rst=0;wvalid=1;wdata=32'hdeadbeef;
  repeat(5)@(negedge rclk);if(rvalid)$fatal(1,"valid during reset");
  @(negedge wclk);wvalid=0;rst=1;
  put(32'h12345678);get(32'h12345678);
  repeat(5)@(negedge rclk);if(rvalid)$fatal(1,"stale output after reset");
  finished=1;$display("PASS BRAM FIFO depth=%0d: full boundary, stalls, continuous reads, wraps, unrelated clocks, occupied reset",DEPTH);$finish;
 end
 initial begin #(DEPTH*1000+100000);$fatal(1,"BRAM FIFO timeout");end
endmodule
