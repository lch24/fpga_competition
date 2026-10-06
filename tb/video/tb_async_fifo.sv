`timescale 1ns/1ps
module tb_async_fifo;
 reg wclk=0,rclk=0;always #3 wclk=~wclk;always #5 rclk=~rclk;
 reg rst_n=0,wvalid=0,rready=0;wire wready,rvalid;reg [15:0] wdata=0;wire [15:0] rdata;
 integer sent=0,received=0;reg [15:0] wl=16'hcafe,rl=16'h1234;
 async_fifo #(.WIDTH(16),.ADDR_BITS(2)) dut(.wclk(wclk),.wrst_n(rst_n),.wvalid(wvalid),.wready(wready),.wdata(wdata),
 .rclk(rclk),.rrst_n(rst_n),.rvalid(rvalid),.rready(rready),.rdata(rdata));
 always @(posedge wclk)if(rst_n&&wvalid&&wready)sent<=sent+1;
 always @(negedge wclk)if(rst_n)begin
  wl={wl[14:0],wl[15]^wl[13]^wl[12]^wl[10]};
  if(!wvalid||wready)begin wvalid=sent<1024&&wl[0];wdata=sent;end
 end
 always @(negedge rclk)if(rst_n)begin rl={rl[14:0],rl[15]^rl[13]^rl[12]^rl[10]};rready=rl[0]&&rl[1];end
 always @(posedge rclk)if(rst_n&&rvalid&&rready)begin
  if(rdata!==received[15:0])$fatal(1,"FIFO order got %d expected %d",rdata,received);
  received<=received+1;
 end
 initial begin #40;rst_n=1;wait(received==1024);repeat(8)@(negedge rclk);if(rvalid)$fatal(1,"FIFO not empty");
  $display("PASS async FIFO: 1024 words, pointer wraps, unrelated clocks, random stalls");$finish;end
 initial begin #200000;$fatal(1,"FIFO timeout %d/%d",sent,received);end
endmodule
