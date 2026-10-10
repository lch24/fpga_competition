`timescale 1ns/1ps
module tb_fp32_pair_add_pool;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1;reg [3:0] req_valid=0,req_sub=0,req_pair=0,rsp_ready=0;
 wire [3:0] req_ready,rsp_valid;reg [255:0] req_a=0,req_b=0;wire [255:0] result;
 fp32_pair_add_pool #(.CLIENTS(4),.USE_CE(1)) dut(.*);
 reg [193:0] vectors[0:511];reg [3:0] accepted,received;
 integer ticks=0,checked=0,cancelled=0,g,i,t,fd;reg finished=0;
 always @(negedge clk)begin ticks=ticks+1;ce=(ticks%7!=0 && ticks%13!=0);end
 initial begin
  $readmemh("vectors.hex",vectors);
  repeat(4)@(negedge clk);#1;rst_n=1;
  // Cancellation while one lane is in flight; no stale response may escape.
  req_valid=15;req_pair=15;req_a={8{32'h3f800000}};req_b={8{32'h40000000}};
  repeat(5)@(negedge clk);#1;rst_n=0;req_valid=0;
  repeat(3)@(negedge clk);#1;rst_n=1;cancelled=1;
  for(g=0;g<128;g=g+1)begin
   @(negedge clk);#1;accepted=0;received=0;req_valid=15;rsp_ready=14;
   for(i=0;i<4;i=i+1)begin
     req_sub[i]=vectors[g*4+i][193];req_pair[i]=vectors[g*4+i][192];
     req_a[i*64+:64]=vectors[g*4+i][191:128];req_b[i*64+:64]=vectors[g*4+i][127:64];
   end
   t=0;
   while(received!=15)begin
    @(posedge clk);
    if(ce)begin
     for(i=0;i<4;i=i+1)begin
      if(req_valid[i] && req_ready[i])accepted[i]=1;
      if(rsp_valid[i] && rsp_ready[i])begin
       if(received[i] || !accepted[i])$fatal(1,"duplicate/unsolicited response");
       if(result[i*64+:64]!==vectors[g*4+i][63:0])$fatal(1,"pair mismatch vector %0d got %h expected %h",g*4+i,result[i*64+:64],vectors[g*4+i][63:0]);
       received[i]=1;checked=checked+1;
      end
     end
    end
    @(negedge clk);#1;req_valid=~accepted;
    // Client 0 cannot consume until all others have completed. A pool that
    // holds its physical ALU during response backpressure deadlocks here.
    if(received[3:1]==7)rsp_ready=15;
    t=t+1;if(t>2000)$fatal(1,"pool deadlock");
   end
   req_valid=0;rsp_ready=0;
  end
  finished=1;$finish;
 end
endmodule
