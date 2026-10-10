`timescale 1ns/1ps
module tb_detection_capture;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0,pts_valid=0,pts_done=0;
 reg [31:0] pts_x=32'h42000000,pts_y=32'h43000000;
 wire pts_ready,busy,done,seq_done;wire [31:0] seq_result;
 reg seq_valid=0,seq_enter=0;reg [7:0] seq_id=0;reg [15:0] seq_arg=0;
 grid_order_ctrl #(.EXTERNAL_SEQ(1),.SHARED_ADD(1),.SHARED_HYPOT(1),.SHARED_VALIDATE(1),.USE_CE(1)) dut(
  .clk(clk),.rst_n(rst_n),.ce(ce),.start(start),.busy(busy),.done(done),
  .pts_valid(pts_valid),.pts_ready(pts_ready),.pts_x(pts_x),.pts_y(pts_y),.pts_done(pts_done),
  .seq_valid(seq_valid),.seq_enter(seq_enter),.seq_id(seq_id),.seq_arg(seq_arg),.seq_done(seq_done),.seq_result(seq_result),
  .out_ready(1'b0),.add_req_ready(1'b0),.add_rsp_valid(1'b0),.add_result(64'd0),
  .math_hyp_ready(1'b0),.math_hyp_rsp_valid(1'b0),.math_hyp_result(32'd0),
  .val_busy(1'b0),.val_done(1'b0),.val_valid(1'b0),.val_cost(32'd0),.val_rd_en(1'b0),.val_rd_addr(6'd0));
 integer counts[0:4];integer ticks=0,checked=0,t,i,waited;reg finished=0;
 always @(negedge clk)begin ticks=ticks+1;ce=ticks%7!=0;end
 task step;begin @(posedge clk);while(!ce)@(posedge clk);@(negedge clk);#1;end endtask
 initial begin
  counts[0]=0;counts[1]=39;counts[2]=40;counts[3]=256;counts[4]=300;
  repeat(4)@(negedge clk);#1;rst_n=1;
  for(t=0;t<5;t=t+1)begin
   start=1;step();start=0;
   for(i=0;i<counts[t];i=i+1)begin
    pts_valid=1;waited=0;
    @(posedge clk);while(!ce || !pts_ready)begin waited=waited+1;if(waited>50)$fatal(1,"overflow did not drain");@(posedge clk);end
    @(negedge clk);#1;
   end
   pts_valid=0;pts_done=1;step();pts_done=0;step();
   seq_valid=1;seq_enter=1;seq_id=32;step();seq_enter=0;step();
   if(!seq_done || seq_result[0]!=(t!=2))$fatal(1,"capture status case %0d %h",t,seq_result);
   seq_valid=0;step();seq_valid=1;seq_enter=1;seq_id=34;step();seq_enter=0;step();
   if(!seq_done || !done)$fatal(1,"finish handshake");
   seq_valid=0;step();checked=checked+1;
  end
  finished=1;$finish;
 end
endmodule
