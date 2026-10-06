`timescale 1ns/1ps
// Blank pyramid forces native detection at BOTH scales, exercising max/threshold
// reset between layers and the real gray->response pipeline restart.
module tb_replay_pyramid;
 reg clk=0,rst_n=0,start=0;
 always #5 clk=~clk;
 wire busy,done,gray_rd_en,out_valid;
 wire [1:0] status;
 wire [10:0] gray_rd_addr;
 reg [7:0] gray_rd_data=0;
 reg dump=0;
 integer taps=0,replays=0,checks=0;
 reg finished=0;
 wire resp_tap_valid;
 detect_ctrl #(.W0(32),.H0(24),.DEPTH(2),.GRAY_ADDR_W(11),
               .REPLAY_RESP(1)) dut (
  .clk(clk),.rst_n(rst_n),.start(start),.cfg_base0(11'd17),
  .busy(busy),.done(done),.status(status),
  .gray_rd_en(gray_rd_en),.gray_rd_addr(gray_rd_addr),.gray_rd_data(gray_rd_data),
  .resp_tap_valid(resp_tap_valid),.resp_tap_data(),
  .cfg_resp_dump_en(dump),.resp_dump_valid(),.resp_dump_ready(1'b1),
  .resp_dump_data(),.resp_dump_done(),
  .out_valid(out_valid),.out_ready(1'b1),.out_x(),.out_y(),.out_total(),.out_grid_ok());
 always @(posedge clk) if(rst_n) begin
  if(gray_rd_en) begin
   if(gray_rd_addr<17 || gray_rd_addr>=17+960)$fatal(1,"gray address outside pyramid");
   gray_rd_data<=0;
  end
  if(resp_tap_valid)taps=taps+1;
  if(dut.g_slot[0].replay_start || dut.g_slot[1].replay_start) replays=replays+1;
  if(out_valid)$fatal(1,"blank frame returned corner");
 end
 initial begin
  repeat(5)@(negedge clk);rst_n=1;start=1;
  @(negedge clk);start=0;wait(done);@(negedge clk);
  if(status!=2'b10 || taps!=960 || replays!=2)
   $fatal(1,"pyramid replay status=%d taps=%d replays=%d",status,taps,replays);
  checks=checks+1;
  // RAM-free mode must explicitly reject the legacy response dump contract.
  rst_n=0;repeat(3)@(negedge clk);rst_n=1;dump=1;start=1;
  @(negedge clk);start=0;wait(done);@(negedge clk);
  if(status!=2'b11 || busy)$fatal(1,"unsupported dump not rejected");
  checks=checks+1;finished=1;
  $display("PASS replay pyramid: two native layers and explicit dump rejection");$finish;
 end
 initial begin #20000000;$fatal(1,"pyramid replay timeout");end
endmodule
