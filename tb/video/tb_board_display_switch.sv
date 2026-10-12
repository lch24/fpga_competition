`timescale 1ns/1ps
module tb_board_display_switch;
 reg clk=0,rst_n=0,enable=0,finished=0,pixel_ready=1,axi_arready=0,axi_rvalid=0,axi_rlast=0;
 reg [31:0] frame_base=0;reg [255:0] axi_rdata=0;
 wire idle,pixel_valid,pixel_first,pixel_last,error,axi_arvalid;
 wire frame_start;
 wire [15:0] pixel;wire [27:0] axi_araddr;wire [3:0] axi_arlen,axi_aruser_id;
 reg [3:0] axi_rid=0;
 integer pending=0,base=0,beat=0,i,checked=0,expected_base=0,index=0;
 reg stall_addr=0,stall_data=0,score=0;
 always #4 clk=~clk;
 board_display_hmic #(.DYNAMIC_BASE(1),.WIDTH(64),.HEIGHT(2)) dut(.*);
 always @(negedge clk)begin
  axi_arready=0;axi_rvalid=0;axi_rlast=0;
  if(rst_n)begin
   if(pending>0&&!stall_data)begin
    axi_rvalid=1;axi_rlast=pending==1;
    for(i=0;i<16;i=i+1)axi_rdata[i*16+:16]=(base/2+beat*16+i);
    pending=pending-1;beat=beat+1;
   end else if(pending==0&&axi_arvalid&&!stall_addr)begin
    axi_arready=1;pending=axi_arlen+1;base=axi_araddr*4;beat=0;
   end
  end
 end
 always @(posedge clk)if(score&&pixel_valid&&pixel_ready)begin
  if(pixel!==16'(expected_base/2+index)||pixel_first!==(index==0)||pixel_last!==(index==127))$fatal(1,"selected frame mismatch %0d",index);
  index=(index+1)%128;checked=checked+1;
 end
 task tick;@(negedge clk);#1;endtask
 task stop;
  begin enable=0;wait(idle);tick();if(pending||axi_arvalid||pixel_valid)$fatal(1,"idle before drain");end
 endtask
 task check_frame(input integer address);
  begin frame_base=address;expected_base=address;index=0;checked=0;score=1;enable=1;
   wait(checked==128);tick();score=0;stop();end
 endtask
 initial begin
  repeat(4)tick();rst_n=1;
  check_frame(32'h200000);check_frame(32'h400000);check_frame(32'h1000000);
  // An asserted ARVALID cannot be withdrawn when the UI stops display.
  stall_addr=1;enable=1;wait(axi_arvalid);tick();enable=0;
  repeat(5)begin tick();if(idle||!axi_arvalid)$fatal(1,"withdrawn AR");end
  stall_data=1;stall_addr=0;wait(dut.state==2);tick();
  repeat(5)begin tick();if(idle)$fatal(1,"lost outstanding read");end
  stall_data=0;wait(idle);tick();check_frame(0);
  pixel_ready=0;enable=1;wait(dut.state==3);tick();stop();pixel_ready=1;
  check_frame(32'h200000);
  // Error drains before the next selection can recover.
  axi_rid=1;enable=1;wait(error);wait(idle);tick();stop();axi_rid=0;check_frame(0);
  finished=1;$display("PASS tb_board_display_switch whole frames, addresses, AR/R drain, stalled pixels, error recovery");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
