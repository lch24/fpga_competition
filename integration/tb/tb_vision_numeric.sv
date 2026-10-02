`timescale 1ns/1ps
// Long, optional test: independent rendered images through ALL real RTL.
// Requires integration/tools/generate_board_images.js, no force statements.
module tb_vision_numeric;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg start_valid=0,frame_valid=0,frame_release_ready=1,rsp_ready=0;
 wire start_ready,frame_ready,frame_release_valid,rsp_valid,busy;
 reg [63:0] square_size_fp64=64'h3ff0000000000000;
 reg [31:0] frame_base=32,frame_stride=328,frame_capacity=39360;
 reg [7:0] frame_status=0;wire [7:0] rsp_status,debug_view;
 wire [31:0] debug_job,result_base,result_stride;wire [15:0] result_width,result_height;
 wire [5:0] debug_phase;wire [287:0] result_params;wire [63:0] result_rms;
 `include "logical_bus.vh"
 vision_ddr_top #(.WIDTH(160),.HEIGHT(120),.DEPTH(1),.GRAY_BASE(65536),.DST_BASE(98304),.MAP_X_BASE(196608),.MAP_Y_BASE(327680)) dut(.*);
 `include "logical_memory.vh"
 reg [15:0] pixels[0:57599];integer view,x,y,n,fd,cycles=0;reg [5:0] prev_phase=63;
 always @(posedge clk)begin
  cycles<=cycles+1;
  if(debug_phase!=prev_phase)begin $display("NUMERIC phase=%d view=%d cycle=%d status=%d",debug_phase,debug_view,cycles,rsp_status);prev_phase<=debug_phase;end
  if(cycles%10000000==0)$display("NUMERIC progress cycle=%d phase=%d cal=%d",cycles,debug_phase,dut.calibration.pc);
  if(dut.dv&&dut.dr)$display("DETECTED view=%d x=%h y=%h",debug_view,dut.dx,dut.dy);
 end
 initial begin
  $readmemh("board_images.hex",pixels);
  repeat(5)@(negedge clk);rst_n=1;start_valid=1;@(negedge clk);start_valid=0;
  for(view=0;view<3;view=view+1)begin
   wait(frame_ready||rsp_valid);if(rsp_valid)$fatal(1,"early pipeline failure status=%d view=%d",rsp_status,view);
   for(y=0;y<120;y=y+1)for(x=0;x<160;x=x+1)begin
    memory.mem[32+y*328+x*2]=pixels[view*19200+y*160+x][7:0];
    memory.mem[33+y*328+x*2]=pixels[view*19200+y*160+x][15:8];
   end
   @(negedge clk);frame_valid=1;@(negedge clk);frame_valid=0;
  end
  wait(rsp_valid);if(rsp_status!==0||violations!=0)$fatal(1,"real pipeline result status=%d violations=%d",rsp_status,violations);
  if(result_rms[62:52]==2047)$fatal(1,"nonfinite RMS");
  fd=$fopen("numeric_camera.hex","w");for(n=0;n<9;n=n+1)$fdisplay(fd,"%08h",result_params[n*32+:32]);$fclose(fd);
  fd=$fopen("numeric_output.hex","w");for(n=0;n<38400;n=n+1)$fdisplay(fd,"%02h",memory.mem[98304+n]);$fclose(fd);
  $display("PASS vision numeric: three rendered images -> actual detection/init/LM -> maps -> corrected DDR, cycles=%d rms=%h",cycles,result_rms);$finish;
 end
 initial begin #16000000000.0;$fatal(1,"full numeric timeout phase=%d",debug_phase);end
endmodule
