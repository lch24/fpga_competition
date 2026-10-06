`timescale 1ns/1ps
// Long, optional test: independent rendered images through ALL real RTL.
// Requires data/generators/system/generate_board_images.js, no force statements.
module tb_vision_numeric #(parameter DETECT_ONLY=0, FIXED_INTERPOLATION=1, FIXED_ACCUM=FIXED_INTERPOLATION);
 reg finished=0;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg start_valid=0,frame_valid=0,frame_release_ready=1,rsp_ready=0;
 wire start_ready,frame_ready,frame_release_valid,rsp_valid,busy;
 reg [63:0] square_size_fp64=64'h3ff0000000000000;
 reg [31:0] frame_base=32,frame_stride=328,frame_capacity=39360;
 reg [7:0] frame_status=0;wire [7:0] rsp_status,debug_view;
 wire [31:0] debug_job,result_base,result_stride;wire [15:0] result_width,result_height;
 wire [5:0] debug_phase;wire [287:0] result_params;wire [63:0] result_rms;
 `include "logical_bus.vh"
 vision_ddr_top #(.FIXED_ACCUM(FIXED_ACCUM),.FIXED_BILINEAR(FIXED_INTERPOLATION),.CANDIDATE_BASE(524288),.WIDTH(160),.HEIGHT(120),.DEPTH(1),.GRAY_BASE(65536),.DST_BASE(98304),.MAP_X_BASE(196608),.MAP_Y_BASE(327680)) dut(.*);
 `include "logical_memory.vh"
 reg [15:0] pixels[0:57599];integer view,x,y,n,fd,cycles=0;reg [5:0] prev_phase=63;
 reg [63:0] expected_corners[0:119];
 reg [1023:0] csv_header;
 integer baseline_fd,baseline_read,baseline_view,baseline_index,ci,observed=0;
 reg [31:0] baseline_x,baseline_y;
 real max_corner_error=0,corner_error_x,corner_error_y;
 function real fp_value(input [31:0] v);
  integer e;
  begin
   e=v[30:23];
   fp_value=(e==0)?0.0:(1.0+v[22:0]/8388608.0)*(2.0**(e-127));
   if(v[31])fp_value=-fp_value;
  end
 endfunction
 initial if(DETECT_ONLY)begin
  baseline_fd=$fopen("../../data/system/detected_corners.csv","r");
  if(!baseline_fd)$fatal(1,"missing committed corner baseline");
  baseline_read=$fgets(csv_header,baseline_fd);
  for(ci=0;ci<120;ci=ci+1)begin
   baseline_read=$fscanf(baseline_fd,"%d,%d,%h,%h\n",baseline_view,baseline_index,baseline_x,baseline_y);
   if(baseline_read!=4||baseline_view!=ci/40||baseline_index!=ci%40)$fatal(1,"bad corner baseline");
   expected_corners[ci]={baseline_x,baseline_y};
  end
  $fclose(baseline_fd);
 end
 always @(posedge clk)begin
  cycles<=cycles+1;
  if(debug_phase!=prev_phase)begin $display("NUMERIC phase=%d view=%d cycle=%d status=%d",debug_phase,debug_view,cycles,rsp_status);prev_phase<=debug_phase;end
  if(cycles%10000000==0)$display("NUMERIC progress cycle=%d phase=%d cal=%d",cycles,debug_phase,dut.calibration.pc);
  if(dut.dv&&dut.dr)$display("DETECTED view=%d x=%h y=%h",debug_view,dut.dx,dut.dy);
  if(DETECT_ONLY&&dut.dv&&dut.dr)begin
   if(observed>=120||debug_view!=observed/40)$fatal(1,"corner count/order mismatch");
   if(FIXED_INTERPOLATION)begin
    if((^dut.dx===1'bx)||(^dut.dy===1'bx)||dut.dx[30:23]==255||dut.dy[30:23]==255)$fatal(1,"nonfinite corner");
    corner_error_x=fp_value(dut.dx)-fp_value(expected_corners[observed][63:32]);
    corner_error_y=fp_value(dut.dy)-fp_value(expected_corners[observed][31:0]);
    if(corner_error_x<0)corner_error_x=-corner_error_x;
    if(corner_error_y<0)corner_error_y=-corner_error_y;
    if(corner_error_x>max_corner_error)max_corner_error=corner_error_x;
    if(corner_error_y>max_corner_error)max_corner_error=corner_error_y;
    if(corner_error_x>0.02||corner_error_y>0.02)$fatal(1,"corner %0d error %f %f",observed,corner_error_x,corner_error_y);
   end else if({dut.dx,dut.dy}!==expected_corners[observed])$fatal(1,"PDS compatibility changed corner %0d",observed);
   observed=observed+1;
  end
  if(DETECT_ONLY&&debug_phase==10)begin
   if(observed!=120||violations!=0)$fatal(1,"detection collection incomplete");
   $display("PASS detection: fixed=%0d corners=120 max_error_pixels=%f",FIXED_INTERPOLATION,max_corner_error);$finish;
  end
 end
 initial begin
  $readmemh("../../data/system/board_images.hex",pixels);
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
  finished=1;
  $display("PASS vision numeric: three rendered images -> actual detection/init/LM -> maps -> corrected DDR, cycles=%d rms=%h",cycles,result_rms);$finish;
 end
 initial begin #16000000000.0;$fatal(1,"full numeric timeout phase=%d",debug_phase);end
endmodule
