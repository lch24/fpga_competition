`timescale 1ns/1ps
module tb_map_coord;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 reg cfg_valid=0;wire cfg_ready;reg [31:0] camera[0:8];
 wire [31:0] cfg_fx=camera[0],cfg_fy=camera[1],cfg_cx=camera[2],cfg_cy=camera[3];
 wire [31:0] cfg_k1=camera[4],cfg_k2=camera[5],cfg_k3=camera[6],cfg_p1=camera[7],cfg_p2=camera[8];
 reg in_valid=0;wire in_ready;reg [15:0] in_dst_x,in_dst_y;reg [31:0] in_pixel_id;reg in_last;
 wire out_valid;reg out_ready=0;wire [31:0] out_src_x,out_src_y,out_pixel_id;
 wire [15:0] out_dst_x,out_dst_y;wire out_last,out_error;
 wire fp_req_valid,fp_req_ready,fp_rsp_valid,fp_rsp_ready,fp_rsp_error;
 wire [2:0] fp_req_op;wire [31:0] fp_req_a,fp_req_b,fp_rsp_result;
 map_coord_core dut(.*);
 fp32_service fp(.clk(clk),.rst_n(rst_n),.req_valid(fp_req_valid),.req_ready(fp_req_ready),.req_op(fp_req_op),.req_a(fp_req_a),.req_b(fp_req_b),.rsp_valid(fp_rsp_valid),.rsp_ready(fp_rsp_ready),.rsp_result(fp_rsp_result),.rsp_error(fp_rsp_error));
 reg [95:0] vectors[0:99];integer i;
 initial begin
  $readmemh("camera.mem",camera);$readmemh("map_coords.mem",vectors);
  repeat(3)@(negedge clk);rst_n=1;cfg_valid=1;
  do @(posedge clk);while(!cfg_ready);@(negedge clk);cfg_valid=0;
  for(i=0;i<100;i=i+1)begin
   in_dst_x=vectors[i][95:80];in_dst_y=vectors[i][79:64];in_pixel_id=i;in_last=i==99;in_valid=1;
   do @(posedge clk);while(!in_ready);@(negedge clk);in_valid=0;
   wait(out_valid);repeat(3)@(negedge clk);
   if(out_src_x!==vectors[i][63:32]||out_src_y!==vectors[i][31:0]||out_pixel_id!=i||out_error||out_last!=(i==99))$fatal(1,"map mismatch %d got %h %h expected %h %h",i,out_src_x,out_src_y,vectors[i][63:32],vectors[i][31:0]);
   out_ready=1;@(negedge clk);out_ready=0;
  end
  $display("PASS map coordinate: 100 current-camera Brown vectors bit-for-bit");$finish;
 end
 initial begin #1000000;$fatal(1,"map timeout");end
endmodule
