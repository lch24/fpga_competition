`timescale 1ns/1ps
module tb_demo_map;
 reg finished=0;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg cfg=0,valid=0;wire cfg_ready,ready,out_valid,error;
 reg [15:0] x,y;wire [31:0] sx,sy;
 wire qv,qr,rv,rr,re;wire [2:0] op;wire [31:0] a,b,result;
 integer file,n,count=0,cycles=0,start_cycle=0,total_cycles=0,report;
 always @(posedge clk)if(rst_n)cycles=cycles+1;real ex,ey,dx,dy,worst=0;
 map_coord_core #(.ENABLE_DISTORT_DEMO(1)) dut(.clk(clk),.rst_n(rst_n),.cfg_inverse(1'b1),
 .cfg_valid(cfg),.cfg_ready(cfg_ready),.cfg_fx(32'h44800000),.cfg_fy(32'h44800000),.cfg_cx(32'h441fe000),.cfg_cy(32'h43b3c000),
 .cfg_k1(32'hbe3851ec),.cfg_k2(32'b0),.cfg_k3(32'b0),.cfg_p1(32'b0),.cfg_p2(32'b0),
 .in_valid(valid),.in_ready(ready),.in_dst_x(x),.in_dst_y(y),.in_pixel_id(32'b0),.in_last(1'b0),
 .out_valid(out_valid),.out_ready(1'b1),.out_src_x(sx),.out_src_y(sy),.out_error(error),
 .fp_req_valid(qv),.fp_req_ready(qr),.fp_req_op(op),.fp_req_a(a),.fp_req_b(b),.fp_rsp_valid(rv),.fp_rsp_ready(rr),.fp_rsp_result(result),.fp_rsp_error(re));
 fp32_service arithmetic(.clk(clk),.rst_n(rst_n),.req_valid(qv),.req_ready(qr),.req_op(op),.req_a(a),.req_b(b),.rsp_valid(rv),.rsp_ready(rr),.rsp_result(result),.rsp_error(re));
 function real value(input [31:0] bits);
  begin value=(bits[31]?-1.0:1.0)*(1.0+bits[22:0]/8388608.0)*(2.0**($signed({1'b0,bits[30:23]})-127));if(bits[30:0]==0)value=0;end
 endfunction
 initial begin
  repeat(3)@(negedge clk);rst_n=1;cfg=1;@(negedge clk);cfg=0;
  file=$fopen("../coordinates.txt","r");if(!file)$fatal(1,"missing reference");
  while(!$feof(file))begin
   n=$fscanf(file,"%d %d %f %f\n",x,y,ex,ey);
   if(n==4)begin
    start_cycle=cycles;valid=1;do @(posedge clk);while(!ready);@(negedge clk);valid=0;
    wait(out_valid);total_cycles=total_cycles+cycles-start_cycle;dx=value(sx)-ex;dy=value(sy)-ey;
    if(error||dx*dx+dy*dy>0.0025)$fatal(1,"inverse mismatch %d,%d error=%f,%f",x,y,dx,dy);
    if(dx*dx+dy*dy>worst)worst=dx*dx+dy*dy;count=count+1;@(negedge clk);
   end
  end
  if(count<2000)$fatal(1,"missing coverage");
  report=$fopen("cycles.txt","w");$fdisplay(report,"samples=%d cycles=%d cycles_per_pixel=%f coordinate_seconds_720p_40MHz=%f",count,total_cycles,1.0*total_cycles/count,1.0*total_cycles/count*921600/40000000);$fclose(report);
  finished=1;$display("PASS demo_map %d actual FP32 inverse coordinates max squared error=%f",count,worst);$finish;
 end
 initial begin #200000000;$fatal(1,"timeout");end
endmodule

