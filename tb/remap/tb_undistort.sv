`timescale 1ns/1ps
// End-to-end reference test over a byte-addressed logical DDR model. This test
// uses the real portable FP32 core (no real/$bitstoshortreal in the RTL).
module tb_undistort;
 reg finished=0;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 reg cmd_valid=0;wire cmd_ready;reg [1:0] cmd_opcode;
 reg [31:0] cmd_job_id=1,cmd_calib_id=42;reg cmd_camera_valid=1;
 reg [287:0] cmd_params;
 reg [15:0] cmd_width=4,cmd_height=3;
 reg [31:0] cmd_src_base=128,cmd_dst_base=256,cmd_map_x_base=512,cmd_map_y_base=768;
 reg [31:0] cmd_src_stride=12,cmd_dst_stride=12,cmd_map_stride=20;
 reg [31:0] cmd_src_capacity=64,cmd_dst_capacity=64,cmd_map_capacity=64;
 reg cmd_border_replicate=0;wire rsp_valid;reg rsp_ready=0;
 wire [31:0] rsp_job_id,map_calib_id;wire [7:0] rsp_status;wire busy,map_valid;
 wire rd_valid,rd_ready;wire [31:0] rd_addr,rd_len;wire [15:0] rd_tag;
 reg r_valid=0;wire r_ready;reg [31:0] r_data;reg [3:0] r_keep;reg [15:0] r_tag;
 reg r_last=1,r_error=0;
 wire wr_valid,wr_ready;wire [31:0] wr_addr,wr_len;wire [15:0] wr_tag;
 wire w_valid,w_ready;wire [31:0] w_data;wire [3:0] w_keep;wire w_last;
 reg b_valid=0;wire b_ready;reg [15:0] b_tag;reg b_error=0;
 undistort_top dut(.*);
 reg [7:0] mem[0:2047];
 reg [15:0] lfsr=16'hbeef;
 integer rd_delay=-1,wr_delay=-1;
 reg [31:0] ra,rl,wa,wl;reg [15:0] rt,wt;
 reg read_pending=0,write_pending=0,write_data_seen=0;
 integer i,j,x,y,addr,n,reads,writes;
 reg inject_read_error=0,inject_write_error=0;
 assign rd_ready=!read_pending&&!r_valid&&lfsr[0];
 assign wr_ready=!write_pending&&!b_valid&&lfsr[1];
 assign w_ready=write_pending&&!write_data_seen&&lfsr[2];
 always @(posedge clk)if(rst_n)begin
  lfsr<={lfsr[14:0],lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
  if(rd_valid&&rd_ready)begin
   if(rd_addr+rd_len>2048||rd_len==0||rd_len>4)$fatal(1,"bad logical read");
   ra<=rd_addr;rl<=rd_len;rt<=rd_tag;read_pending<=1;rd_delay<=3;reads<=reads+1;
  end
  if(rd_delay>0)rd_delay<=rd_delay-1;
  if(read_pending&&rd_delay==0&&!r_valid)begin
   r_data<=0;r_keep<=0;
   for(j=0;j<4;j=j+1)if(j<rl)begin r_data[j*8+:8]<=mem[ra+j];r_keep[j]<=1;end
   r_tag<=rt;r_error<=inject_read_error;r_valid<=1;rd_delay<=-1;
  end
  if(r_valid&&r_ready)begin r_valid<=0;read_pending<=0;end
  if(wr_valid&&wr_ready)begin
   if(wr_addr+wr_len>2048||wr_len==0||wr_len>4)$fatal(1,"bad logical write");
   wa<=wr_addr;wl<=wr_len;wt<=wr_tag;write_pending<=1;write_data_seen<=0;
  end
  if(w_valid&&w_ready)begin
   if(!w_last||w_keep!=((1<<wl)-1))$fatal(1,"bad write packing");
   for(j=0;j<4;j=j+1)if(j<wl)mem[wa+j]<=w_data[j*8+:8];
   write_data_seen<=1;wr_delay<=5;writes<=writes+1;
  end
  if(wr_delay>0)wr_delay<=wr_delay-1;
  if(write_pending&&write_data_seen&&wr_delay==0&&!b_valid)begin
   b_valid<=1;b_tag<=wt;b_error<=inject_write_error;wr_delay<=-1;
  end
  if(b_valid&&b_ready)begin b_valid<=0;write_pending<=0;write_data_seen<=0;end
  if(rsp_valid&&(read_pending||write_pending||r_valid||b_valid))$fatal(1,"response before drain");
 end
 task put32(input integer a,input reg [31:0] v);
  integer k;
  begin for(k=0;k<4;k=k+1)mem[a+k]=v[k*8+:8];end
 endtask
 function [31:0] get32(input integer a);begin get32={mem[a+3],mem[a+2],mem[a+1],mem[a]};end endfunction
 function [15:0] get16(input integer a);begin get16={mem[a+1],mem[a]};end endfunction
 task run_job(input reg [1:0] op,input integer expected_status);
  begin
   @(negedge clk);cmd_opcode=op;cmd_valid=1;
   do @(posedge clk);while(!cmd_ready);
   @(negedge clk);cmd_valid=0;
   wait(rsp_valid);#1;
   if(rsp_status!=expected_status)$fatal(1,"status %d expected %d",rsp_status,expected_status);
   repeat(4)begin @(negedge clk);if(!rsp_valid||!busy||cmd_ready)$fatal(1,"response not held");end
   rsp_ready=1;@(negedge clk);rsp_ready=0;cmd_job_id=cmd_job_id+1;
  end
 endtask
 initial begin
  reads=0;writes=0;
  for(i=0;i<2048;i=i+1)mem[i]=8'ha5;
  // fx=fy=4, cx=cy=0, all distortion zero: exact identity.
  cmd_params={32'd0,32'd0,32'd0,32'd0,32'd0,32'd0,32'd0,32'h40800000,32'h40800000};
  for(y=0;y<3;y=y+1)for(x=0;x<4;x=x+1)begin
   addr=128+y*12+x*2;mem[addr]=(x+y*4)*17;mem[addr+1]=((x+y*4)*17)>>3;
  end
  repeat(4)@(negedge clk);rst_n=1;
  run_job(2,4); // No published map yet.
  run_job(1,0);
  if(!map_valid||map_calib_id!=42)$fatal(1,"map not published");
  for(y=0;y<3;y=y+1)for(x=0;x<4;x=x+1)begin
   case(x)
    0:if(get32(512+y*20+x*4)!=0)$fatal(1,"identity map x0");
    1:if(get32(512+y*20+x*4)!=32'h3f800000)$fatal(1,"identity map x1");
    2:if(get32(512+y*20+x*4)!=32'h40000000)$fatal(1,"identity map x2");
    3:if(get32(512+y*20+x*4)!=32'h40400000)$fatal(1,"identity map x3");
   endcase
  end
  run_job(2,0);
  for(y=0;y<3;y=y+1)begin
   for(x=0;x<4;x=x+1)if(get16(256+y*12+x*2)!=get16(128+y*12+x*2))$fatal(1,"identity pixel mismatch");
   for(x=8;x<12;x=x+1)if(mem[256+y*12+x]!=8'ha5)$fatal(1,"padding overwritten");
  end
  // Inject a partial left-border sample: x=-0.5,y=0, white source.
  mem[128]=8'hff;mem[129]=8'hff;
  put32(512,32'hbf000000);put32(768,0);
  run_job(2,0);
  if(get16(256)!=16'h8410)$fatal(1,"partial black border got %h",get16(256));
  cmd_border_replicate=1;run_job(2,0);
  if(get16(256)!=16'hffff)$fatal(1,"replicate border");
  // Non-finite map produces a black token and preserves output count.
  put32(512,32'h7fc00000);run_job(2,0);if(get16(256)!=0)$fatal(1,"NaN boundary");
  inject_read_error=1;run_job(2,5);inject_read_error=0;
  inject_write_error=1;run_job(1,5);inject_write_error=0;
  if(map_valid)$fatal(1,"failed rebuild published map");
  cmd_dst_base=128;run_job(2,1);
  finished=1;
  $display("PASS undistort: identity, padding, fractional border, replicate, NaN, errors, repeated jobs, backpressure (%0d reads %0d writes)",reads,writes);
  $finish;
 end
 initial begin #2000000;$fatal(1,"simulation timeout");end
endmodule
