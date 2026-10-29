`timescale 1ns/1ps
// Unit test for video_grid_prefetch: 64x40 output, 4x4 control grid.
// The map is a linear warp with fractional Q16.16 coordinates, so the exact
// expected descriptor can be computed with the same integer bilinear weights.
module tb_grid_prefetch;
 localparam W=64,H=40,N=W*H,STRIDE=64,NCOL=17,NROW=11,MAP=32'h00020000;
 localparam BASE_WORD=MAP>>2;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,out_ready=0;
 wire out_valid,error,arvalid;wire [63:0] descriptor;wire [27:0] araddr;wire [3:0] arlen;
 wire arready,rvalid,rlast;
 reg [31:0] mem[0:131071];reg [255:0] read_data;reg rb=0;integer read_at,rbeat,rsz,clock_count=0;
 reg [3:0] rid=0;
 assign arready=!rb&&(clock_count%9!=0);assign rvalid=rb&&(clock_count%5!=0);assign rlast=rbeat==rsz-1;
 integer read_lane;
 always @(read_at or rbeat)for(read_lane=0;read_lane<8;read_lane=read_lane+1)
  read_data[read_lane*32+:32]=mem[read_at+rbeat*8+read_lane];
 always @(posedge clk)if(rst_n)begin
  clock_count<=clock_count+1;
  if(arvalid&&arready)begin rb<=1;read_at<=araddr;rbeat<=0;rsz<=arlen+1;end
  if(rvalid)begin rbeat<=rbeat+1;if(rlast)rb<=0;end
 end
 video_grid_prefetch #(.WIDTH(W),.HEIGHT(H),.STEP(4)) dut(
  .clk(clk),.rst_n(rst_n),.start(start),.map_base(MAP),.out_valid(out_valid),.out_ready(out_ready),
  .descriptor(descriptor),.error(error),.arvalid(arvalid),.arready(arready),.araddr(araddr),.arlen(arlen),
  .rvalid(rvalid),.rdata(read_data),.rlast(rlast),.rid(rid));
 // Control points: sx=0.5u+0.25v+3.5, sy=-0.125u+0.75v+5.5 at (4c,4r).
 longint cp_x[0:NROW*NCOL-1],cp_y[0:NROW*NCOL-1];
 integer c,r,k;
 initial begin
  for(r=0;r<NROW;r=r+1)for(c=0;c<NCOL;c=c+1)begin
   cp_x[r*NCOL+c]=longint'(c*4)*32768+longint'(r*4)*16384+longint'(3*65536+32768);
   cp_y[r*NCOL+c]=-longint'(c*4)*8192+longint'(r*4)*49152+longint'(5*65536+32768);
   mem[BASE_WORD+(r*STRIDE+c)*2]=cp_x[r*NCOL+c][31:0];
   mem[BASE_WORD+(r*STRIDE+c)*2+1]=cp_y[r*NCOL+c][31:0];
  end
 end
 function [63:0] expected(input int px,py);
  int fx,fy,gx,gy,sx,sy,ix,iy;longint vx,vy;reg [3:0] mask;
  begin
   gx=px>>2;gy=py>>2;fx=px&3;fy=py&3;
   vx=(4-fx)*(4-fy)*cp_x[gy*NCOL+gx]+fx*(4-fy)*cp_x[gy*NCOL+gx+1]+
      (4-fx)*fy*cp_x[(gy+1)*NCOL+gx]+fx*fy*cp_x[(gy+1)*NCOL+gx+1];
   vy=(4-fx)*(4-fy)*cp_y[gy*NCOL+gx]+fx*(4-fy)*cp_y[gy*NCOL+gx+1]+
      (4-fx)*fy*cp_y[(gy+1)*NCOL+gx]+fx*fy*cp_y[(gy+1)*NCOL+gx+1];
   sx=int'(vx>>>4);sy=int'(vy>>>4);ix=sx>>>16;iy=sy>>>16;
   mask[0]=ix>=0&&ix<W&&iy>=0&&iy<H;
   mask[1]=ix+1>=0&&ix+1<W&&iy>=0&&iy<H;
   mask[2]=ix>=0&&ix<W&&iy+1>=0&&iy+1<H;
   mask[3]=ix+1>=0&&ix+1<W&&iy+1>=0&&iy+1<H;
   expected={mask,sy[29:16],sx[29:16],sy[15:0],sx[15:0]};
  end
 endfunction
 integer count=0,ar_count=0,r_count=0,bad=0,frame=0,cycles=0;
 always @(posedge clk)if(rst_n)begin
  if(arvalid&&arready)begin
   if(araddr!==(BASE_WORD+ar_count*128))$fatal(1,"map row address %0d got %h",ar_count,araddr);
   ar_count<=ar_count+1;
  end
  if(rvalid)r_count<=r_count+1;
  if(out_valid&&out_ready)begin
   if(count>=N)$fatal(1,"extra descriptor %0d",count);
   if(descriptor!==expected(count%W,count/W))
    $fatal(1,"descriptor %0d got %h expected %h",count,descriptor,expected(count%W,count/W));
   count<=count+1;
  end
 end
 // Ready pattern creates both bubbles and long stalls.
 always @(negedge clk)out_ready<=(clock_count%7!=2)&&(clock_count%13!=5);
 task frame_run;
  begin
   count=0;ar_count=0;r_count=0;
   @(negedge clk);start=1;@(negedge clk);start=0;
   wait(count==N);
   repeat(20)@(posedge clk);
   if(out_valid)$fatal(1,"out_valid after drain");
   if(error)$fatal(1,"unexpected map error");
   frame=frame+1;
  end
 endtask
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  frame_run;
  if(ar_count!=NROW)$fatal(1,"row requests=%0d expected %0d",ar_count,NROW);
  if(r_count!=NROW*16)$fatal(1,"reads=%0d expected %0d",r_count,NROW*16);
  frame_run; // second frame must reload every row buffer
  if(ar_count!=NROW)$fatal(1,"second frame rows=%0d",ar_count);
  if(r_count!=NROW*16)$fatal(1,"second frame reads=%0d",r_count);
  $display("PASS grid prefetch 64x40 descriptors, row rotation, stalls and restart");
  $finish;
 end
 initial begin #4000000;$fatal(1,"grid prefetch timeout state=%0d count=%0d",dut.pstate,count);end
endmodule
