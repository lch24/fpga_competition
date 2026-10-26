`timescale 1ns/1ps
module tb_corner_review_dma;
 localparam W=64,H=32,P=3,V=3,DST=32'h2000,N=W*H;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,pv=0,cmd=0,rq=0;reg [71:0] point;reg [1:0] view;
 wire full,ready,rsp,busy,av;wire [7:0] status;wire [27:0] aa;wire [3:0] al;wire [255:0] data;wire [31:0] strb;
 reg awready=0,wready=0,wdone=0;reg [3:0] wid=0;
 corner_review_dma #(.WIDTH(W),.HEIGHT(H),.POINTS(P),.VIEWS(V)) dut(.clk(clk),.rst_n(rst_n),.point_valid(pv),.point(point),.points_complete(full),
  .cmd_valid(cmd),.cmd_ready(ready),.view(view),.dst_base(DST),.rsp_valid(rsp),.rsp_ready(rq),.status(status),.busy(busy),
  .awvalid(av),.awready(awready),.awaddr(aa),.awlen(al),.wdata(data),.wstrb(strb),.wready(wready),.wdone(wdone),.wid(wid));
 reg [15:0] memory[0:8191];integer xs[0:8],ys[0:8];integer ticks=0,phase=0,at=0,k,delay=0;
 reg stalled=0;reg [31:0] held;
 always @(negedge clk)begin
  ticks=ticks+1;awready=phase==0&&ticks%4!=0;wready=phase==1&&ticks%3!=0;wdone=phase==2&&delay==0;
 end
 always @(posedge clk)if(rst_n)begin
  if(stalled&&(!av||{al,aa}!==held))$fatal(1,"marker AW changed under stall");
  stalled<=av&&!awready;held<={al,aa};
  if(av&&awready)begin
   if(al!=0||aa[2:0]!=0||aa*4<DST||aa*4>=DST+N*2)$fatal(1,"marker invalid native range/alignment");
   at=aa*2;phase=1;
  end
  else if(wready)begin
   for(k=0;k<16;k=k+1)begin
    if(strb[k*2+:2]!=0&&strb[k*2+:2]!=3)$fatal(1,"partial pixel mask");
    if(strb[k*2+:2]==3)memory[at+k]=data[k*16+:16];
   end
   phase=2;delay=2;
  end else if(phase==2)begin if(delay!=0)delay=delay-1;else if(wdone)phase=0;end
 end
 function [31:0] fp(input integer value);
  integer n,e;reg [31:0] mant;
  begin
   if(value==0)fp=0;
   else begin e=0;for(n=0;n<12;n=n+1)if(value>>n)e=n;mant=value<<(23-e);fp=((127+e)<<23)|(mant&32'h7fffff);end
  end
 endfunction
 task send(input integer v,i,x,y);
  begin @(negedge clk);point={2'(v),6'(i),fp(y),fp(x)};pv=1;@(negedge clk);pv=0;end
 endtask
 task paint(input integer v,expected_status);
  integer i,j,x,y,hit;reg [15:0] wanted;
  begin
   for(i=0;i<8192;i=i+1)memory[i]=16'h2a55;
   @(negedge clk);view=v;cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
   wait(rsp);#1;if(status!=expected_status)$fatal(1,"marker status %0d expected %0d",status,expected_status);
   if(expected_status==0)for(i=0;i<8192;i=i+1)begin
    wanted=16'h2a55;
    if(i>=DST/2&&i<DST/2+N)begin
     x=(i-DST/2)%W;y=(i-DST/2)/W;
     for(j=0;j<P;j=j+1)if(xs[v*P+j]>=0&&x>=xs[v*P+j]-2&&x<=xs[v*P+j]+2&&y>=ys[v*P+j]-2&&y<=ys[v*P+j]+2)wanted=16'hf800;
    end
    if(memory[i]!==wanted)$fatal(1,"marker view %0d memory[%0d]=%h expected=%h",v,i,memory[i],wanted);
   end
   repeat(5)@(negedge clk);if(!rsp||!busy)$fatal(1,"response not held");rq=1;@(negedge clk);rq=0;
  end
 endtask
 integer i,v;
 initial begin
  xs[0]=0;ys[0]=0;xs[1]=15;ys[1]=10;xs[2]=63;ys[2]=31;
  xs[3]=16;ys[3]=2;xs[4]=31;ys[4]=16;xs[5]=32;ys[5]=30;
  xs[6]=22;ys[6]=9;xs[7]=22;ys[7]=9;xs[8]=-1;ys[8]=0;
  repeat(4)@(negedge clk);rst_n=1;paint(0,1);
  for(v=0;v<V;v=v+1)for(i=0;i<P;i=i+1)begin
   if(xs[v*P+i]>=0)send(v,i,xs[v*P+i],ys[v*P+i]);
   else begin @(negedge clk);point={2'(v),6'(i),32'b0,32'h7fc00000};pv=1;@(negedge clk);pv=0;end
  end
  if(!full)$fatal(1,"complete point packet sequence not recognized");
  for(v=0;v<V;v=v+1)paint(v,0);
  wid=1;paint(0,5);wid=0;
  // Rounded display location only; original solver FP32 values are not changed.
  if(dut.round_pixel(32'h3f000000)!==13'h1001||dut.round_pixel(32'h3fc00000)!==13'h1002||(dut.round_pixel(32'hbf800000)&13'h1000)!==13'b0)$fatal(1,"rounding/sign guard");
  send(0,0,1,1);if(full)$fatal(1,"new job retained previous valid set");
  $display("PASS red markers: all views/5x5 footprint/edge clipping/beat lanes/masked writes/stalls/delayed completion/errors/new-job invalidation");$finish;
 end
 initial begin #2000000;$fatal(1,"marker timeout");end
endmodule
