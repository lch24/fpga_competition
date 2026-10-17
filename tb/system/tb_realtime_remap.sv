`timescale 1ns/1ps
module tb_realtime_remap;
 reg finished=0;reg bypass=0;
 localparam W=128,H=40,N=W*H,MAP=32'h20000,DST=32'h40000;
 reg clk=0;always #4 clk=~clk;
 reg rst_n=0,cmd=0;wire ready,rsp;wire [7:0] status;wire [31:0] cycles;
 reg [31:0] source=32'h10000;
 wire [1:0] av,aq,rv;wire [55:0] aa;wire [7:0] al;
 wire wv,wq,wr,wd;wire [27:0] wa;wire [3:0] wl;wire [255:0] data;
 wire rl;wire [255:0] rd;reg inject_error=0;wire [3:0] rid=inject_error?4'd1:4'd0;
 realtime_remap #(.WIDTH(W),.HEIGHT(H),.CACHE_INDEX_BITS(3)) dut(
  .cmd_bypass(bypass),.clk(clk),.rst_n(rst_n),.cmd_valid(cmd),.cmd_ready(ready),.src_base(source),.dst_base(DST),.map_base(MAP),
  .rsp_valid(rsp),.rsp_ready(1'b1),.rsp_status(status),.last_cycles(cycles),
  .arvalid(av),.arready(aq),.araddr(aa),.arlen(al),.rvalid(rv),.rdata(rd),.rlast(rl),.rid(rid),
  .awvalid(wv),.awready(wq),.awaddr(wa),.awlen(wl),.wdata(data),.wready(wr),.wdone(wd),.wid(4'b0));
 wire [27:0] araddr,awaddr;wire [3:0] arlen,awlen;
 wire arvalid,arready,awvalid,awready,rvalid,wready,wdone;wire [255:0] ww;
 wire [31:0] ws;wire [3:0] crv,car;wire [2:0] caw,cwr,cwd;
 reg display_request=0,capture_request=0;reg [31:0] clock_count=0;
 reg [255:0] capture_payload={16{16'hcafe}};
 assign aq=car[3:2];assign rv=crv[3:2];assign wq=caw[2];assign wr=cwr[2];assign wd=cwd[2];
 video_hmic_arbiter arb(.clk(clk),.rst_n(rst_n),
  .c_arvalid({av,display_request,1'b0}),.c_arready(car),.c_araddr({aa,28'h4000,28'b0}),.c_arlen({al,4'd15,4'b0}),
  .c_rvalid(crv),.c_rdata(),.c_rlast(),.c_rid(),
  .c_awvalid({wv,1'b0,capture_request}),.c_awready(caw),.c_awaddr({wa,28'b0,28'h18000}),.c_awlen({wl,4'b0,4'b0}),
  .c_wdata({data,256'b0,capture_payload}),.c_wstrb({32'hffffffff,32'b0,32'hffffffff}),.c_wready(cwr),.c_wdone(cwd),.c_wid(),
  .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rd),.rvalid(rvalid),.rlast(rl),.rid(rid),
  .awaddr(awaddr),.awlen(awlen),.awvalid(awvalid),.awready(awready),.wdata(ww),.wstrb(ws),.wready(wready),.wdone(wdone),.wid(4'b0),.idle());
 reg [15:0] memory[0:262143],expected[0:N-1];reg [63:0] expected_neighbors[0:N-1];integer neighbor_index=0;
 always @(posedge clk)begin
  if(cmd&&ready)neighbor_index<=0;
  else if(dut.cv&&dut.cr)begin
   if(!inject_error&&dut.neighbors!==expected_neighbors[neighbor_index])
    $fatal(1,"neighbors index=%0d desc=%h got=%h expected=%h addr=%h/%h",neighbor_index,dut.cache.desc,dut.neighbors,expected_neighbors[neighbor_index],dut.cache.a0,dut.cache.a1);
   neighbor_index<=neighbor_index+1;
  end
 end
 reg rb=0,wb=0;reg [31:0] read_at,write_at;reg [4:0] rbeat,wbeat,rsz,wsz;
 reg write_completion=0;integer j;reg [255:0] read_data;
 assign arready=!rb&&clock_count%7!=0;assign awready=!wb&&!write_completion&&clock_count%5!=0;
 assign rvalid=rb&&clock_count%4!=0;assign rl=rbeat==rsz-1;assign rd=read_data;
 assign wready=wb&&clock_count%3!=0;
 // Delayed write completion tests the post-data drain, not just last WREADY.
 assign wdone=write_completion;
 integer read_lane;
 always @(read_at or rbeat)for(read_lane=0;read_lane<16;read_lane=read_lane+1)read_data[read_lane*16+:16]=memory[read_at+rbeat*16+read_lane];
 reg ar_stalled=0,aw_stalled=0;reg [31:0] held_ar,held_aw;
 always @(posedge clk)begin
  if(rst_n)begin
   if(dut.bypass&&av[0])$fatal(1,"identity path read map DDR");
   clock_count<=clock_count+1;
   if(ar_stalled&&(!arvalid||{arlen,araddr}!=held_ar))$fatal(1,"AR changed under stall");
   if(aw_stalled&&(!awvalid||{awlen,awaddr}!=held_aw))$fatal(1,"AW changed under stall");
   ar_stalled<=arvalid&&!arready;held_ar<={arlen,araddr};
   aw_stalled<=awvalid&&!awready;held_aw<={awlen,awaddr};
   if(clock_count%401==0)display_request<=1;
   if(display_request&&car[1])display_request<=0;
   if(clock_count%197==0)capture_request<=1;
   if(capture_request&&caw[0])capture_request<=0;
   if(arvalid&&arready)begin rb<=1;read_at<=araddr*2;rbeat<=0;rsz<=arlen+1'b1;end
   if(rvalid)begin rbeat<=rbeat+1;if(rl)rb<=0;end
   if(awvalid&&awready)begin wb<=1;write_at<=awaddr*2;wbeat<=0;wsz<=awlen+1'b1;end
   if(wready)begin
    for(j=0;j<16;j=j+1)if(ws[j*2+:2]==3)memory[write_at+wbeat*16+j]<=ww[j*16+:16];
    wbeat<=wbeat+1;if(wbeat==wsz-1)begin wb<=0;write_completion<=1;end
   end
   if(write_completion)write_completion<=0;
  end
 end
 function [7:0] chan(input [15:0] p,input integer c);
  case(c)0:chan={p[4:0],p[4:2]};1:chan={p[10:5],p[10:9]};default:chan={p[15:11],p[15:13]};endcase
 endfunction
 function [15:0] reference(input [15:0] a,b,c,d,input [15:0] dx,dy);
  reg [63:0] sum;reg [7:0] channels[0:2];integer k;
  begin
   for(k=0;k<3;k=k+1)begin
    sum=64'(chan(a,k))*(65536-dx)*(65536-dy)+64'(chan(b,k))*dx*(65536-dy)+
        64'(chan(c,k))*(65536-dx)*dy+64'(chan(d,k))*dx*dy+64'h80000000;
    channels[k]=sum>>32;
   end
   reference={channels[2][7:3],channels[1][7:2],channels[0][7:3]};
  end
 endfunction
 function [15:0] sample(input integer x,y);
  sample=x<0||x>=W||y<0||y>=H?0:memory[source/2+y*W+x];
 endfunction
 task frame(input integer number);
  integer i,x,y,k;reg [15:0] fx,fy;reg [3:0] mask;reg [63:0] desc;
  begin
   source=number%2?32'h14000:32'h10000;
   for(i=0;i<N;i=i+1)memory[source/2+i]=16'((i*37+number*1337)^((i/W)*271));
   for(i=0;i<N;i=i+1)begin
    x=i%W;y=i/W;
    if(number==1)begin x=(i*29)%(W+1)-1;y=(i*13)%(H+1)-1;end
    if(number==2)begin x=x-1;y=y-1;end
    fx=16'(i*113);fy=16'(i*711+number);
    mask[0]=x>=0&&x<W&&y>=0&&y<H;mask[1]=x+1>=0&&x+1<W&&y>=0&&y<H;
    mask[2]=x>=0&&x<W&&y+1>=0&&y+1<H;mask[3]=x+1>=0&&x+1<W&&y+1>=0&&y+1<H;
    desc={mask,14'(y),14'(x),fy,fx};
    for(k=0;k<4;k=k+1)memory[MAP/2+i*4+k]=desc[k*16+:16];
    expected[i]=reference(sample(x,y),sample(x+1,y),sample(x,y+1),sample(x+1,y+1),fx,fy);
    expected_neighbors[i]={sample(x+1,y+1),sample(x,y+1),sample(x+1,y),sample(x,y)};
    if(bypass)begin
     expected[i]=sample(i%W,i/W);
     expected_neighbors[i]={48'b0,expected[i]};
     for(k=0;k<4;k=k+1)memory[MAP/2+i*4+k]=16'hffff; // Poison: preview cannot depend on the map.
    end
    memory[DST/2+i]=16'hdead;
   end
   @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
   wait(rsp);#1;
   if(status!==(inject_error?8'd5:8'd0))$fatal(1,"frame %0d status %0d",number,status);
   if(!inject_error)for(i=0;i<N;i=i+1)if(memory[DST/2+i]!==expected[i])
    $fatal(1,"frame %0d pixel %0d got %h expected %h",number,i,memory[DST/2+i],expected[i]);
   $display("PASS frame %0d pixels=%0d cycles=%0d error=%0b",number,N,cycles,inject_error);
   @(posedge clk);@(negedge clk);
  end
 endtask
 initial begin
  repeat(5)@(posedge clk);@(negedge clk);rst_n=1;
  frame(0);frame(1);frame(2);
  bypass=1;frame(4);frame(5); // All pixels including right/bottom edge must copy exactly.
  inject_error=1;frame(6);inject_error=0;bypass=0;frame(0);
  inject_error=1;frame(3);
  finished=1;
  $display("PASS realtime_remap numerical borders, cache conflicts/frame invalidation, native arbitration/stalls/error drain");$finish;
 end
 initial begin #30000000;$fatal(1,"timeout state=%d cache=%d writer=%d",dut.state,dut.cache.state,dut.writer.state);end
endmodule
