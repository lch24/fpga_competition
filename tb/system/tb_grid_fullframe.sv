`timescale 1ns/1ps
// Full-size throughput/correctness test for the 4x4 control-point map path.
// Same 1280x192 fisheye strip as tb_realtime_fullframe, but the DDR table is
// only 321x49 control points (Q16.16) and the reader interpolates them.
// Expected pixels use the exact same integer bilinear weights as the RTL.
module tb_grid_fullframe #(parameter CACHE_BITS=6);
 reg finished=0;
 localparam W=1280,HSRC=720,HST=192,N=W*HST;
 localparam ROW0=264;
 localparam STEP=4,GW=W/STEP,GH=HST/STEP,NCOL=GW+1,NROW=GH+1,STRIDE=384;
 localparam SRC=32'h00100000,MAP=32'h00400000,DST=32'h00800000;
 localparam real CX=640.0,CY=360.0;
 localparam real FP_RAW=560.0,FP_OUT=300.0;
 localparam real K1=-0.05,K2=0.005,K3=-0.0002;
 reg clk=0;always #4 clk=~clk;
 reg rst_n=0,cmd=0;wire ready,rsp;wire [7:0] status;wire [31:0] cycles;
 wire [1:0] av,aq,rv;wire [55:0] aa;wire [7:0] al;
 wire wv,wq,wr,wd;wire [27:0] wa;wire [3:0] wl;wire [255:0] data;
 wire rl;wire [255:0] rd;reg inject_error=0;wire [3:0] rid=inject_error?4'd1:4'd0;
 realtime_remap #(.WIDTH(W),.HEIGHT(HST),.CACHE_INDEX_BITS(CACHE_BITS),.GRID_MAP(1),.MAP_STEP(STEP),
  .SRC_WIDTH(W),.SRC_HEIGHT(HSRC)) dut(
  .cmd_bypass(1'b0),.clk(clk),.rst_n(rst_n),.cmd_valid(cmd),.cmd_ready(ready),.src_base(SRC),.dst_base(DST),.map_base(MAP),
  .cmd_affine(1'b0),.cmd_gamma(1'b0),.cmd_gamma_mode(3'd0),.cmd_matrix(192'b0),.gamma_curve(),
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
 reg [15:0] memory[0:4450000],expected[0:N-1];reg [63:0] expected_neighbors[0:N-1];
 integer neighbor_index=0;
 always @(posedge clk)begin
  if(cmd&&ready)neighbor_index<=0;
  else if(dut.cv&&dut.cr)begin
   if(dut.neighbors!==expected_neighbors[neighbor_index])
    $fatal(1,"neighbors index=%0d desc=%h got=%h expected=%h",neighbor_index,dut.cache.desc,dut.neighbors,expected_neighbors[neighbor_index]);
   neighbor_index<=neighbor_index+1;
  end
 end
 reg rb=0,wb=0;reg [31:0] read_at,write_at;reg [4:0] rbeat,wbeat,rsz,wsz;
 reg write_completion=0;integer j,k,px,py,sxq,syq;reg [255:0] read_data;
 assign arready=!rb&&clock_count%7!=0;assign awready=!wb&&!write_completion&&clock_count%5!=0;
 assign rvalid=rb&&clock_count%4!=0;assign rl=rbeat==rsz-1;assign rd=read_data;
 assign wready=wb&&clock_count%3!=0;
 assign wdone=write_completion;
 integer read_lane;
 always @(read_at or rbeat)for(read_lane=0;read_lane<16;read_lane=read_lane+1)
  read_data[read_lane*16+:16]=memory[read_at+rbeat*16+read_lane];
 reg ar_stalled=0,aw_stalled=0;reg [31:0] held_ar,held_aw;
 always @(posedge clk)begin
  if(rst_n)begin
   clock_count<=clock_count+1;
   if(ar_stalled&&(!arvalid||{arlen,araddr}!=held_ar))$fatal(1,"AR changed under stall");
   if(aw_stalled&&(!awvalid||{awlen,awaddr}!=held_aw))$fatal(1,"AW changed under stall");
   ar_stalled<=arvalid&&!arready;held_ar<={arlen,araddr};
   aw_stalled<=awvalid&&!awready;held_aw<={awlen,awaddr};
   if(clock_count%401==0)display_request<=1;
   if(display_request&&car[1])display_request<=0;
   if(clock_count%197==0)capture_request<=1;
   if(capture_request&&caw[0])capture_request<=0;
   if(arvalid&&arready)begin rb<=1;read_at<=araddr*2;rbeat<=0;rsz<=arlen+1;end
   if(rvalid)begin rbeat<=rbeat+1;if(rl)rb<=0;end
   if(awvalid&&awready)begin wb<=1;write_at<=awaddr*2;wbeat<=0;wsz<=awlen+1;end
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
  reg [63:0] sum;reg [7:0] channels[0:2];integer m;
  begin
   for(m=0;m<3;m=m+1)begin
    sum=64'(chan(a,m))*(65536-dx)*(65536-dy)+64'(chan(b,m))*dx*(65536-dy)+
        64'(chan(c,m))*(65536-dx)*dy+64'(chan(d,m))*dx*dy+64'h80000000;
    channels[m]=sum>>32;
   end
   reference={channels[2][7:3],channels[1][7:2],channels[0][7:3]};
  end
 endfunction
 function [15:0] sample(input integer x,y);
  sample=(x<0||x>=W||y<0||y>=HSRC)?0:memory[SRC/2+y*W+x];
 endfunction
 // Equidistant fisheye warp at a control point.
 function real warp_x(input integer u,v);
  real nx,ny,rad,th,thd,scale;
  begin
   nx=(u-CX)/FP_OUT;ny=(v-CY)/FP_OUT;rad=$sqrt(nx*nx+ny*ny);
   th=rad>0.0?$atan2(rad,1.0):0.0;
   thd=th*(1.0+K1*th*th+K2*th*th*th*th+K3*th*th*th*th*th*th);
   scale=rad>1.0e-9?(FP_RAW*thd)/rad:FP_RAW;
   warp_x=scale*nx+CX;
  end
 endfunction
 function real warp_y(input integer u,v);
  real nx,ny,rad,th,thd,scale;
  begin
   nx=(u-CX)/FP_OUT;ny=(v-CY)/FP_OUT;rad=$sqrt(nx*nx+ny*ny);
   th=rad>0.0?$atan2(rad,1.0):0.0;
   thd=th*(1.0+K1*th*th+K2*th*th*th*th+K3*th*th*th*th*th*th);
   scale=rad>1.0e-9?(FP_RAW*thd)/rad:FP_RAW;
   warp_y=scale*ny+CY;
  end
 endfunction
 longint cp_x[0:NROW*NCOL-1],cp_y[0:NROW*NCOL-1];
 task build_grid;
  integer r,c,idx;
  begin
   for(r=0;r<NROW;r=r+1)for(c=0;c<NCOL;c=c+1)begin
    idx=r*NCOL+c;
    cp_x[idx]=longint'($floor(warp_x(c*STEP,ROW0+r*STEP)*65536.0));
    cp_y[idx]=longint'($floor(warp_y(c*STEP,ROW0+r*STEP)*65536.0));
    memory[MAP/2+(r*STRIDE+c)*4+0]=cp_x[idx][15:0];
    memory[MAP/2+(r*STRIDE+c)*4+1]=cp_x[idx][31:16];
    memory[MAP/2+(r*STRIDE+c)*4+2]=cp_y[idx][15:0];
    memory[MAP/2+(r*STRIDE+c)*4+3]=cp_y[idx][31:16];
   end
  end
 endtask
 task interp(input integer u,v,output integer osx,osy);
  integer gx,gy,fx,fy;longint vx,vy;
  begin
   gx=u>>2;gy=v>>2;fx=u&3;fy=v&3;
   vx=(4-fx)*(4-fy)*cp_x[gy*NCOL+gx]+fx*(4-fy)*cp_x[gy*NCOL+gx+1]+
      (4-fx)*fy*cp_x[(gy+1)*NCOL+gx]+fx*fy*cp_x[(gy+1)*NCOL+gx+1];
   vy=(4-fx)*(4-fy)*cp_y[gy*NCOL+gx]+fx*(4-fy)*cp_y[gy*NCOL+gx+1]+
      (4-fx)*fy*cp_y[(gy+1)*NCOL+gx]+fx*fy*cp_y[(gy+1)*NCOL+gx+1];
   osx=int'(vx>>>4);osy=int'(vy>>>4);
  end
 endtask
 integer bad_pixels;real cpp;
 initial begin
  repeat(5)@(posedge clk);@(negedge clk);rst_n=1;
  for(k=0;k<W*HSRC;k=k+1)memory[SRC/2+k]=16'((k*37)^((k/W)*271));
  build_grid;
  for(k=0;k<N;k=k+1)begin
   px=k%W;py=k/W;
   interp(px,py,sxq,syq);
   expected_neighbors[k]={sample((sxq>>>16)+1,(syq>>>16)+1),sample(sxq>>>16,(syq>>>16)+1),
                          sample((sxq>>>16)+1,syq>>>16),sample(sxq>>>16,syq>>>16)};
   expected[k]=reference(sample(sxq>>>16,syq>>>16),sample((sxq>>>16)+1,syq>>>16),
                         sample(sxq>>>16,(syq>>>16)+1),sample((sxq>>>16)+1,(syq>>>16)+1),
                         sxq[15:0],syq[15:0]);
   memory[DST/2+k]=16'hdead;
  end
  @(negedge clk);cmd=1;do @(posedge clk);while(!ready);@(negedge clk);cmd=0;
  wait(rsp);#1;
  if(status!==8'd0)$fatal(1,"grid fullframe status %0d",status);
  bad_pixels=0;
  for(k=0;k<N;k=k+1)if(memory[DST/2+k]!==expected[k])bad_pixels=bad_pixels+1;
  if(bad_pixels!=0)$fatal(1,"grid fullframe %0d/%0d pixels differ",bad_pixels,N);
  cpp=cycles/real'(N);
  $display("grid fullframe rows %0d..%0d grid=%0dx%0d map_bytes=%0d pixels=%0d cycles=%0d cycles_per_pixel=%0.3f",
   ROW0,ROW0+HST-1,NCOL,NROW,NROW*STRIDE*8,N,cycles,cpp);
  if(cpp>4.52)$display("RESULT OVER BUDGET: %0.3f > 4.52 cycles/pixel",cpp);
  else $display("RESULT WITHIN BUDGET: %0.3f <= 4.52 cycles/pixel",cpp);
  finished=1;$display("PASS grid map full-frame fisheye warp throughput and pixels");$finish;
 end
 initial begin #200000000;$fatal(1,"timeout state=%0d cache=%0d writer=%0d index=%0d",dut.state,dut.cache.state,dut.writer.state,neighbor_index);end
endmodule
