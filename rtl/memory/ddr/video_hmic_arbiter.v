`timescale 1ns/1ps
// Demo HMIC, NOT AXI: no RREADY/WVALID; every accepted read burst must be
// absorbed, and a write owner is retained through WUSERO_LAST. One outstanding
// read and one outstanding write, independently. Requests are locked BEFORE
// asserting the physical address valid. ID zero is retained for existing IP.
module video_hmic_arbiter #(parameter NR=4,NW=3)(
 input wire clk,rst_n,
 input wire [NR-1:0] c_arvalid,output reg [NR-1:0] c_arready,
 input wire [NR*28-1:0] c_araddr,input wire [NR*4-1:0] c_arlen,
 output reg [NR-1:0] c_rvalid,output wire [255:0] c_rdata,
 output wire c_rlast,output wire [3:0] c_rid,
 input wire [NW-1:0] c_awvalid,output reg [NW-1:0] c_awready,
 input wire [NW*28-1:0] c_awaddr,input wire [NW*4-1:0] c_awlen,
 input wire [NW*256-1:0] c_wdata,input wire [NW*32-1:0] c_wstrb,
 output reg [NW-1:0] c_wready,c_wdone,output wire [3:0] c_wid,
 output wire [27:0] araddr,awaddr,output wire [3:0] arlen,awlen,
 output wire arvalid,awvalid,input wire arready,awready,
 input wire [255:0] rdata,input wire rvalid,rlast,input wire [3:0] rid,
 output wire [255:0] wdata,output wire [31:0] wstrb,
 input wire wready,wdone,input wire [3:0] wid,
 output wire idle
);
 localparam RW=$clog2(NR),WW=$clog2(NW);
 reg [RW-1:0] ro,rhead;reg [WW-1:0] wo,whead;
 reg rlock,rbusy,wlock,wbusy;integer k,j,rs,ws;
 always @*begin
  rs=-1;ws=-1;
  // Compile-time indices: avoid a wide variable add/modulo/mux network.
  for(k=0;k<NR;k=k+1)if(rhead==k)
   for(j=0;j<NR;j=j+1)if(rs<0&&c_arvalid[(k+j)%NR])rs=(k+j)%NR;
  // HDMI requests are bounded bursts; give them priority to avoid underflow.
  if(c_arvalid[1])rs=1;
  for(k=0;k<NW;k=k+1)if(whead==k)
   for(j=0;j<NW;j=j+1)if(ws<0&&c_awvalid[(k+j)%NW])ws=(k+j)%NW;
  c_arready=0;c_rvalid=0;c_awready=0;c_wready=0;c_wdone=0;
  if(rlock)c_arready[ro]=arready;
  if(rbusy)c_rvalid[ro]=rvalid;
  if(wlock)c_awready[wo]=awready;
  if(wbusy)begin c_wready[wo]=wready;c_wdone[wo]=wdone;end
 end
 // Lock is set only for an asserted client request. Clients hold valid and
 // payload until ready (including display shutdown: requests drain first).
 // The registered lock is therefore already the physical valid. Re-muxing
 // client valid here needlessly creates owner -> valid -> DDR ready -> client.
 assign arvalid=rlock;assign araddr=c_araddr[ro*28+:28];assign arlen=c_arlen[ro*4+:4];
 assign awvalid=wlock;assign awaddr=c_awaddr[wo*28+:28];assign awlen=c_awlen[wo*4+:4];
 assign wdata=c_wdata[wo*256+:256];assign wstrb=c_wstrb[wo*32+:32];
 assign c_rdata=rdata;assign c_rlast=rlast;assign c_rid=rid;assign c_wid=wid;
 assign idle=!(rlock||rbusy||wlock||wbusy);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ro<=0;wo<=0;rhead<=0;whead<=0;rlock<=0;rbusy<=0;wlock<=0;wbusy<=0;end
  else begin
   if(!rlock&&!rbusy&&rs>=0)begin ro<=rs;rlock<=1;end
   if(arvalid&&arready)begin rlock<=0;rbusy<=1;end
   if(rbusy&&rvalid&&rlast)begin rbusy<=0;rhead<=ro==NR-1?0:ro+1'b1;end
   if(!wlock&&!wbusy&&ws>=0)begin wo<=ws;wlock<=1;end
   if(awvalid&&awready)begin wlock<=0;wbusy<=1;end
   if(wbusy&&wdone)begin wbusy<=0;whead<=wo==NW-1?0:wo+1'b1;end
  end
 end
endmodule
