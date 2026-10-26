`timescale 1ns/1ps
// Runtime contains NO floating-point arithmetic. Packed inverse map -> cached
// neighbors -> elastic RGB565 interpolation -> native burst writer.
module realtime_remap #(parameter WIDTH=1280,HEIGHT=720,CACHE_INDEX_BITS=6,ENABLE_ADVANCED=0)(
 input wire clk,rst_n,cmd_valid,output wire cmd_ready,
 input wire [31:0] src_base,dst_base,map_base,
 input wire cmd_bypass, // Identity sampling for before/after display, no map read.
 input wire cmd_affine,cmd_gamma,input wire [2:0] cmd_gamma_mode,input wire [191:0] cmd_matrix,
 output wire [2:0] gamma_curve,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output reg [31:0] last_cycles,
 output wire [1:0] arvalid,input wire [1:0] arready,
 output wire [55:0] araddr,output wire [7:0] arlen,
 input wire [1:0] rvalid,input wire [255:0] rdata,input wire rlast,input wire [3:0] rid,
 output wire awvalid,input wire awready,output wire [27:0] awaddr,output wire [3:0] awlen,
 output wire [255:0] wdata,input wire wready,wdone,input wire [3:0] wid
);
 localparam IDLE=0,START=1,RUN=2,RESPONSE=3;
 reg [1:0] state;reg [31:0] source,destination,mapping,cycles;
 // Dedicated start pulse prevents the state decoder driving all writer CEs.
 reg start;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)start<=0;else start<=state==START;
 wire mv,mr,cv,cr,pv,pr,me,ce,we,done;
 reg bypass,identity_valid;
 reg affine,gamma_enable;reg [2:0] gamma_mode;reg [191:0] matrix;
 reg [$clog2(WIDTH)-1:0] identity_x;
 reg [$clog2(HEIGHT)-1:0] identity_y;
 wire [13:0] identity_x14=identity_x,identity_y14=identity_y;
 wire map_valid,map_ready,map_error;
 wire [63:0] map_descriptor;
 wire affine_valid;wire [63:0] affine_descriptor;
 wire [63:0] descriptor=affine?affine_descriptor:bypass?{4'b0001,identity_y14,identity_x14,32'b0}:map_descriptor;
 generate if(ENABLE_ADVANCED)begin : g_affine
  video_affine_coords #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) coords(.clk(clk),.rst_n(rst_n),.start(start&&affine),
   .matrix(matrix),.out_valid(affine_valid),.out_ready(mr),.descriptor(affine_descriptor));
 end else begin assign affine_valid=0;assign affine_descriptor=0;end endgenerate
 wire [63:0] neighbors;
 reg descriptor_valid,descriptor_error;reg [63:0] descriptor_q;
 wire cache_ready;
 assign mr=!descriptor_valid||cache_ready;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin descriptor_valid<=0;descriptor_error<=0;descriptor_q<=0;end
  else if(start)begin descriptor_valid<=0;descriptor_error<=0;end
  else if(mr)begin
   descriptor_valid<=mv;
   if(mv)begin descriptor_q<=descriptor;descriptor_error<=me;end
  end
 end
 assign mv=affine?affine_valid:bypass?identity_valid:map_valid;
 assign map_ready=!bypass&&!affine&&mr;
 assign me=!bypass&&!affine&&map_error;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin identity_valid<=0;identity_x<=0;identity_y<=0;end
  else if(start)begin identity_valid<=bypass&&!affine;identity_x<=0;identity_y<=0;end
  else if(bypass&&identity_valid&&mr)begin
   if(identity_x==WIDTH-1)begin
    identity_x<=0;
    if(identity_y==HEIGHT-1)identity_valid<=0;
    else identity_y<=identity_y+1'b1;
   end else identity_x<=identity_x+1'b1;
  end
 end
 wire [15:0] dx,dy,pixel;
 assign cmd_ready=state==IDLE;assign rsp_valid=state==RESPONSE;
 video_map_prefetch #(.PIXELS(WIDTH*HEIGHT)) maps(
  .clk(clk),.rst_n(rst_n),.start(start&&!bypass&&!affine),.map_base(mapping),.out_valid(map_valid),.out_ready(map_ready),.descriptor(map_descriptor),.error(map_error),
  .arvalid(arvalid[0]),.arready(arready[0]),.araddr(araddr[0+:28]),.arlen(arlen[0+:4]),
  .rvalid(rvalid[0]),.rdata(rdata),.rlast(rlast),.rid(rid));
 wire cache_bounds,pixel_bounds;
 video_neighbor_cache #(.WIDTH(WIDTH),.INDEX_BITS(CACHE_INDEX_BITS)) cache(
  .clk(clk),.rst_n(rst_n),.start(start),.src_base(source),.in_valid(descriptor_valid),.in_ready(cache_ready),.descriptor(descriptor_q),.map_error(descriptor_error),
  .out_valid(cv),.out_ready(cr),.pixels(neighbors),.dx(dx),.dy(dy),.error(ce),.out_in_bounds(cache_bounds),
  .arvalid(arvalid[1]),.arready(arready[1]),.araddr(araddr[28+:28]),.arlen(arlen[4+:4]),
  .rvalid(rvalid[1]),.rdata(rdata),.rlast(rlast),.rid(rid));
 video_bilinear_pipe #(.WITH_BOUNDS(ENABLE_ADVANCED)) interpolate(.clk(clk),.rst_n(rst_n),.in_valid(cv),.in_ready(cr),
  .pixels(neighbors),.dx(dx),.dy(dy),.out_valid(pv),.out_ready(pr),.pixel(pixel),.in_bounds(cache_bounds),.out_in_bounds(pixel_bounds));
 wire writer_valid,writer_ready;wire [15:0] writer_pixel;
 generate if(ENABLE_ADVANCED)begin : g_gamma
  reg [$clog2(WIDTH*HEIGHT)-1:0] at;
  always @(posedge clk or negedge rst_n)
   if(!rst_n)at<=0;else if(start)at<=0;else if(pv&&pr)at<=at==WIDTH*HEIGHT-1?0:at+1'b1;
  video_adaptive_gamma enhance(.clk(clk),.rst_n(rst_n),.in_valid(pv),.in_ready(pr),.in_pixel(pixel),
   .in_first(at==0),.in_last(at==WIDTH*HEIGHT-1),.in_bounds(pixel_bounds),.enable(gamma_enable),.mode(gamma_mode),
   .out_valid(writer_valid),.out_ready(writer_ready),.out_pixel(writer_pixel),.auto_curve(gamma_curve));
 end else begin assign pr=writer_ready;assign writer_valid=pv;assign writer_pixel=pixel;assign gamma_curve=0;end endgenerate
 video_frame_writer #(.PIXELS(WIDTH*HEIGHT)) writer(
  .clk(clk),.rst_n(rst_n),.start(start),.dst_base(destination),.in_valid(writer_valid),.in_ready(writer_ready),.pixel(writer_pixel),.done(done),.error(we),
  .awvalid(awvalid),.awready(awready),.awaddr(awaddr),.awlen(awlen),.wdata(wdata),.wready(wready),.wdone(wdone),.wid(wid));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin affine<=0;gamma_enable<=0;gamma_mode<=0;matrix<=0;bypass<=0;state<=IDLE;source<=0;destination<=0;mapping<=0;cycles<=0;last_cycles<=0;rsp_status<=0;end
  else case(state)
   IDLE:if(cmd_valid)begin
    bypass<=cmd_bypass;source<=src_base;destination<=dst_base;mapping<=map_base;cycles<=0;rsp_status<=0;
    affine<=ENABLE_ADVANCED&&cmd_affine;gamma_enable<=ENABLE_ADVANCED&&cmd_gamma;
    gamma_mode<=ENABLE_ADVANCED?cmd_gamma_mode:0;matrix<=ENABLE_ADVANCED?cmd_matrix:192'b0;
    if(WIDTH%64!=0||WIDTH*HEIGHT%256!=0||WIDTH<64||HEIGHT<2||src_base[6:0]!=0||dst_base[8:0]!=0||(!cmd_bypass&&!(ENABLE_ADVANCED&&cmd_affine)&&map_base[8:0]!=0))
     begin rsp_status<=1;state<=RESPONSE;end
    else state<=START;
   end
   START:state<=RUN;
   RUN:begin cycles<=cycles+1;if(done&&!start)begin last_cycles<=cycles;rsp_status<=me||ce||we?5:0;state<=RESPONSE;end end
   RESPONSE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
