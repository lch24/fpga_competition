`timescale 1ns/1ps
// Runtime contains NO floating-point arithmetic. Packed inverse map -> cached
// neighbors -> elastic RGB565 interpolation -> native burst writer.
module realtime_remap #(parameter WIDTH=1280,HEIGHT=720,CACHE_INDEX_BITS=6)(
 input wire clk,rst_n,cmd_valid,output wire cmd_ready,
 input wire [31:0] src_base,dst_base,map_base,
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
 wire start=state==START;wire mv,mr,cv,cr,pv,pr,me,ce,we,done;
 wire [63:0] descriptor,neighbors;wire [15:0] dx,dy,pixel;
 assign cmd_ready=state==IDLE;assign rsp_valid=state==RESPONSE;
 video_map_prefetch #(.PIXELS(WIDTH*HEIGHT)) maps(
  .clk(clk),.rst_n(rst_n),.start(start),.map_base(mapping),.out_valid(mv),.out_ready(mr),.descriptor(descriptor),.error(me),
  .arvalid(arvalid[0]),.arready(arready[0]),.araddr(araddr[0+:28]),.arlen(arlen[0+:4]),
  .rvalid(rvalid[0]),.rdata(rdata),.rlast(rlast),.rid(rid));
 video_neighbor_cache #(.WIDTH(WIDTH),.INDEX_BITS(CACHE_INDEX_BITS)) cache(
  .clk(clk),.rst_n(rst_n),.start(start),.src_base(source),.in_valid(mv),.in_ready(mr),.descriptor(descriptor),.map_error(me),
  .out_valid(cv),.out_ready(cr),.pixels(neighbors),.dx(dx),.dy(dy),.error(ce),
  .arvalid(arvalid[1]),.arready(arready[1]),.araddr(araddr[28+:28]),.arlen(arlen[4+:4]),
  .rvalid(rvalid[1]),.rdata(rdata),.rlast(rlast),.rid(rid));
 video_bilinear_pipe interpolate(.clk(clk),.rst_n(rst_n),.in_valid(cv),.in_ready(cr),
  .pixels(neighbors),.dx(dx),.dy(dy),.out_valid(pv),.out_ready(pr),.pixel(pixel));
 video_frame_writer #(.PIXELS(WIDTH*HEIGHT)) writer(
  .clk(clk),.rst_n(rst_n),.start(start),.dst_base(destination),.in_valid(pv),.in_ready(pr),.pixel(pixel),.done(done),.error(we),
  .awvalid(awvalid),.awready(awready),.awaddr(awaddr),.awlen(awlen),.wdata(wdata),.wready(wready),.wdone(wdone),.wid(wid));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;source<=0;destination<=0;mapping<=0;cycles<=0;last_cycles<=0;rsp_status<=0;end
  else case(state)
   IDLE:if(cmd_valid)begin
    source<=src_base;destination<=dst_base;mapping<=map_base;cycles<=0;rsp_status<=0;
    if(WIDTH%64!=0||WIDTH*HEIGHT%256!=0||WIDTH<64||HEIGHT<2||src_base[6:0]!=0||dst_base[8:0]!=0||map_base[8:0]!=0)
     begin rsp_status<=1;state<=RESPONSE;end
    else state<=START;
   end
   START:state<=RUN;
   RUN:begin cycles<=cycles+1;if(done)begin last_cycles<=cycles;rsp_status<=me||ce||we?5:0;state<=RESPONSE;end end
   RESPONSE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
