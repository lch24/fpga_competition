`timescale 1ns/1ps
// Offline map: one 64-bit descriptor per destination pixel, little endian.
// [15:0] fx, [31:16] fy (Q0.16), [45:32] signed x0, [59:46] signed y0,
// [63:60] valid neighbor mask {p11,p01,p10,p00}. Black border, not clamp.
// Shares the existing tested FP32 map computation; no per-video-frame FP.
// GRID_MAP=0 keeps the legacy one-descriptor-per-pixel format.  GRID_MAP=1
// writes a control-point table every MAP_STEP destination pixels:
// [31:0] source x Q16.16, [63:32] source y Q16.16 (both signed).  Caller
// passes the grid dimensions in cmd_width/cmd_height and the padded row
// stride (in points) in GRID_STRIDE; IMAGE_WIDTH/HEIGHT are the real frame
// size used by quantisation/mask.
module packed_map_builder #(parameter ENABLE_DISTORT_DEMO=0,parameter GRID_MAP=0,
 parameter MAP_STEP=1,parameter GRID_STRIDE=0,parameter IMAGE_WIDTH=0,parameter IMAGE_HEIGHT=0)(
 input wire clk,rst_n,cmd_valid,output wire cmd_ready,
 input wire [1:0] cmd_opcode,input wire [287:0] cmd_params,
 input wire [15:0] cmd_width,cmd_height,input wire [31:0] cmd_map_base,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,output wire w_valid,input wire w_ready,
 output wire [31:0] w_data,output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 localparam IDLE=0,CONFIG=1,RUN=2,WRITE_LO=3,WAIT_LO=4,WRITE_HI=5,WAIT_HI=6,DONE=7;
 reg inverse;
 reg [2:0] state;reg [287:0] params;reg [15:0] width,height,x,y;
 reg [31:0] base,pixel_id,address;reg [63:0] descriptor;reg last,issued;
 wire [15:0] image_width=(IMAGE_WIDTH==0)?width:IMAGE_WIDTH[15:0];
 wire [15:0] image_height=(IMAGE_HEIGHT==0)?height:IMAGE_HEIGHT[15:0];
 wire [15:0] max_width=(IMAGE_WIDTH==0)?16'd1920:IMAGE_WIDTH[15:0];
 wire [15:0] max_height=(IMAGE_HEIGHT==0)?16'd1080:IMAGE_HEIGHT[15:0];
 wire [31:0] stride_points=(GRID_STRIDE==0)?{16'd0,width}:GRID_STRIDE[31:0];
 wire [31:0] cmd_stride_points=(GRID_STRIDE==0)?{16'd0,cmd_width}:GRID_STRIDE[31:0];
 wire [63:0] map_bytes=64'd8*({32'd0,cmd_stride_points}*(cmd_height==0?64'd0:(cmd_height-1))+cmd_width);
 wire [15:0] core_dst_x=x*MAP_STEP,core_dst_y=y*MAP_STEP;
 integer parameter_index;reg parameters_ok;
 always @*begin
  parameters_ok=1;
  for(parameter_index=0;parameter_index<9;parameter_index=parameter_index+1)
   if(cmd_params[parameter_index*32+23+:8]==255)parameters_ok=0;
  if(cmd_params[31]||cmd_params[63]||cmd_params[30:0]==0||cmd_params[62:32]==0)parameters_ok=0;
 end
 wire cfg_ready,ci_ready,co_valid,co_last,co_error;
 wire [31:0] sx,sy,co_id;wire [15:0] ox,oy;
 wire qv,qr,rv,rr,re;wire [2:0] op;wire [31:0] a,b,result;
 assign cmd_ready=state==IDLE;assign rsp_valid=state==DONE;
 map_coord_core #(.ENABLE_DISTORT_DEMO(ENABLE_DISTORT_DEMO)) core(.cfg_inverse(inverse),.clk(clk),.rst_n(rst_n),.cfg_valid(state==CONFIG),.cfg_ready(cfg_ready),
  .cfg_fx(params[31:0]),.cfg_fy(params[63:32]),.cfg_cx(params[95:64]),.cfg_cy(params[127:96]),
  .cfg_k1(params[159:128]),.cfg_k2(params[191:160]),.cfg_k3(params[223:192]),.cfg_p1(params[255:224]),.cfg_p2(params[287:256]),
  .in_valid(state==RUN&&!issued),.in_ready(ci_ready),.in_dst_x(core_dst_x),.in_dst_y(core_dst_y),.in_pixel_id(pixel_id),.in_last(x==width-1&&y==height-1),
  .out_valid(co_valid),.out_ready(state==RUN),.out_src_x(sx),.out_src_y(sy),.out_dst_x(ox),.out_dst_y(oy),
  .out_pixel_id(co_id),.out_last(co_last),.out_error(co_error),
  .fp_req_valid(qv),.fp_req_ready(qr),.fp_req_op(op),.fp_req_a(a),.fp_req_b(b),
  .fp_rsp_valid(rv),.fp_rsp_ready(rr),.fp_rsp_result(result),.fp_rsp_error(re));
 fp32_service arithmetic(.clk(clk),.rst_n(rst_n),.req_valid(qv),.req_ready(qr),.req_op(op),.req_a(a),.req_b(b),
  .rsp_valid(rv),.rsp_ready(rr),.rsp_result(result),.rsp_error(re));
 wire signed [31:0] x0,y0,x1,y1;wire [15:0] dx,dy;wire [3:0] mask;
 sample_coord quantize(.src_x(sx),.src_y(sy),.width(image_width),.height(image_height),.border_replicate(1'b0),
  .x0(x0),.y0(y0),.x1(x1),.y1(y1),.dx(dx),.dy(dy),.mask(mask));
 // Clamp extreme (mostly non-finite) source coordinates before packing Q16.16.
 // Any real source pixel is inside +/-32768, so clamped points are far outside
 // the image and the reader derives mask=0 for them.
 wire signed [31:0] gsx=(x0>32'sd32767)?32'h7fff0000:(x0<-32'sd32768)?32'h80000000:{x0[15:0],dx};
 wire signed [31:0] gsy=(y0>32'sd32767)?32'h7fff0000:(y0<-32'sd32768)?32'h80000000:{y0[15:0],dy};
 wire [31:0] point_index=y*stride_points+x;
 wire [63:0] map_word=GRID_MAP?{gsy,gsx}:{mask,y0[13:0],x0[13:0],dy,dx};
 wire wi,wo,we;
 output_writer writer(.clk(clk),.rst_n(rst_n),.in_valid(state==WRITE_LO||state==WRITE_HI),.in_ready(wi),
  .in_addr(state==WRITE_LO?address:address+4),.in_data(state==WRITE_LO?descriptor[31:0]:descriptor[63:32]),.in_bytes(3'd4),
  .out_valid(wo),.out_ready(state==WAIT_LO||state==WAIT_HI),.out_error(we),
  .wr_valid(wr_valid),.wr_ready(wr_ready),.wr_addr(wr_addr),.wr_len(wr_len),.wr_tag(wr_tag),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_keep(w_keep),.w_last(w_last),
  .b_valid(b_valid),.b_ready(b_ready),.b_tag(b_tag),.b_error(b_error));
 // Issue ONE coordinate, wait for its descriptor before accepting the next.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;inverse<=0;rsp_status<=0;params<=0;width<=0;height<=0;x<=0;y<=0;base<=0;pixel_id<=0;descriptor<=0;address<=0;last<=0;issued<=0;end
  else case(state)
   IDLE:if(cmd_valid)begin
    inverse<=ENABLE_DISTORT_DEMO && cmd_opcode==3;
    params<=cmd_params;width<=cmd_width;height<=cmd_height;base<=cmd_map_base;x<=0;y<=0;pixel_id<=0;issued<=0;rsp_status<=0;
    if(!parameters_ok||(cmd_opcode!=1&&!(ENABLE_DISTORT_DEMO&&cmd_opcode==3))||cmd_width==0||cmd_height==0||cmd_width>max_width||cmd_height>max_height||cmd_map_base[4:0]!=0||
       {32'b0,cmd_map_base}+map_bytes>64'h40000000)begin rsp_status<=1;state<=DONE;end
    else state<=CONFIG;
   end
   CONFIG:if(cfg_ready)state<=RUN;
   RUN:begin
    if(ci_ready&&!issued)issued<=1;
    if(co_valid)begin
     descriptor<=map_word;address<=base+(point_index<<3);last<=co_last;
     if(co_error)begin rsp_status<=4;state<=DONE;end else state<=WRITE_LO;
    end
   end
   WRITE_LO:if(wi)state<=WAIT_LO;
   WAIT_LO:if(wo)begin if(we)begin rsp_status<=5;state<=DONE;end else state<=WRITE_HI;end
   WRITE_HI:if(wi)state<=WAIT_HI;
   WAIT_HI:if(wo)begin
    if(we)begin rsp_status<=5;state<=DONE;end else if(last)state<=DONE;
    else begin pixel_id<=pixel_id+1;issued<=0;if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;state<=RUN;end
   end
   DONE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
