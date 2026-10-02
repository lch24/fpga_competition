`timescale 1ns/1ps
// RGB565 DDR->DDR processor: opcode 1 BUILD_MAP, opcode 2 REMAP.
// Parameters packed low-to-high: fx,fy,cx,cy,k1,k2,k3,p1,p2 (raw FP32).
// One job at a time; job response is sticky until accepted. Map publication
// occurs after both planes' actual write completions. All reads/writes are
// logical byte transactions routed through the shared DDR service.
module undistort_top (
 input wire clk,rst_n,
 input wire cmd_valid,output wire cmd_ready,input wire [1:0] cmd_opcode,
 input wire [31:0] cmd_job_id,cmd_calib_id,input wire cmd_camera_valid,
 input wire [287:0] cmd_params,
 input wire [15:0] cmd_width,cmd_height,
 input wire [31:0] cmd_src_base,cmd_dst_base,cmd_map_x_base,cmd_map_y_base,
 input wire [31:0] cmd_src_stride,cmd_dst_stride,cmd_map_stride,
 input wire [31:0] cmd_src_capacity,cmd_dst_capacity,cmd_map_capacity,
 input wire cmd_border_replicate,
 output wire rsp_valid,input wire rsp_ready,output reg [31:0] rsp_job_id,
 output reg [7:0] rsp_status,output wire busy,output reg map_valid,
 output reg [31:0] map_calib_id,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,rd_len,
 output wire [15:0] rd_tag,
 input wire r_valid,output wire r_ready,input wire [31:0] r_data,
 input wire [3:0] r_keep,input wire [15:0] r_tag,input wire r_last,r_error,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,
 output wire w_valid,input wire w_ready,output wire [31:0] w_data,
 output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 localparam IDLE=0,CFG_CORE=1,CFG_WRITER=2,BUILD=3,READ_MAP=4,
  WAIT_MAP=5,SAMPLE=6,FETCH=7,WAIT_FETCH=8,WRITE_PIXEL=9,WAIT_PIXEL=10,RESPONSE=11;
 reg [3:0] state;
 reg [31:0] calib,src,dst,mx,my,ss,ds,ms;
 reg [15:0] width,height,x,y;
 reg [287:0] params;
 reg border,issued_last,numeric_error;
 reg [31:0] pixel_id;
 reg [15:0] frac_x,frac_y,pixel;
 reg [15:0] published_w,published_h;
 reg [31:0] published_x,published_y,published_stride;
 wire check_valid;
 descriptor_check check(.build_map(cmd_opcode==1),.width(cmd_width),.height(cmd_height),
  .src_base(cmd_src_base),.dst_base(cmd_dst_base),.map_x_base(cmd_map_x_base),.map_y_base(cmd_map_y_base),
  .src_stride(cmd_src_stride),.dst_stride(cmd_dst_stride),.map_stride(cmd_map_stride),
  .src_capacity(cmd_src_capacity),.dst_capacity(cmd_dst_capacity),.map_capacity(cmd_map_capacity),.valid(check_valid));
 wire finite_params=(cmd_params[30:23]!=255)&&(cmd_params[62:55]!=255)&&
  (cmd_params[94:87]!=255)&&(cmd_params[126:119]!=255)&&(cmd_params[158:151]!=255)&&
  (cmd_params[190:183]!=255)&&(cmd_params[222:215]!=255)&&(cmd_params[254:247]!=255)&&
  (cmd_params[286:279]!=255)&&!cmd_params[31]&&!cmd_params[63]&&
  cmd_params[30:0]!=0&&cmd_params[62:32]!=0;
 wire table_matches=map_valid&&cmd_calib_id==map_calib_id&&cmd_width==published_w&&
  cmd_height==published_h&&cmd_map_x_base==published_x&&cmd_map_y_base==published_y&&cmd_map_stride==published_stride;
 assign cmd_ready=state==IDLE;assign busy=state!=IDLE;assign rsp_valid=state==RESPONSE;
 wire last_pixel=x==width-1&&y==height-1;

 wire core_cfg_ready,ci_ready,co_valid,co_ready,co_last,co_error;
 wire [31:0] co_x,co_y,co_id;wire [15:0] co_dx,co_dy;
 wire fpq_valid,fpq_ready,fpr_valid,fpr_ready,fpr_error;
 wire [2:0] fpq_op;wire [31:0] fpq_a,fpq_b,fpr_data;
 map_coord_core core(.clk(clk),.rst_n(rst_n),.cfg_valid(state==CFG_CORE),.cfg_ready(core_cfg_ready),
  .cfg_fx(params[31:0]),.cfg_fy(params[63:32]),.cfg_cx(params[95:64]),.cfg_cy(params[127:96]),
  .cfg_k1(params[159:128]),.cfg_k2(params[191:160]),.cfg_k3(params[223:192]),
  .cfg_p1(params[255:224]),.cfg_p2(params[287:256]),
  .in_valid(state==BUILD&&!issued_last),.in_ready(ci_ready),.in_dst_x(x),.in_dst_y(y),
  .in_pixel_id(pixel_id),.in_last(last_pixel),.out_valid(co_valid),.out_ready(co_ready),
  .out_src_x(co_x),.out_src_y(co_y),.out_dst_x(co_dx),.out_dst_y(co_dy),
  .out_pixel_id(co_id),.out_last(co_last),.out_error(co_error),
  .fp_req_valid(fpq_valid),.fp_req_ready(fpq_ready),.fp_req_op(fpq_op),.fp_req_a(fpq_a),.fp_req_b(fpq_b),
  .fp_rsp_valid(fpr_valid),.fp_rsp_ready(fpr_ready),.fp_rsp_result(fpr_data),.fp_rsp_error(fpr_error));
 fp32_service fp(.clk(clk),.rst_n(rst_n),.req_valid(fpq_valid),.req_ready(fpq_ready),
  .req_op(fpq_op),.req_a(fpq_a),.req_b(fpq_b),.rsp_valid(fpr_valid),.rsp_ready(fpr_ready),.rsp_result(fpr_data),.rsp_error(fpr_error));
 wire mw_cfg_ready,mw_ready,mw_rsp_valid,mw_error;
 wire mwr_valid,mwr_ready,mw_valid,mw_ready_data,mb_ready;
 wire [31:0] mwr_addr,mwr_len,mw_data;wire [15:0] mwr_tag;wire [3:0] mw_keep;wire mw_last;
 assign co_ready=state==BUILD&&mw_ready;
 map_table_writer map_writer(.clk(clk),.rst_n(rst_n),.cfg_valid(state==CFG_WRITER),.cfg_ready(mw_cfg_ready),
  .cfg_width(width),.cfg_height(height),.cfg_map_x_base(mx),.cfg_map_y_base(my),.cfg_stride(ms),
  .map_valid(state==BUILD&&co_valid),.map_ready(mw_ready),.map_src_x(co_x),.map_src_y(co_y),
  .map_dst_x(co_dx),.map_dst_y(co_dy),.map_pixel_id(co_id),.map_last(co_last),.map_error(co_error),
  .rsp_valid(mw_rsp_valid),.rsp_ready(state==BUILD),.rsp_error(mw_error),
  .wr_valid(mwr_valid),.wr_ready(mwr_ready),.wr_addr(mwr_addr),.wr_len(mwr_len),.wr_tag(mwr_tag),
  .w_valid(mw_valid),.w_ready(mw_ready_data),.w_data(mw_data),.w_keep(mw_keep),.w_last(mw_last),
  .b_valid(state==BUILD&&b_valid),.b_ready(mb_ready),.b_tag(b_tag),.b_error(b_error));

 wire mr_in_ready,mr_out_valid,mr_error,mrr_valid,mrr_ready,mr_rready;
 wire [31:0] sx,sy,mrr_addr,mrr_len;wire [15:0] mrr_tag;
 map_reader reader(.clk(clk),.rst_n(rst_n),.map_x_base(mx),.map_y_base(my),.map_stride(ms),
  .in_valid(state==READ_MAP),.in_ready(mr_in_ready),.in_x(x),.in_y(y),
  .out_valid(mr_out_valid),.out_ready(state==SAMPLE),.src_x(sx),.src_y(sy),.out_error(mr_error),
  .rd_valid(mrr_valid),.rd_ready(mrr_ready),.rd_addr(mrr_addr),.rd_len(mrr_len),.rd_tag(mrr_tag),
  .r_valid((state==WAIT_MAP)&&r_valid),.r_ready(mr_rready),.r_data(r_data),.r_keep(r_keep),.r_tag(r_tag),.r_last(r_last),.r_error(r_error));
 wire signed [31:0] x0,y0,x1,y1;wire [15:0] dx,dy;wire [3:0] mask;
 sample_coord sample(.src_x(sx),.src_y(sy),.width(width),.height(height),.border_replicate(border),
  .x0(x0),.y0(y0),.x1(x1),.y1(y1),.dx(dx),.dy(dy),.mask(mask));
 wire nf_in_ready,nf_out_valid,nf_error,nfr_valid,nfr_ready,nf_rready;
 wire [63:0] pixels;wire [31:0] nfr_addr,nfr_len;wire [15:0] nfr_tag;
 neighbor_fetch fetch(.clk(clk),.rst_n(rst_n),.src_base(src),.src_stride(ss),
  .in_valid(state==FETCH),.in_ready(nf_in_ready),.x0(x0),.y0(y0),.x1(x1),.y1(y1),.mask(mask),
  .out_valid(nf_out_valid),.out_ready(state==WAIT_FETCH),.pixels(pixels),.out_error(nf_error),
  .rd_valid(nfr_valid),.rd_ready(nfr_ready),.rd_addr(nfr_addr),.rd_len(nfr_len),.rd_tag(nfr_tag),
  .r_valid(state==WAIT_FETCH&&r_valid),.r_ready(nf_rready),.r_data(r_data),.r_keep(r_keep),.r_tag(r_tag),.r_last(r_last),.r_error(r_error));
 wire [15:0] interpolated;
 bilinear_rgb565 interp(.p00(pixels[15:0]),.p10(pixels[31:16]),.p01(pixels[47:32]),.p11(pixels[63:48]),.dx(frac_x),.dy(frac_y),.pixel(interpolated));
 wire pw_in_ready,pw_out_valid,pw_error,pwr_valid,pwr_ready,pw_valid,pw_ready_data,pb_ready;
 wire [31:0] pwr_addr,pwr_len,pw_data;wire [15:0] pwr_tag;wire [3:0] pw_keep;wire pw_last;
 output_writer pixel_writer(.clk(clk),.rst_n(rst_n),.in_valid(state==WRITE_PIXEL),.in_ready(pw_in_ready),
  .in_addr(dst+y*ds+({16'd0,x}<<1)),.in_data({16'd0,pixel}),.in_bytes(3'd2),
  .out_valid(pw_out_valid),.out_ready(state==WAIT_PIXEL),.out_error(pw_error),
  .wr_valid(pwr_valid),.wr_ready(pwr_ready),.wr_addr(pwr_addr),.wr_len(pwr_len),.wr_tag(pwr_tag),
  .w_valid(pw_valid),.w_ready(pw_ready_data),.w_data(pw_data),.w_keep(pw_keep),.w_last(pw_last),
  .b_valid(state==WAIT_PIXEL&&b_valid),.b_ready(pb_ready),.b_tag(b_tag),.b_error(b_error));

 wire map_read_selected=state==READ_MAP||state==WAIT_MAP||state==SAMPLE;
 assign rd_valid=map_read_selected?mrr_valid:nfr_valid;
 assign rd_addr=map_read_selected?mrr_addr:nfr_addr;
 assign rd_len=map_read_selected?mrr_len:nfr_len;
 assign rd_tag=map_read_selected?mrr_tag:nfr_tag;
 assign mrr_ready=map_read_selected&&rd_ready;assign nfr_ready=!map_read_selected&&rd_ready;
 assign r_ready=map_read_selected?mr_rready:nf_rready;
 wire build_selected=state==BUILD;
 assign wr_valid=build_selected?mwr_valid:pwr_valid;assign wr_addr=build_selected?mwr_addr:pwr_addr;
 assign wr_len=build_selected?mwr_len:pwr_len;assign wr_tag=build_selected?mwr_tag:pwr_tag;
 assign w_valid=build_selected?mw_valid:pw_valid;assign w_data=build_selected?mw_data:pw_data;
 assign w_keep=build_selected?mw_keep:pw_keep;assign w_last=build_selected?mw_last:pw_last;
 assign b_ready=build_selected?mb_ready:pb_ready;
 assign mwr_ready=build_selected&&wr_ready;assign pwr_ready=!build_selected&&wr_ready;
 assign mw_ready_data=build_selected&&w_ready;assign pw_ready_data=!build_selected&&w_ready;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;rsp_status<=0;rsp_job_id<=0;calib<=0;src<=0;dst<=0;mx<=0;my<=0;ss<=0;ds<=0;ms<=0;
   width<=0;height<=0;x<=0;y<=0;params<=0;border<=0;issued_last<=0;numeric_error<=0;pixel_id<=0;
   frac_x<=0;frac_y<=0;pixel<=0;map_valid<=0;map_calib_id<=0;
   published_w<=0;published_h<=0;published_x<=0;published_y<=0;published_stride<=0;
  end else case(state)
   IDLE:if(cmd_valid)begin
    rsp_job_id<=cmd_job_id;rsp_status<=0;calib<=cmd_calib_id;
    src<=cmd_src_base;dst<=cmd_dst_base;mx<=cmd_map_x_base;my<=cmd_map_y_base;
    ss<=cmd_src_stride;ds<=cmd_dst_stride;ms<=cmd_map_stride;width<=cmd_width;height<=cmd_height;
    params<=cmd_params;border<=cmd_border_replicate;x<=0;y<=0;pixel_id<=0;issued_last<=0;numeric_error<=0;
    if(!check_valid||(cmd_opcode!=1&&cmd_opcode!=2))begin rsp_status<=1;state<=RESPONSE;end
    else if(cmd_opcode==1)begin
     if(!cmd_camera_valid||!finite_params)begin rsp_status<=4;state<=RESPONSE;end
     else begin map_valid<=0;state<=CFG_CORE;end
    end else if(!table_matches)begin rsp_status<=4;state<=RESPONSE;end
    else state<=READ_MAP;
   end
   CFG_CORE:if(core_cfg_ready)state<=CFG_WRITER;
   CFG_WRITER:if(mw_cfg_ready)state<=BUILD;
   BUILD:begin
    if(co_valid&&co_ready&&co_error)numeric_error<=1;
    if(ci_ready&&!issued_last)begin
     if(last_pixel)issued_last<=1;
     else begin pixel_id<=pixel_id+1;if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;end
    end
    if(mw_rsp_valid)begin
     if(numeric_error)rsp_status<=4;
     else if(mw_error)rsp_status<=5;
     else begin map_valid<=1;map_calib_id<=calib;published_w<=width;published_h<=height;published_x<=mx;published_y<=my;published_stride<=ms;end
     state<=RESPONSE;
    end
   end
   READ_MAP:if(mr_in_ready)state<=WAIT_MAP;
   WAIT_MAP:if(mr_out_valid)state<=SAMPLE;
   SAMPLE:begin
    frac_x<=dx;frac_y<=dy;
    if(mr_error)begin rsp_status<=5;state<=RESPONSE;end else state<=FETCH;
   end
   FETCH:if(nf_in_ready)state<=WAIT_FETCH;
   WAIT_FETCH:if(nf_out_valid)begin
    pixel<=interpolated;
    if(nf_error)begin rsp_status<=5;state<=RESPONSE;end else state<=WRITE_PIXEL;
   end
   WRITE_PIXEL:if(pw_in_ready)state<=WAIT_PIXEL;
   WAIT_PIXEL:if(pw_out_valid)begin
    if(pw_error)begin rsp_status<=5;state<=RESPONSE;end
    else if(last_pixel)state<=RESPONSE;
    else begin pixel_id<=pixel_id+1;if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;state<=READ_MAP;end
   end
   RESPONSE:if(rsp_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
