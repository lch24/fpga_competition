// Generate repetitive named DDR connections. Behaviour remains in RTL modules.
const fs=require('fs'),path=require('path');
const root=path.resolve(__dirname,'..');
const bus=[['rd_valid',1,1],['rd_ready',1,0],['rd_addr',32,1],['rd_len',32,1],['rd_tag',16,1],
 ['r_valid',1,0],['r_ready',1,1],['r_data',32,0],['r_keep',4,0],['r_tag',16,0],['r_last',1,0],['r_error',1,0],
 ['wr_valid',1,1],['wr_ready',1,0],['wr_addr',32,1],['wr_len',32,1],['wr_tag',16,1],
 ['w_valid',1,1],['w_ready',1,0],['w_data',32,1],['w_keep',4,1],['w_last',1,1],
 ['b_valid',1,0],['b_ready',1,1],['b_tag',16,0],['b_error',1,0]];
const shared=new Set(['r_data','r_keep','r_tag','r_last','r_error','b_tag','b_error']);
const width=w=>w===1?'':`[${w-1}:0] `;
const portDecl=bus.map(([n,w,o])=>`${o?'output':'input'} wire ${width(w)}${n}`).join(',\n ');
const wireDecl=bus.map(([n,w])=>`wire ${width(shared.has(n)?w:3*w)}c_${n};`).join('\n ');
function client(i,rename={}){return bus.map(([n,w])=>`.${rename[n]||n}(c_${n}${shared.has(n)?'':w===1?`[${i}]`:`[${i*w}+:${w}]`})`).join(',\n  ');}
const service=bus.map(([n])=>`.c_${n}(c_${n}),.${n}(${n})`).join(',\n  ');
const detector={rd_valid:'m_rd_req_valid',rd_ready:'m_rd_req_ready',rd_addr:'m_rd_req_addr',rd_len:'m_rd_req_len_bytes',rd_tag:'m_rd_req_tag',
 r_valid:'m_rd_ret_valid',r_ready:'m_rd_ret_ready',r_data:'m_rd_ret_data',r_keep:'m_rd_ret_keep',r_tag:'m_rd_ret_tag',r_last:'m_rd_ret_last',r_error:'m_rd_ret_error',
 wr_valid:'m_wr_req_valid',wr_ready:'m_wr_req_ready',wr_addr:'m_wr_req_addr',wr_len:'m_wr_req_len_bytes',wr_tag:'m_wr_req_tag',
 w_valid:'m_wr_dat_valid',w_ready:'m_wr_dat_ready',w_data:'m_wr_dat_data',w_keep:'m_wr_dat_keep',w_last:'m_wr_dat_last',
 b_valid:'m_wr_cplt_valid',b_ready:'m_wr_cplt_ready',b_tag:'m_wr_cplt_tag',b_error:'m_wr_cplt_error'};
const top=`
\`timescale 1ns/1ps
\`include "calib_defs.vh"
// Generated connections: integration/tools/generate_wiring.js.
// DDR -> Gray8 -> detected corners -> real calibration -> maps -> RGB565 DDR.
// One job and one frame lease at a time. Last input frame is the correction input.
// Result memory remains owned by caller until rsp_ready; next start may overwrite.
module vision_ddr_top #(
 parameter WIDTH=1280,HEIGHT=720,DEPTH=2,
 parameter GRAY_BASE=32'h03000000,DST_BASE=32'h01000000,
 parameter MAP_X_BASE=32'h02000000,MAP_Y_BASE=32'h02800000
)(
 input wire clk,rst_n,start_valid,output wire start_ready,input wire [63:0] square_size_fp64,
 input wire frame_valid,output wire frame_ready,input wire [31:0] frame_base,frame_stride,frame_capacity,
 input wire [7:0] frame_status,output wire frame_release_valid,input wire frame_release_ready,
 output wire rsp_valid,input wire rsp_ready,output wire [7:0] rsp_status,
 output wire [31:0] result_base,result_stride,output wire [15:0] result_width,result_height,
 output wire busy,output wire [31:0] debug_job,output wire [7:0] debug_view,output wire [5:0] debug_phase,
 output wire [287:0] result_params,output reg [63:0] result_rms,
 ${portDecl}
);
 localparam [63:0] IMAGE_BYTES=64'd2*WIDTH*HEIGHT,GRAY_BYTES=64'd1*WIDTH*HEIGHT,MAP_BYTES=64'd4*WIDTH*HEIGHT;
 function disjoint;
  input [63:0] a,an,b,bn;begin disjoint=(a+an<=b)||(b+bn<=a);end
 endfunction
 wire layout_ok=WIDTH>=32&&HEIGHT>=32&&WIDTH<=1920&&HEIGHT<=1080&&DEPTH>=1&&DEPTH<=4&&
  (WIDTH>>(DEPTH-1))>=16&&(HEIGHT>>(DEPTH-1))>=16&&
  \`PAR_BOARD_ROWS>=2&&\`PAR_BOARD_COLS>=2&&\`PAR_POINTS<=64&&
  GRAY_BASE+GRAY_BYTES<=64'h40000000&&DST_BASE+IMAGE_BYTES<=64'h40000000&&
  MAP_X_BASE+MAP_BYTES<=64'h40000000&&MAP_Y_BASE+MAP_BYTES<=64'h40000000&&
  disjoint(GRAY_BASE,GRAY_BYTES,DST_BASE,IMAGE_BYTES)&&disjoint(GRAY_BASE,GRAY_BYTES,MAP_X_BASE,MAP_BYTES)&&
  disjoint(GRAY_BASE,GRAY_BYTES,MAP_Y_BASE,MAP_BYTES)&&disjoint(DST_BASE,IMAGE_BYTES,MAP_X_BASE,MAP_BYTES)&&
  disjoint(DST_BASE,IMAGE_BYTES,MAP_Y_BASE,MAP_BYTES)&&disjoint(MAP_X_BASE,MAP_BYTES,MAP_Y_BASE,MAP_BYTES);
 wire [63:0] frame_bytes=64'd1*(HEIGHT-1)*frame_stride+64'd2*WIDTH;
 wire frame_ok=frame_stride>=2*WIDTH&&frame_bytes<=frame_capacity&&{32'd0,frame_base}+frame_bytes<=64'h40000000&&
  disjoint(frame_base,frame_bytes,GRAY_BASE,GRAY_BYTES)&&disjoint(frame_base,frame_bytes,DST_BASE,IMAGE_BYTES)&&
  disjoint(frame_base,frame_bytes,MAP_X_BASE,MAP_BYTES)&&disjoint(frame_base,frame_bytes,MAP_Y_BASE,MAP_BYTES);
 assign result_base=DST_BASE;assign result_stride=2*WIDTH;assign result_width=WIDTH;assign result_height=HEIGHT;
 wire ar;wire [31:0] src,ss,capacity;wire [63:0] square;
 wire gv,gr,gdone,gack;wire [7:0] gs;
 wire process_frame,ddone,dv,dr,grid_ok;wire [1:0] dstatus;wire [15:0] total;wire [31:0] dx,dy;
 wire collect_v,collect_r,cv,cr,last,vr,va,ccv,ccr;wire [7:0] index,view_status;
 wire cbv,cbr,mv,mr;wire [7:0] ms;wire [31:0] mid;wire [15:0] mw,mh;wire [287:0] mp;
 wire pv,pr,usable,rv,rr;wire [31:0] pid,rid;wire [15:0] pw,ph;wire [287:0] pp;wire [7:0] rs;
 wire diag_v,diag_metrics;wire [63:0] diag_rms;
 wire uv,ur,udone,uack;wire [1:0] opcode;wire [7:0] us;
 ${wireDecl}
 vision_sequence #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) sequence_ctrl(
  .clk(clk),.rst_n(rst_n),.layout_ok(layout_ok),.start_valid(start_valid),.start_ready(start_ready),.square_size_fp64(square_size_fp64),
  .frame_valid(frame_valid),.frame_ready(frame_ready),.frame_base(frame_base),.frame_stride(frame_stride),.frame_capacity(frame_capacity),
  .frame_status(frame_status),.frame_config_ok(frame_ok),.frame_release_valid(frame_release_valid),.frame_release_ready(frame_release_ready),
  .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.busy(busy),.job(debug_job),.view_id(debug_view),.debug_phase(debug_phase),
  .src_base(src),.src_stride(ss),.src_capacity(capacity),.square(square),.algorithm_rst_n(ar),
  .gray_cmd_valid(gv),.gray_cmd_ready(gr),.gray_rsp_valid(gdone),.gray_rsp_ready(gack),.gray_rsp_status(gs),
  .det_process(process_frame),.det_done(ddone),.det_status(dstatus),.det_valid(dv),.det_ready(dr),.det_x(dx),.det_y(dy),.det_total(total),.det_grid_ok(grid_ok),
  .collect_valid(collect_v),.collect_ready(collect_r),.corner_valid(cv),.corner_ready(cr),.corner_index(index),.corner_last(last),
  .view_rsp_valid(vr),.view_rsp_ready(va),.view_status(view_status),.cal_cmd_valid(ccv),.cal_cmd_ready(ccr),
  .mb_begin_valid(cbv),.mb_begin_ready(cbr),.mb_valid(mv),.mb_ready(mr),.mb_status(ms),.mb_id(mid),.mb_width(mw),.mb_height(mh),.mb_params(mp),.params(result_params),
  .remap_cmd_valid(uv),.remap_cmd_ready(ur),.remap_opcode(opcode),.remap_rsp_valid(udone),.remap_rsp_ready(uack),.remap_status(us));
 rgb565_gray_dma gray(
  .clk(clk),.rst_n(rst_n),.cmd_valid(gv),.cmd_ready(gr),.width(16'(WIDTH)),.height(16'(HEIGHT)),
  .src_base(src),.src_stride(ss),.dst_base(GRAY_BASE),.dst_stride(32'(WIDTH)),.rsp_valid(gdone),.rsp_ready(gack),.rsp_status(gs),
  ${client(0)});
 corner_detect_ddr_top #(.W0(WIDTH),.H0(HEIGHT),.DEPTH(DEPTH),.GRAY_ADDR_W($clog2(2*WIDTH*HEIGHT)),.ROWS(\`PAR_BOARD_ROWS),.COLS(\`PAR_BOARD_COLS)) detector(
  .clk(clk),.rst_n(ar),.process_frame(process_frame),.busy(),.done(ddone),.status(dstatus),
  .cfg_gray_base(GRAY_BASE),.cfg_gray_stride(32'(WIDTH)),.cfg_gray_w(16'(WIDTH)),.cfg_gray_h(16'(HEIGHT)),.cfg_ram_base({$clog2(2*WIDTH*HEIGHT){1'b0}}),
  .cfg_resp_base(32'd0),.cfg_resp_dump_en(1'b0),.cfg_pyr_en(DEPTH>1),
  .out_valid(dv),.out_ready(dr),.out_x(dx),.out_y(dy),.out_total(total),.out_grid_ok(grid_ok),
  .ext_rd_req_valid(1'b0),.ext_rd_req_addr(32'd0),.ext_rd_req_len(32'd0),.ext_rd_req_tag(16'd0),.ext_rd_ret_ready(1'b1),
  .ext_wr_req_valid(1'b0),.ext_wr_req_addr(32'd0),.ext_wr_req_len(32'd0),.ext_wr_req_tag(16'd0),
  .ext_wr_dat_valid(1'b0),.ext_wr_dat_data(32'd0),.ext_wr_dat_keep(4'd0),.ext_wr_dat_last(1'b0),.ext_wr_done_ready(1'b1),
  ${client(1,detector)});
 calib_top calibration(
  .clk(clk),.rst_n(ar),.collect_valid(collect_v),.collect_ready(collect_r),.collect_job_id(debug_job),
  .corner_valid(cv),.corner_ready(cr),.corner_job_id(debug_job),.corner_view_id(debug_view),.corner_point_index(index),.corner_x_fp32(dx),.corner_y_fp32(dy),.corner_last(last),
  .view_rsp_valid(vr),.view_rsp_ready(va),.view_rsp_job_id(debug_job),.view_rsp_view_id(debug_view),.view_rsp_status(view_status),
  .cmd_valid(ccv),.cmd_ready(ccr),.cmd_job_id(debug_job),.cmd_width(16'(WIDTH)),.cmd_height(16'(HEIGHT)),.cmd_square_size_fp64(square),
  .camera_valid(pv),.camera_ready(pr),.camera_calib_id(pid),.camera_width(pw),.camera_height(ph),.camera_usable(usable),.camera_params(pp),
  .diag_valid(diag_v),.diag_ready(1'b1),.diag_metrics_valid(diag_metrics),.diag_rms_fp64(diag_rms),
  .rsp_valid(rv),.rsp_ready(rr),.rsp_status(rs),.rsp_job_id(rid));
 calibration_mailbox mailbox(
  .clk(clk),.rst_n(ar),.begin_valid(cbv),.begin_ready(cbr),.begin_job_id(debug_job),
  .param_valid(pv),.param_ready(pr),.param_calib_id(pid),.param_width(pw),.param_height(ph),.param_camera_valid(usable),.param_values(pp),
  .calib_rsp_valid(rv),.calib_rsp_ready(rr),.calib_rsp_id(rid),.calib_rsp_status(rs),
  .result_valid(mv),.result_ready(mr),.result_status(ms),.result_calib_id(mid),.result_width(mw),.result_height(mh),.result_values(mp));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)result_rms<=0;
  else if(start_valid&&start_ready)result_rms<=0;
  else if(diag_v&&diag_metrics)result_rms<=diag_rms;
 end
 undistort_top correction(
  .clk(clk),.rst_n(rst_n),.cmd_valid(uv),.cmd_ready(ur),.cmd_opcode(opcode),.cmd_job_id(debug_job),.cmd_calib_id(debug_job),
  .cmd_camera_valid(1'b1),.cmd_params(result_params),.cmd_width(16'(WIDTH)),.cmd_height(16'(HEIGHT)),
  .cmd_src_base(src),.cmd_dst_base(DST_BASE),.cmd_map_x_base(MAP_X_BASE),.cmd_map_y_base(MAP_Y_BASE),
  .cmd_src_stride(ss),.cmd_dst_stride(32'(2*WIDTH)),.cmd_map_stride(32'(4*WIDTH)),
  .cmd_src_capacity(capacity),.cmd_dst_capacity(32'(IMAGE_BYTES)),.cmd_map_capacity(32'(MAP_BYTES)),.cmd_border_replicate(1'b0),
  .rsp_valid(udone),.rsp_ready(uack),.rsp_status(us),
  ${client(2)});
 ddr_service #(.CLIENTS(3)) memory_service(.clk(clk),.rst_n(rst_n),
  ${service});
endmodule
`;
fs.mkdirSync(path.join(root,'rtl'),{recursive:true});
fs.writeFileSync(path.join(root,'rtl/vision_ddr_top.v'),top.trimStart());
// Reuse declarations when generating the camera wrapper separately.
module.exports={bus,width,portDecl};
