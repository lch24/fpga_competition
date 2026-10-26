`timescale 1ns/1ps
`include "calib_defs.vh"
// Real UI + sequence + asynchronous bridge + corner FIFO + marker DMA.
// Numerical workers and HMIC memory are protocol models. REAL_COPY=1 uses
// the actual remapper, writer, native arbiter and board write-customer mux.
// Crucially: only the REAL sequence can release a frame; no forced FSM states.
module tb_corner_review_integration;
 parameter CORE_HALF=4,ALG_HALF=12.5,ALG_OFFSET=3,REAL_COPY=0;
 localparam W=128,H=48,DST=32'h01000000;
 reg clk=0,aclk=0,rst=0;
 always #(CORE_HALF) clk=~clk;
 initial begin #(ALG_OFFSET);forever #(ALG_HALF) aclk=~aclk;end
 reg [4:0] keys=0;reg display_idle=1;
 wire sv,sready,sremap,fv,fr,rel,relready,rv,rr;
 wire [31:0] sb,fb;wire [7:0] fs,rs;
 wire asv,asready,asremap,afv,afr,arel,arelready,arv,arr;
 wire [31:0] asb,afb;wire [7:0] afs,ars;
 wire capv,caprr;wire [31:0] capbase;reg caprsp=0;reg [31:0] capret=0;
 wire dispen,ready,cont,video,calibrated,mapready;wire [31:0] dispbase;
 wire [2:0] captured;wire [1:0] selected;wire [3:0] phase;wire [7:0] status;
 wire copyv,copyrr,bypass;wire [31:0] copysrc,copydst;wire copyready,copyrsp;wire [7:0] copystatus;
 wire markv,markready,markrsp,markrr,markbusy;wire [7:0] markstatus;
 wire awv,awready,wready,wdone;wire [27:0] awaddr;wire [3:0] awlen;
 wire [255:0] wdata;wire [31:0] wstrb;
 wire awaitreview;reg [1:0] waits=0,continues=0;
 wire complete,pv,pr;wire [71:0] pd;
 integer cycle=0;reg point_drain=1;
 always @(posedge clk)begin
  if(!rst)begin waits<=0;cycle<=0;end
  else begin waits<={waits[0],awaitreview};cycle<=cycle+1;end
 end
 always @(posedge aclk)if(!rst)continues<=0;else continues<={continues[0],cont};
 video_board_flow #(.ENABLE_CORNER_REVIEW(1),.REVIEW_CYCLES(8)) flow(
  .clk(clk),.rst_n(rst),.preview_enable(1'b0),.camera_ready(1'b1),.pressed(keys),
  .display_idle(display_idle),.display_frame_start(dispen),
  .start_valid(sv),.start_ready(sready),.start_remap(sremap),.start_base(sb),
  .frame_valid(fv),.frame_ready(fr),.frame_base(fb),.frame_status(fs),
  .frame_release_valid(rel),.frame_release_ready(relready),
  .cap_cmd_valid(capv),.cap_cmd_ready(1'b1),.cap_base(capbase),.cap_rsp_valid(caprsp),
  .cap_rsp_ready(caprr),.cap_status(8'd0),.cap_rsp_base(capret),
  .rsp_valid(rv),.rsp_ready(rr),.rsp_status(rs),.display_enable(dispen),.display_base(dispbase),
  .captured(captured),.selected(selected),.status(status),.phase(phase),.calibrated(calibrated),
  .video_active(video),.map_ready(mapready),.remap_valid(copyv),.remap_ready(copyready),
  .remap_src(copysrc),.remap_dst(copydst),.remap_bypass(bypass),
  .remap_rsp_valid(copyrsp),.remap_rsp_ready(copyrr),.remap_status(copystatus),
  .review_wait(waits[1]&&complete),.review_continue(cont),.review_ready(ready),
  .mark_valid(markv),.mark_ready(markready),.mark_rsp_valid(markrsp),.mark_rsp_ready(markrr),.mark_status(markstatus));
 vision_clock_bridge bridge(.core_clk(clk),.core_rst_n(rst),.algorithm_clk(aclk),.algorithm_rst_n(rst),
  .start_valid(sv),.start_ready(sready),.start_remap(sremap),.start_base(sb),
  .a_start_valid(asv),.a_start_ready(asready),.a_start_remap(asremap),.a_start_base(asb),
  .frame_valid(fv),.frame_ready(fr),.frame_base(fb),.frame_status(fs),
  .a_frame_valid(afv),.a_frame_ready(afr),.a_frame_base(afb),.a_frame_status(afs),
  .release_valid(rel),.release_ready(relready),.a_release_valid(arel),.a_release_ready(arelready),
  .rsp_valid(rv),.rsp_ready(rr),.rsp_status(rs),.a_rsp_valid(arv),.a_rsp_ready(arr),.a_rsp_status(ars),
  .a_rd_valid(1'b0),.rd_ready(1'b1),.a_rd_addr(32'd0),.a_rd_len(32'd0),.a_rd_tag(16'd0),
  .r_valid(1'b0),.a_r_ready(1'b1),.r_data(32'd0),.r_keep(4'd0),.r_tag(16'd0),.r_last(1'b0),.r_error(1'b0),
  .a_wr_valid(1'b0),.wr_ready(1'b1),.a_wr_addr(32'd0),.a_wr_len(32'd0),.a_wr_tag(16'd0),
  .a_w_valid(1'b0),.w_ready(1'b1),.a_w_data(32'd0),.a_w_keep(4'd0),.a_w_last(1'b0),
  .b_valid(1'b0),.a_b_ready(1'b1),.b_tag(16'd0),.b_error(1'b0));
 wire grayv,grayrr,detstart,detready,cv,viewv,calv,mbready,worker,collectv;
 wire [7:0] ci,viewid,viewstatus;wire [31:0] job;wire [5:0] aphase;
 reg grayrsp=0,detdone=0,detvalid=0,gridok=1,mbvalid=0,maprsp=0;
 reg [31:0] dx=0,dy=0;reg [7:0] mbstatus=0;wire mapv,maprr;wire [1:0] mapop;
 integer failview=-1,solver_status=0;
 vision_sequence #(.WIDTH(W),.HEIGHT(H),.MANUAL_CONTROL(1),.BUILD_ONLY(1),.ENABLE_CORNER_REVIEW(1)) seq(
  .clk(aclk),.rst_n(rst),.layout_ok(1'b1),.start_valid(asv),.start_ready(asready),
  .start_remap(asremap),.remap_base(asb),.square_size_fp64(64'h3ff0000000000000),
  .frame_valid(afv),.frame_ready(afr),.frame_base(afb),.frame_status(afs),.frame_stride(32'd256),.frame_capacity(32'd12288),.frame_config_ok(1'b1),
  .frame_release_valid(arel),.frame_release_ready(arelready),.rsp_valid(arv),.rsp_ready(arr),.rsp_status(ars),
  .job(job),.view_id(viewid),.debug_phase(aphase),.algorithm_rst_n(worker),
  .gray_cmd_valid(grayv),.gray_cmd_ready(1'b1),.gray_rsp_valid(grayrsp),.gray_rsp_ready(grayrr),.gray_rsp_status(8'd0),
  .det_process(detstart),.det_done(detdone),.det_status(2'b01),.det_valid(detvalid),.det_ready(detready),
  .det_x(dx),.det_y(dy),.det_total(16'd40),.det_grid_ok(gridok),
  .collect_valid(collectv),.collect_ready(1'b1),.corner_valid(cv),.corner_ready(pr),.corner_index(ci),
  .view_rsp_valid(viewv),.view_rsp_ready(1'b1),.view_status(viewstatus),
  .cal_cmd_valid(calv),.cal_cmd_ready(1'b1),.review_wait(awaitreview),.review_continue(continues[1]),
  .mb_begin_ready(1'b1),.mb_valid(mbvalid),.mb_ready(mbready),.mb_status(mbstatus),
  .mb_id(job),.mb_width(16'd128),.mb_height(16'd48),.mb_params(288'h12345678),
  .remap_cmd_valid(mapv),.remap_cmd_ready(1'b1),.remap_opcode(mapop),
  .remap_rsp_valid(maprsp),.remap_rsp_ready(maprr),.remap_status(8'd0));
 // Same atomic mirror as vision_ddr_top, with the store always ready.
 async_fifo #(.WIDTH(72),.ADDR_BITS(3)) points(.wclk(aclk),.wrst_n(rst),.wvalid(cv),.wready(pr),
  .wdata({viewid[1:0],ci[5:0],dy,dx}),.rclk(clk),.rrst_n(rst),.rvalid(pv),.rready(point_drain),.rdata(pd));
 corner_review_dma #(.WIDTH(W),.HEIGHT(H)) marks(.clk(clk),.rst_n(rst),.point_valid(pv&&point_drain),.point(pd),.points_complete(complete),
  .cmd_valid(markv),.cmd_ready(markready),.view(selected),.dst_base(DST),
  .rsp_valid(markrsp),.rsp_ready(markrr),.status(markstatus),.busy(markbusy),
  .awvalid(awv),.awready(awready),.awaddr(awaddr),.awlen(awlen),.wdata(wdata),.wstrb(wstrb),.wready(wready),.wdone(wdone),.wid(4'd0));
 integer detections=0,solves=0,releases=0,starts=0,frames=0,point_count=0,maps=0,collections=0;
 reg allow_solve=0;
 always @(posedge aclk)if(rst)begin
  if(detstart)detections<=detections+1;
  if(collectv)collections<=collections+1;
  if(calv)begin if(!allow_solve)$fatal(1,"solver ran before explicit confirmation");solves<=solves+1;end
  if(cv&&pr)point_count<=point_count+1;
  if(afv&&afr)begin
   if(afb!=viewid*32'h200000)$fatal(1,"wrong immutable frame descriptor");
   frames<=frames+1;
  end
  if(mapv)maps<=maps+1;
  if(awaitreview&&(!worker||calv||detstart||grayv))$fatal(1,"review lost worker/started computation");
 end
 always @(posedge clk)if(rst)begin
  if(sv&&sready)starts<=starts+1;
  if(rel&&relready)releases<=releases+1;
  if(markbusy&&copyv)$fatal(1,"copier and marker overlap");
  if((copyv||markbusy)&&!display_idle)$fatal(1,"overwriting active display");
 end
 function [31:0] fp(input integer n);
  integer e;reg [31:0] m;
  begin e=0;m=n;while(m>1)begin m=m>>1;e=e+1;end
   fp=n==0?0:((127+e)<<23)|((n<<(23-e))&32'h7fffff);
  end
 endfunction
 task ct;@(negedge clk);endtask
 task at;@(negedge aclk);endtask
 task press(input integer k);begin ct();keys=1<<k;ct();keys=0;end endtask
 // Workers hold valid until accepted and do not manufacture release tokens.
 integer i;
 initial forever begin
  wait(rst&&grayv);at();repeat(3)at();grayrsp=1;wait(grayrr);at();grayrsp=0;
  wait(detstart);at();detdone=0;gridok=1;repeat(3)at();
  for(i=0;i<40;i=i+1)begin
   dx=fp(8+(i%8)*7+viewid*3);dy=fp(8+(i/8)*7+viewid);
   detvalid=1;do @(posedge aclk);while(!detready);at();detvalid=0;
   if(i%7==0)repeat(2)at();
  end
  gridok=viewid!=failview;detdone=1;wait(viewv);at();
 end
 initial forever begin
  wait(rst&&((viewv&&viewstatus!=0)||calv));
  mbstatus=(viewv&&viewstatus!=0)?viewstatus:solver_status;
  at();repeat(15)at();mbvalid=1;wait(mbready);at();mbvalid=0;
 end
 initial forever begin
  wait(rst&&mapv);if(mapop!=1)$fatal(1,"not a build-only command");at();repeat(7)at();
  maprsp=1;wait(maprr);at();maprsp=0;
 end
 // Mutable output only; originals represented by per-view immutable patterns.
 reg [7:0] output_bytes[0:W*H*2-1];integer copy_view,p,b;reg copy_busy=0;
 function [15:0] original(input integer v,pixel_index);original=16'h1234^(v*16'h100)^(pixel_index*73);endfunction
 generate if(!REAL_COPY)begin : g_model_copy
 reg cq=0,crsp=0,aq=0,wq=0,wd=0;
 assign copyready=cq;assign copyrsp=crsp;assign copystatus=0;
 assign awready=aq;assign wready=wq;assign wdone=wd;
 initial forever begin
  wait(rst&&copyv);copy_view=copysrc/32'h200000;
  if(copy_view>2||copydst!=DST||!bypass||dispen)$fatal(1,"review copier descriptor");
  ct();repeat(5)ct();cq=1;ct();cq=0;copy_busy=1;
  for(p=0;p<W*H;p=p+1)begin output_bytes[2*p]=original(copy_view,p)&255;output_bytes[2*p+1]=original(copy_view,p)>>8;end
  repeat(13)ct();copy_busy=0;crsp=1;wait(copyrr);ct();crsp=0;
 end
 reg [31:0] byte_addr;
 initial forever begin
  wait(rst&&awv);if(copy_busy||dispen||awlen!=0)$fatal(1,"marker ownership");
  ct();repeat(3)ct();byte_addr={awaddr,2'b0};
  if(byte_addr<DST||byte_addr>=DST+W*H*2)$fatal(1,"marker wrote outside output frame");
  aq=1;ct();aq=0;repeat(2)ct();wq=1;
  @(posedge clk);
  for(b=0;b<32;b=b+1)if(wstrb[b])output_bytes[byte_addr-DST+b]=wdata[b*8+:8];
  ct();wq=0;repeat(4)ct();wd=1;ct();wd=0;
 end
 end else begin : g_real_copy
  // Same engine/marker mux and native customer 2 as the board top.
  wire [1:0] eav,ear,erv;wire [55:0] eaa;wire [7:0] eal;
  wire ewv;wire [27:0] ewa;wire [3:0] ewl;wire [255:0] ewd;
  wire [3:0] car,crv;wire [2:0] caw,cwr,cwd;
  wire nav,naw;wire [27:0] naa,nwa;wire [3:0] nal,nwl;
  wire [255:0] nwd;wire [31:0] nws;
  reg rb=0,wb=0;reg [31:0] read_byte=0,write_byte=0;
  reg [4:0] rbeat=0,wbeat=0,rsize=0,wsize=0;
  integer completion_delay=0,lane,l;
  wire nar=!rb&&cycle%7!=0,nawready=!wb&&completion_delay==0&&cycle%5!=0;
  wire nrv=rb&&cycle%4!=0,nrl=rbeat==rsize-1,nwr=wb&&cycle%3!=0;
  wire nwdone=completion_delay==1;
  reg [255:0] nrd;
  always @*for(l=0;l<16;l=l+1)
   nrd[l*16+:16]=original(read_byte/32'h200000,(read_byte%32'h200000)/2+rbeat*16+l);
  realtime_remap #(.WIDTH(W),.HEIGHT(H),.CACHE_INDEX_BITS(3)) engine(
   .clk(clk),.rst_n(rst),.cmd_valid(copyv),.cmd_ready(copyready),.src_base(copysrc),.dst_base(copydst),.map_base(32'h2000000),.cmd_bypass(bypass),
   .rsp_valid(copyrsp),.rsp_ready(copyrr),.rsp_status(copystatus),
   .arvalid(eav),.arready(car[3:2]),.araddr(eaa),.arlen(eal),.rvalid(crv[3:2]),.rdata(nrd),.rlast(nrl),.rid(4'd0),
   .awvalid(ewv),.awready(caw[2]&&!markbusy),.awaddr(ewa),.awlen(ewl),.wdata(ewd),
   .wready(cwr[2]&&!markbusy),.wdone(cwd[2]&&!markbusy),.wid(4'd0));
  assign awready=caw[2]&&markbusy;assign wready=cwr[2]&&markbusy;assign wdone=cwd[2]&&markbusy;
  video_hmic_arbiter arb(.clk(clk),.rst_n(rst),
   .c_arvalid({eav,2'b0}),.c_arready(car),.c_araddr({eaa,56'b0}),.c_arlen({eal,8'b0}),.c_rvalid(crv),
   .c_awvalid({markbusy?awv:ewv,2'b0}),.c_awready(caw),.c_awaddr({markbusy?awaddr:ewa,56'b0}),.c_awlen({markbusy?awlen:ewl,8'b0}),
   .c_wdata({markbusy?wdata:ewd,512'b0}),.c_wstrb({markbusy?wstrb:32'hffffffff,64'b0}),.c_wready(cwr),.c_wdone(cwd),
   .araddr(naa),.arlen(nal),.arvalid(nav),.arready(nar),.rdata(nrd),.rvalid(nrv),.rlast(nrl),.rid(4'd0),
   .awaddr(nwa),.awlen(nwl),.awvalid(naw),.awready(nawready),.wdata(nwd),.wstrb(nws),.wready(nwr),.wdone(nwdone),.wid(4'd0));
  always @(posedge clk)if(rst)begin
   if(copyv&&copyready)begin
    if(!bypass||copysrc/32'h200000>2||copydst!=DST||dispen)$fatal(1,"real copier descriptor");
    copy_busy<=1;
   end
   if(copyrsp&&copyrr)begin
    if(copystatus!=0||wb||completion_delay!=0)$fatal(1,"copy error/early physical completion");copy_busy<=0;
   end
   if(eav[0])$fatal(1,"identity copier read the distortion map");
   if(markbusy&&copy_busy)$fatal(1,"write owner changed before copier finished");
   if(nav&&nar)begin
    if({naa,2'b0}/32'h200000>2)$fatal(1,"read outside immutable photos");
    rb<=1;read_byte<={naa,2'b0};rbeat<=0;rsize<={1'b0,nal}+1'b1;
   end
   if(nrv)begin rbeat<=rbeat+1'b1;if(nrl)rb<=0;end
   if(naw&&nawready)begin
    if({nwa,2'b0}<DST||{nwa,2'b0}+32*(nwl+1)>DST+W*H*2)$fatal(1,"native write changed original/outside output");
    wb<=1;write_byte<={nwa,2'b0};wbeat<=0;wsize<={1'b0,nwl}+1'b1;
   end
   if(nwr)begin
    for(lane=0;lane<32;lane=lane+1)if(nws[lane])output_bytes[write_byte-DST+wbeat*32+lane]<=nwd[lane*8+:8];
    wbeat<=wbeat+1'b1;if(wbeat==wsize-1)begin wb<=0;completion_delay<=7;end
   end
   if(completion_delay>0)completion_delay<=completion_delay-1;
   if(dispen&&(wb||completion_delay!=0))$fatal(1,"display published before writes completed");
  end
 end endgenerate
 task photo(input integer v);
  begin press(v);wait(capv);ct();capret=capbase;repeat(4)ct();caprsp=1;wait(caprr);ct();caprsp=0;wait(phase==0);end
 endtask
 task check_picture(input integer v);
  integer x,y,n,cx,cy;reg red;reg [15:0] expected;
  begin wait(ready);ct();
   if(selected!=v||!dispen||dispbase!=DST||calibrated||!awaitreview)$fatal(1,"wrong review state/view");
   for(y=0;y<H;y=y+1)for(x=0;x<W;x=x+1)begin
    red=0;
    for(n=0;n<40;n=n+1)begin cx=8+(n%8)*7+v*3;cy=8+(n/8)*7+v;if(x>=cx-2&&x<=cx+2&&y>=cy-2&&y<=cy+2)red=1;end
    expected=red?16'hf800:original(v,y*W+x);
    if({output_bytes[2*(y*W+x)+1],output_bytes[2*(y*W+x)]}!==expected)$fatal(1,"wrong photo/marker pixel view=%d x=%d y=%d",v,x,y);
   end
  end
 endtask
 task select_picture(input integer v);
  begin display_idle=0;press(v);repeat(15)ct();if(copyv||markv||dispen)$fatal(1,"display drain ignored");
   display_idle=1;check_picture(v);
  end
 endtask
 task confirm;
  begin display_idle=0;press(3);repeat(20)ct();if(cont||calv||dispen)$fatal(1,"confirmation before display drained");
   allow_solve=1;display_idle=1;wait(cont);wait(phase==0);repeat(8)ct();allow_solve=0;
  end
 endtask
 integer timeout_cycles=0;
 always @(posedge clk)if(rst)begin
  timeout_cycles<=timeout_cycles+1;
  if(timeout_cycles>(REAL_COPY?1000000:250000))$fatal(1,"integration timeout UI=%d algorithm=%d corners=%d releases=%d",phase,aphase,point_count,releases);
 end
 initial begin
  repeat(5)ct();rst=1;
  photo(0);photo(1);photo(2);if(captured!=7)$fatal(1,"photos not captured");
  // Pause FIFO to force corner backpressure, then drain it normally.
  point_drain=0;press(3);wait(detections==1);repeat(120)at();
  if(point_count>8||calv)$fatal(1,"corner FIFO did not backpressure detector");point_drain=1;
  wait(awaitreview);repeat(150)ct();
  if(releases!=2||point_count!=120||solves!=0)$fatal(1,"last lease must remain held during review");
  // On the faulty RTL this waits forever: UI stays LEASE while seq waits KEY3.
  check_picture(0);repeat(200)ct();if(!ready||cont||solves!=0)$fatal(1,"auto-resume or preview timeout");
  press(4);repeat(10)ct();if(!ready||maps!=0)$fatal(1,"KEY4 bypassed review");
  select_picture(1);select_picture(2);select_picture(0);
  confirm();if(!calibrated||status!=0||releases!=3||detections!=3||solves!=1||frames!=3||starts!=1)$fatal(1,"resume/release/result or repeated detection");
  // Repeated task: stale corners must not enter review on an early failure.
  failview=1;press(3);wait(phase!=0);wait(phase==0);repeat(8)ct();
  if(status!=2||calibrated||ready||releases!=5||detections!=5||solves!=1)$fatal(1,"detection error did not return/release");
  failview=-1;solver_status=4;press(3);wait(phase!=0);check_picture(0);confirm();
  if(status!=4||calibrated||releases!=8||solves!=2)$fatal(1,"solver error did not return/release");
  solver_status=0;press(3);wait(phase!=0);check_picture(0);select_picture(2);confirm();
  if(!calibrated||status!=0||releases!=11||solves!=3||detections!=11||frames!=11||collections!=4)$fatal(1,"repeat job retained stale state");
  press(4);wait(video);if(!mapready||maps!=1)$fatal(1,"post-review map/video handoff");
  $display("PASS integration clocks=%0f/%0f real_copy=%0d: real leases/CDC/FIFO, 120 points, 3 marked photos, second KEY3, errors/retry, map/video",CORE_HALF,ALG_HALF,REAL_COPY);$finish;
 end
endmodule
