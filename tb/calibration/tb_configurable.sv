`timescale 1ns/1ps
`include "calib_defs.vh"
// Geometry regression: real corner RAM, init, residual, LM and validation cores.
// SKIP_LM supports capacity-boundary tests without an expensive dense LM solve.
module tb_configurable;
 parameter SKIP_LM=0;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 integer errors=0,cycles=0,reads=0,mode=0,i,v,p,seen,seeds=0,owner;
 reg done=0;
 reg [63:0] points[0:`PAR_TOTAL_POINTS-1],state_words[0:`PAR_STATE_N-1];
 reg [63:0] expected_r[0:`PAR_RESIDUALS-1],expected_m[0:1+13*`PAR_VIEWS],cost_words[0:0];
 reg [`PAR_STATE_W-1:0] truth,trial;
 reg clear=0,cv=0,cl=0,sv=0;wire cr,sr;
 reg [7:0] cv_id=0,cp=0,sv_id=0,sv_status=0;reg [31:0] cx=0,cy=0;
 wire [`PAR_VIEWS-1:0] vd,vf,vu;wire [8*`PAR_VIEWS-1:0] vs;
 wire [`PAR_POINT_BITS*`PAR_VIEWS-1:0] counts;
 wire rd,rv;wire [`PAR_VIEW_BITS-1:0] rid;wire [`PAR_POINT_BITS-1:0] rpi;wire [31:0] rx,ry;
 reg direct_en=0;reg [`PAR_VIEW_BITS-1:0] direct_view=0;reg [`PAR_POINT_BITS-1:0] direct_point=0;
 wire re,ie,le,ve;wire [`PAR_VIEW_BITS-1:0] ri,ii,li,vi;wire [`PAR_POINT_BITS-1:0] rp,ip,lp,vp;
 assign rd=mode==0?direct_en:mode==1?re:mode==2?ie:mode==3?le:ve;
 assign rid=mode==0?direct_view:mode==1?ri:mode==2?ii:mode==3?li:vi;
 assign rpi=mode==0?direct_point:mode==1?rp:mode==2?ip:mode==3?lp:vp;
 corner_store store(.clk(clk),.rst_n(rst_n),.clear(clear),.corner_valid(cv),.corner_ready(cr),.corner_view_id(cv_id),
 .corner_point_index(cp),.corner_x_fp32(cx),.corner_y_fp32(cy),.corner_last(cl),.view_rsp_valid(sv),.view_rsp_ready(sr),
 .view_rsp_view_id(sv_id),.view_rsp_status(sv_status),.view_done(vd),.view_status(vs),.view_point_count(counts),
 .view_format_error(vf),.view_usable(vu),.rd_en(rd),.rd_view_id(rid),.rd_point_index(rpi),.rd_valid(rv),.rd_x_fp32(rx),.rd_y_fp32(ry));
 reg rcmd=0;wire rready,rrsp,rdata,rlast;wire [7:0] rstatus;wire [`PAR_RES_BITS-1:0] rindex;wire [63:0] rvalue,rcost;
 wire data_ready=(cycles%7!=0);
 residual_engine residual_core(.clk(clk),.rst_n(rst_n),.cmd_valid(rcmd),.cmd_ready(rready),.cmd_width(16'd1280),.cmd_height(16'd720),.cmd_state(truth),
 .point_rd_en(re),.point_rd_view_id(ri),.point_rd_index(rp),.point_rd_valid(rv&&mode==1),.point_rd_x_fp32(rx),.point_rd_y_fp32(ry),
 .data_valid(rdata),.data_ready(data_ready),.data_index(rindex),.data_fp64(rvalue),.data_last(rlast),
 .rsp_valid(rrsp),.rsp_ready(1'b0),.rsp_status(rstatus),.rsp_cost_fp64(rcost));
 reg icmd=0;wire iready,irsp,seed_valid;wire [7:0] istatus;wire [2:0] seed_id,seed_count;wire [`PAR_STATE_W-1:0] seed_state;
 init_controller init_core(.clk(clk),.rst_n(rst_n),.cmd_valid(icmd),.cmd_ready(iready),.cmd_width(16'd1280),.cmd_height(16'd720),
 .point_rd_en(ie),.point_rd_view_id(ii),.point_rd_index(ip),.point_rd_valid(rv&&mode==2),.point_rd_x_fp32(rx),.point_rd_y_fp32(ry),
 .seed_valid(seed_valid),.seed_ready(data_ready),.seed_id(seed_id),.seed_state(seed_state),.rsp_valid(irsp),.rsp_ready(1'b0),.rsp_status(istatus),.rsp_seed_count(seed_count));
 reg lcmd=0;wire lready,lrsp,lconverged;wire [7:0] lstatus,laccepted,louter;wire [63:0] lcost;wire [`PAR_STATE_W-1:0] lstate;
 lm_controller lm_core(.clk(clk),.rst_n(rst_n),.cmd_valid(lcmd),.cmd_ready(lready),.cmd_width(16'd1280),.cmd_height(16'd720),.cmd_state(trial),.cmd_stage(2'd2),
 .point_rd_en(le),.point_rd_view_id(li),.point_rd_index(lp),.point_rd_valid(rv&&mode==3),.point_rd_x_fp32(rx),.point_rd_y_fp32(ry),
 .rsp_valid(lrsp),.rsp_ready(1'b0),.rsp_status(lstatus),.rsp_state(lstate),.rsp_cost_fp64(lcost),.rsp_converged(lconverged),.rsp_accepted_steps(laccepted),.rsp_outer_iterations(louter));
 reg vcmd=0;wire vready,vrsp,usable,metrics,weak_geometry;wire [7:0] vstatus;wire [287:0] camera;wire [63:0] rms,max_error;
 wire [`PAR_VIEW_RMS_W-1:0] view_rms;wire [`PAR_POSES_W-1:0] poses;
 wire [64*(2+13*`PAR_VIEWS)-1:0] actual_metrics={poses,max_error,view_rms,rms};
 validate_result check_core(.clk(clk),.rst_n(rst_n),.cmd_valid(vcmd),.cmd_ready(vready),.cmd_width(16'd1280),.cmd_height(16'd720),
 .cmd_state(truth),.cmd_square_size_fp64(64'h4004000000000000),.cmd_best_cost_fp64(cost_words[0]),.cmd_converged(1'b1),
 .point_rd_en(ve),.point_rd_view_id(vi),.point_rd_index(vp),.point_rd_valid(rv&&mode==4),.point_rd_x_fp32(rx),.point_rd_y_fp32(ry),
 .rsp_valid(vrsp),.rsp_ready(1'b0),.rsp_status(vstatus),.rsp_camera_usable(usable),.rsp_camera_params(camera),.rsp_metrics_valid(metrics),
 .rsp_weak_geometry(weak_geometry),.rsp_rms_fp64(rms),.rsp_view_rms_fp64(view_rms),.rsp_max_error_fp64(max_error),.rsp_poses_fp64(poses));
 task check(input bit ok,input string message);begin if(!ok)begin errors=errors+1;$display("FAIL %s cycle=%0d",message,cycles);$fdisplay(report,"FAIL %s cycle=%0d",message,cycles);end end endtask
 task near(input [63:0] a,b,input real tol,input string message);real x,y,d;begin
 x=$bitstoreal(a);y=$bitstoreal(b);d=x-y;if(d<0)d=-d;if(y<0)y=-y;
 check(a[62:52]!=2047 && !$isunknown(a) && d<=tol*(1+y),message);end endtask
 always @(posedge clk)if(rst_n&&!done)begin
 cycles=cycles+1;
 if(rd)begin reads=reads+1;check(rid<`PAR_VIEWS&&rpi<`PAR_POINTS,"read address range");end
 if(mode==1&&rdata&&data_ready)begin
 check(rindex==seen && rlast==(seen==`PAR_RESIDUALS-1),"residual order/last");
 near(rvalue,expected_r[seen],1e-9,"analytical residual");seen=seen+1;
 end
 if(mode==2&&seed_valid&&data_ready)begin
 check(seed_id==seeds,"all seeds ordered");
 if(seed_id==0)for(integer j=0;j<`PAR_STATE_N;j=j+1)near(seed_state[64*j+:64],truth[64*j+:64],2e-4,"Zhang seed versus known camera/pose");
 seeds=seeds+1;
 end
 end
 real oldcost,diff;integer start_reads,report;
 initial begin
 report=$fopen("config_results.txt","w");if(!report)$fatal(1,"report open");
 $readmemh("points.hex",points);$readmemh("state.hex",state_words);$readmemh("residual.hex",expected_r);$readmemh("metrics.hex",expected_m);$readmemh("cost.hex",cost_words);
 truth=0;for(i=0;i<`PAR_STATE_N;i=i+1)truth[64*i+:64]=state_words[i];
 trial=truth;trial[0+:64]=$realtobits($ln(808.0));
 // Only fx differs: independently compute trial cost from known ideal projection.
 oldcost=0;for(i=0;i<`PAR_TOTAL_POINTS;i=i+1)begin
 diff=$bitstoreal(expected_r[2*i])+0.01*($bitstoreal(expected_r[2*i])+$bitstoshortreal(points[i][31:0])-640.0);
 oldcost=oldcost+diff*diff+$bitstoreal(expected_r[2*i+1])*$bitstoreal(expected_r[2*i+1]);end
 repeat(5)@(negedge clk);rst_n=1;@(negedge clk);
 for(v=0;v<`PAR_VIEWS;v=v+1)begin
  for(p=0;p<`PAR_POINTS;p=p+1)begin
   cv_id=v;cp=p;cx=points[v*`PAR_POINTS+p][31:0];cy=points[v*`PAR_POINTS+p][63:32];cl=p==`PAR_POINTS-1;cv=1;
   do @(posedge clk);while(!cr);@(negedge clk);cv=0;
  end
  sv_id=v;sv=1;do @(posedge clk);while(!sr);@(negedge clk);sv=0;
 end
 check((&vu)&&vf==0&&(&vd),"all configured views committed");
 for(v=0;v<`PAR_VIEWS;v=v+1)check(counts[v*`PAR_POINT_BITS+:`PAR_POINT_BITS]==`PAR_POINTS,"per-view count including full capacity");
 for(v=0;v<`PAR_VIEWS;v=v+1)for(p=0;p<`PAR_POINTS;p=p+1)begin
 direct_view=v;direct_point=p;direct_en=1;@(negedge clk);direct_en=0;
 check(rv&&{ry,rx}==points[v*`PAR_POINTS+p],"RAM round trip");@(negedge clk);end
 mode=1;seen=0;start_reads=reads;rcmd=1;@(negedge clk);rcmd=0;wait(rrsp);@(negedge clk);
 check(rstatus==0&&seen==`PAR_RESIDUALS&&reads-start_reads==`PAR_TOTAL_POINTS,"complete residual service");near(rcost,cost_words[0],1e-10,"total cost");
 $display("CONFIG_RESIDUAL_PASS errors=%0d cycles=%0d",errors,cycles);
 $fdisplay(report,"residual errors=%0d cycles=%0d",errors,cycles);
 mode=2;start_reads=reads;icmd=1;@(negedge clk);icmd=0;wait(irsp);@(negedge clk);
 check(istatus==0&&seed_count==5&&seeds==5&&reads-start_reads==`PAR_TOTAL_POINTS,"all init seeds and views");
 $display("CONFIG_INIT_PASS errors=%0d cycles=%0d",errors,cycles);
 $fdisplay(report,"init errors=%0d seeds=%0d cycles=%0d",errors,seeds,cycles);
 if(!SKIP_LM)begin
 mode=3;lcmd=1;@(negedge clk);lcmd=0;wait(lrsp);@(negedge clk);
 check(lstatus==0&&laccepted>0&&louter>0&&louter<=`PAR_LM_MAX_ITERS,"LM updates within configured limit");
 check($bitstoreal(lcost)<oldcost,"LM reduced independently computed initial cost");
 check(lstate[8*64+:64]==0,"k3 stays fixed");
 for(i=0;i<`PAR_STATE_N;i=i+1)check(lstate[64*i+52+:11]!=2047&&!$isunknown(lstate[64*i+:64]),"finite LM state");
 $display("CONFIG_LM_PASS n=%0d accepted=%0d outer=%0d oldcost=%g newcost=%g errors=%0d cycles=%0d",`PAR_ACTIVE_N,laccepted,louter,oldcost,$bitstoreal(lcost),errors,cycles);
 $fdisplay(report,"lm n=%0d accepted=%0d outer=%0d oldcost=%g newcost=%g errors=%0d cycles=%0d",`PAR_ACTIVE_N,laccepted,louter,oldcost,$bitstoreal(lcost),errors,cycles);
 end
 mode=4;vcmd=1;@(negedge clk);vcmd=0;wait(vrsp);@(negedge clk);
 check(vstatus==0&&usable&&metrics,"valid known physical camera");check(weak_geometry==(`PAR_VIEWS<5),"weak geometry over all view pairs");
 for(i=0;i<2+13*`PAR_VIEWS;i=i+1)near(actual_metrics[64*i+:64],expected_m[i],1e-9,"all view RMS and physical R/t");
 check(camera[0+:32]==32'h44480000&&camera[32+:32]==32'h444d0000,"known fx/fy");
 $display("CONFIG_VALIDATION_PASS errors=%0d cycles=%0d",errors,cycles);
 // Last configured view failure must be represented in the status/debug buses.
 mode=0;clear=1;@(negedge clk);clear=0;
 for(v=0;v<`PAR_VIEWS;v=v+1)begin sv_id=v;sv_status=2;sv=1;@(negedge clk);sv=0;end
 check((&vd)&&vu==0&&vf==0&&vs[8*(`PAR_VIEWS-1)+:8]==2,"all view failure statuses preserved after clear");
 done=1;$display("CONFIG_RESULT views=%0d rows=%0d cols=%0d errors=%0d cycles=%0d",`PAR_VIEWS,`PAR_BOARD_ROWS,`PAR_BOARD_COLS,errors,cycles);
 $fdisplay(report,"RESULT views=%0d rows=%0d cols=%0d skip_lm=%0d errors=%0d cycles=%0d",`PAR_VIEWS,`PAR_BOARD_ROWS,`PAR_BOARD_COLS,SKIP_LM,errors,cycles);$fclose(report);
 end
endmodule
