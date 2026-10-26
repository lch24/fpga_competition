`timescale 1ns/1ps
`include "calib_defs.vh"
// Real shared sequencer, workspace and FP arithmetic. Brown residuals are an
// independent real-number TB service so only changed LM logic consumes cycles.
// Fixtures: nonzero-k3 synthetic truth and historical UART folding corners.
module tb_lm_lens;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;
 wire cmd_ready,rsp_valid,converged;wire [7:0] status,accepted,outer;
 reg [15:0] width,height;reg [`PAR_STATE_W-1:0] input_state,expected;
 reg [64*`PAR_TOTAL_POINTS-1:0] points;
 wire [`PAR_STATE_W-1:0] result;wire [63:0] cost;
 reg [63:0] expected_cost;
 wire [127:0] execution_req;wire [95:0] execution_rsp;
 wire fv,fr,sv,sr;wire [4:0] fo,flags;wire [63:0] fa,fb,fd;
 calib_execution_service engine(.clk(clk),.rst_n(rst_n),.request(execution_req),.response(execution_rsp),
 .shared_req_valid(fv),.shared_req_ready(fr),.shared_req_op(fo),.shared_req_a(fa),.shared_req_b(fb),
 .shared_rsp_valid(sv),.shared_rsp_ready(sr),.shared_rsp_result(fd),.shared_rsp_flags(flags));
 fp_operator #(.FP_W(64)) alu(.clk(clk),.rst_n(rst_n),.req_valid(fv),.req_ready(fr),.req_op(fo),.req_a(fa),.req_b(fb),
 .rsp_valid(sv),.rsp_ready(sr),.rsp_result(fd),.rsp_flags(flags),.rsp_less(),.rsp_equal(),.rsp_unordered());
 wire ev,er,dr,rr;wire [`PAR_STATE_W-1:0] es;
 reg busy=0;integer count=0,calls=0,cycles=0,errors=0,cases=0;
 reg [63:0] residuals[0:`PAR_RESIDUALS-1];reg [63:0] residual_cost;
 reg done=0;
 wire dv=busy && count<`PAR_RESIDUALS;
 wire rv=busy && count==`PAR_RESIDUALS;
 assign er=!busy;
 function real f32;input [31:0] b;real m;integer e;begin
   e=b[30:23];m=e==0?b[22:0]/8388608.0:1.0+b[22:0]/8388608.0;
   f32=(b[31]?-1.0:1.0)*m*(2.0**(e==0?-126:e-127));
 end endfunction
 real s[0:`PAR_STATE_N-1],rot[0:8],skew[0:8];
 real angle,cs,sn,sum,fx,fy,cx,cy,tx,ty,tz,x,y,xc,yc,zc,r2,radial,u,v,du,dv_real,total;
 integer i,j,k,view,row,col,base;
 always @(posedge clk)begin
  if(!rst_n)begin busy<=0;count<=0;end
  else begin
   if(ev && er)begin
    for(i=0;i<`PAR_STATE_N;i=i+1)s[i]=$bitstoreal(es[64*i+:64]);
    fx=$exp(s[0]);fy=$exp(s[1]);cx=s[2]*width;cy=s[3]*height;total=0;
    for(view=0;view<`PAR_VIEWS;view=view+1)begin
     base=9+6*view;angle=$sqrt(s[base]*s[base]+s[base+1]*s[base+1]+s[base+2]*s[base+2]);
     sn=angle<1e-14?1.0:$sin(angle)/angle;
     cs=angle<1e-14?0.5:(1.0-$cos(angle))/(angle*angle);
     skew[0]=0;skew[1]=-s[base+2];skew[2]=s[base+1];
     skew[3]=s[base+2];skew[4]=0;skew[5]=-s[base];
     skew[6]=-s[base+1];skew[7]=s[base];skew[8]=0;
     for(row=0;row<3;row=row+1)for(col=0;col<3;col=col+1)begin
      sum=0;for(k=0;k<3;k=k+1)sum=sum+skew[3*row+k]*skew[3*k+col];
      rot[3*row+col]=(row==col?1.0:0.0)+sn*skew[3*row+col]+cs*sum;
     end
     tx=s[base+3];ty=s[base+4];tz=$exp(s[base+5]);
     for(j=0;j<`PAR_POINTS;j=j+1)begin
      x=(j%`PAR_BOARD_COLS)-(`PAR_BOARD_COLS-1)/2.0;y=(j/`PAR_BOARD_COLS)-(`PAR_BOARD_ROWS-1)/2.0;
      xc=rot[0]*x+rot[1]*y+tx;yc=rot[3]*x+rot[4]*y+ty;zc=rot[6]*x+rot[7]*y+tz;
      x=xc/zc;y=yc/zc;r2=x*x+y*y;
      radial=1.0+s[4]*r2+s[5]*r2*r2+s[8]*r2*r2*r2;
      u=fx*(x*radial+2.0*s[6]*x*y+s[7]*(r2+2.0*x*x))+cx;
      v=fy*(y*radial+s[6]*(r2+2.0*y*y)+2.0*s[7]*x*y)+cy;
      i=view*`PAR_POINTS+j;
      du=u-f32(points[64*i+:32]);dv_real=v-f32(points[64*i+32+:32]);
      residuals[2*i]=$realtobits(du);residuals[2*i+1]=$realtobits(dv_real);
      total=total+du*du+dv_real*dv_real;
     end
    end
    residual_cost<=$realtobits(total);busy<=1;count<=0;calls=calls+1;
   end
   if(dv && dr)count<=count+1;
   if(rv && rr)busy<=0;
  end
 end
 lm_controller #(.FP_SHARED(1),.SHARE_RESIDUAL(1),.ENGINE_SHARED(1)) dut(
 .clk(clk),.rst_n(rst_n),.execution_req(execution_req),.execution_rsp(execution_rsp),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_width(width),.cmd_height(height),.cmd_state(input_state),.cmd_stage(2'd2),
 .ext_cmd_valid(ev),.ext_cmd_ready(er),.ext_cmd_state(es),.ext_point_rd_en(1'b0),
 .ext_point_rd_view_id({`PAR_VIEW_BITS{1'b0}}),.ext_point_rd_index({`PAR_POINT_BITS{1'b0}}),
 .ext_data_valid(dv),.ext_data_ready(dr),.ext_data_index(count[`PAR_RES_BITS-1:0]),
 .ext_data_fp64(dv?residuals[count]:64'd0),.ext_data_last(count==`PAR_RESIDUALS-1),
 .ext_rsp_valid(rv),.ext_rsp_ready(rr),.ext_rsp_status(8'd0),.ext_rsp_cost_fp64(residual_cost),
 .point_rd_valid(1'b0),.point_rd_x_fp32(32'd0),.point_rd_y_fp32(32'd0),
 .shared_req_ready(9'd0),.shared_rsp_valid(9'd0),.shared_rsp_result(64'd0),.shared_rsp_flags(5'd0),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(status),.rsp_state(result),.rsp_cost_fp64(cost),
 .rsp_converged(converged),.rsp_accepted_steps(accepted),.rsp_outer_iterations(outer));
 integer file,n,index;real a,b,d;
 task check;input condition;input [511:0] message;begin
  if(condition!==1'b1)begin errors=errors+1;$display("FAIL lens case=%0d %0s",cases,message);end
 end endtask
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  file=$fopen("lm_lens_vectors.txt","r");if(!file)$fatal(1,"fixture open");
  while(!$feof(file))begin
   n=$fscanf(file,"%d %d %h %h %h %h\n",width,height,expected_cost,input_state,points,expected);
   if(n==6)begin
    @(negedge clk);check(cmd_ready,"ready between jobs");cmd_valid=1;calls=0;
    @(negedge clk);cmd_valid=0;cycles=0;
    while(!rsp_valid && cycles<300000000)begin @(negedge clk);cycles=cycles+1;end
    if(!rsp_valid)$fatal(1,"timeout pc=%d adapter=%d",engine.debug_pc,dut.state);
    check(status==0 && converged && accepted>0,"full-k3 fit converges");
    for(index=0;index<`PAR_STATE_N;index=index+1)begin
     a=$bitstoreal(result[64*index+:64]);b=$bitstoreal(expected[64*index+:64]);d=a-b;
     if(d<0)d=-d;
     check(result[64*index+52+:11]!=2047 && d<2e-5*(1+(b<0?-b:b)),"independent full-k3 state reference");
    end
    a=$bitstoreal(cost);b=$bitstoreal(expected_cost);d=a-b;if(d<0)d=-d;
    check(d<1e-7*(1+b),"reported cost is corner SSE, excludes lens prior");
    check(result[512+:64]!=0,"k3 remains fitted");
    repeat(5)begin @(negedge clk);check(rsp_valid && !cmd_ready,"output backpressure");end
    $display("LM_LENS case=%0d cycles=%0d evaluations=%0d accepted=%0d outer=%0d cost=%.14g k3=%.14g",cases,cycles,calls,accepted,outer,$bitstoreal(cost),$bitstoreal(result[512+:64]));
    rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;
   end else if(n!=-1)$fatal(1,"fixture format");
  end
  $fclose(file);done=1;$display("LM_LENS_DONE cases=%0d errors=%0d",cases,errors);
 end
endmodule
