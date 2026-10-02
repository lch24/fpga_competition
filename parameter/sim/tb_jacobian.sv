`timescale 1ns/1ps
// 非线性解析残差服务验证差分、活动列映射、scale及背压；真实残差核另由LM联调。
module tb_jacobian;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;wire [7:0] rsp_status;
 reg [1:0] stage=0;reg [1727:0] original,cmd_state;
 wire ev_cmd,ev_ready,ev_data_ready,ev_rsp_ready;wire [1727:0] ev_state;
 wire j_valid,j_last;reg j_ready=0;wire [7:0] j_row;wire [4:0] j_col;wire [63:0] j_value;wire [1663:0] scales;
 reg [1727:0] service_state;integer svc=0,svc_index=0,requests=0,tick=0,mode=0;
 wire ev_data=(svc==1),ev_rsp=(svc==2);
 wire [7:0] ev_index=(mode==2 && svc_index==3)?7:svc_index;
 wire [63:0] ev_value=$realtobits(sample(service_state,svc_index));
 wire [7:0] ev_status=(mode==1)?4:(mode==5)?5:0;
 wire [63:0] ev_cost=(mode==4)?64'h7ff0000000000000:64'h3ff0000000000000;
 assign ev_ready=rst_n && svc==0 && tick%5!=0;
 reg done=0;integer errors=0,cases=0,report,n,k,t,i,expected_count,cycles;
 real expected_j[0:6239],expected_scale[0:25];real step,x,norm,d;
 reg [1727:0] qplus,qminus;reg [1663:0] snapshot;
 reg [77:0] held;reg stalled=0;integer received=0;
 reg eval_stalled=0;reg [1727:0] eval_held;
 jacobian dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_state(cmd_state),.cmd_stage(stage),
 .eval_cmd_valid(ev_cmd),.eval_cmd_ready(ev_ready),.eval_cmd_state(ev_state),.eval_data_valid(ev_data),.eval_data_ready(ev_data_ready),
 .eval_data_index(ev_index),.eval_data_fp64(ev_value),.eval_data_last(svc_index==239),
 .eval_rsp_valid(ev_rsp),.eval_rsp_ready(ev_rsp_ready),.eval_rsp_status(ev_status),.eval_rsp_cost_fp64(ev_cost),
 .j_valid(j_valid),.j_ready(j_ready),.j_row(j_row),.j_col(j_col),.j_fp64(j_value),.j_last(j_last),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_scales_fp64(scales));
 function integer index_of;input integer col;begin index_of=col<4?col:col<22?col+5:col-18;end endfunction
 function real sample;input [1727:0] state_bits;input integer row;real p,z;integer a,b;begin
 a=row%27;b=(row*7+3)%27;p=$bitstoreal(state_bits[64*a+:64]);z=$bitstoreal(state_bits[64*b+:64]);
 sample=(mode==6)?3.0:((row%9)-4)*0.2*p+0.03*p*p+0.11*z;end endfunction
 task check;input condition;input [511:0] label;begin if(condition!==1'b1)begin errors=errors+1;if(errors<40)$fdisplay(report,"FAIL case=%0d %0s",cases,label);end end endtask
 task near;input [63:0] bits;input real expected;real actual,diff;begin actual=$bitstoreal(bits);diff=actual-expected;if(diff<0)diff=-diff;
 check((^bits)!==1'bx && bits[62:52]!=2047 && diff<=2e-9*(1+(expected<0?-expected:expected)),"numeric result");end endtask
 always @(posedge clk)begin
  if(!rst_n)begin svc<=0;svc_index<=0;requests<=0;stalled<=0;eval_stalled<=0;end
  else begin
   if(eval_stalled)check(ev_cmd && ev_state===eval_held,"perturbed command stable under stall");
   eval_stalled<=ev_cmd && !ev_ready;eval_held<=ev_state;
   if(ev_cmd && ev_ready)begin service_state<=ev_state;svc<=1;svc_index<=0;requests<=requests+1;end
   if(ev_data && ev_data_ready)begin
    if(svc_index==239 || ((mode==1 || mode==3 || mode==5) && svc_index==6))svc<=2;
    else svc_index<=svc_index+1;
   end
   if(ev_rsp && ev_rsp_ready)svc<=0;
   if(stalled)check(j_valid && {j_row,j_col,j_value,j_last}===held,"J output stall stability");
   stalled<=j_valid && !j_ready;held<={j_row,j_col,j_value,j_last};
   if(j_valid && j_ready)begin
    check(j_col==received/240 && j_row==received%240 && j_last==(received==n*240-1),"J order/last");
    near(j_value,expected_j[received]);received=received+1;
   end
  end
 end
 always @(negedge clk)begin tick=tick+1;j_ready=rst_n && tick%7!=0 && tick%7!=1;end
 task prepare;begin
 n=stage==0?22:stage==1?23:26;received=0;
 for(k=0;k<n;k=k+1)begin
  x=$bitstoreal(original[64*index_of(k)+:64]);step=1e-6*(1+(x<0?-x:x));
  qplus=original;qplus[64*index_of(k)+:64]=$realtobits(x+step);
  qminus=qplus;qminus[64*index_of(k)+:64]=$realtobits((x+step)-2*step);norm=0;
  for(t=0;t<240;t=t+1)begin d=(sample(qplus,t)-sample(qminus,t))/(2*step);expected_j[k*240+t]=d;norm=norm+d*d;end
  norm=$sqrt(norm);expected_scale[k]=1/(norm<1e-12?1e-12:norm);
  for(t=0;t<240;t=t+1)expected_j[k*240+t]=expected_j[k*240+t]*expected_scale[k];
 end end endtask
 task launch;begin @(negedge clk);check(cmd_ready,"ready");cmd_state=original;cmd_valid=1;@(negedge clk);cmd_valid=0;cmd_state=~original;end endtask
 task await_response;begin cycles=0;while(!rsp_valid && cycles<3000000)begin @(negedge clk);cycles=cycles+1;end check(rsp_valid,"timeout");if(!rsp_valid)$fatal(1,"jacobian timeout");end endtask
 task consume;begin snapshot=scales;repeat(7)begin @(negedge clk);check(rsp_valid && !cmd_ready && scales===snapshot,"response stall");end rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;end endtask
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;repeat(3)@(negedge clk);rst_n=1;@(negedge clk);end endtask
 initial begin
 report=$fopen("jacobian_results.txt","w");original=0;for(i=0;i<27;i=i+1)original[i*64+:64]=$realtobits((i-12)*0.12);
 reset_dut();mode=0;
 for(stage=0;stage<3;stage=stage+1)begin prepare();launch();await_response();check(rsp_status==0 && received==n*240,"complete J");
 for(k=0;k<26;k=k+1)if(k<n)near(scales[k*64+:64],expected_scale[k]);else check(scales[k*64+:64]==0,"unused scales zero");consume();end
 stage=2;mode=6;prepare();launch();await_response();check(rsp_status==0,"zero derivative");for(k=0;k<n;k=k+1)near(scales[k*64+:64],1e12);consume();
 for(mode=1;mode<=5;mode=mode+1)begin stage=0;received=0;launch();await_response();check(rsp_status==(mode==1 || mode==4?4:mode==5?5:1) && scales==0,"service failure");consume();end
 mode=0;stage=3;launch();await_response();check(rsp_status==1,"bad stage");consume();
 stage=0;launch();repeat(25)@(negedge clk);reset_dut();check(cmd_ready && !rsp_valid,"reset cancellation");cases=cases+1;
 prepare();launch();await_response();check(rsp_status==0 && received==n*240,"recovery");consume();
 $fdisplay(report,"RESULT cases=%0d errors=%0d",cases,errors);$fclose(report);done=1;
 end
endmodule
