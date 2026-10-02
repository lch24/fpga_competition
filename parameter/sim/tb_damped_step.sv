`timescale 1ns/1ps
module tb_damped_step;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;wire [7:0] rsp_status;
 reg [1:0] stage=0;reg done=0;integer errors=0,cases=0,cycles=0,report;
 task check;input condition;input [511:0] label;begin if(condition!==1'b1)begin
 errors=errors+1;if(errors<40)$fdisplay(report,"FAIL case=%0d %0s",cases,label);end end endtask
 task near;input [63:0] bits;input real expected;input real tolerance;real x,d;begin
 x=$bitstoreal(bits);d=x-expected;if(d<0)d=-d;
 check((^bits)!==1'bx && bits[62:52]!=2047 && d<=tolerance*(1+(expected<0?-expected:expected)),"numeric mismatch");
 if(d>tolerance*(1+(expected<0?-expected:expected)) && errors<40)$fdisplay(report,"actual=%0.17g expected=%0.17g",x,expected);
 end endtask
 task launch;begin @(negedge clk);check(cmd_ready,"command ready");cmd_valid=1;@(negedge clk);cmd_valid=0;end endtask
 task await_response;begin cycles=0;while(!rsp_valid && cycles<100000000)begin @(negedge clk);cycles=cycles+1;end check(rsp_valid,"timeout");if(!rsp_valid)$fatal(1,"timeout");end endtask
 task consume;reg [7:0] status;begin status=rsp_status;repeat(7)begin @(negedge clk);check(rsp_valid && rsp_status===status && !cmd_ready,"stalled response");end rsp_ready=1;@(negedge clk);rsp_ready=0;cases=cases+1;end endtask

 reg load_valid=0,load_abort=0,ng_valid=0,ng_kind=0,ng_last=0,load_rsp_ready=0;
 wire load_ready,abort_ready,ng_ready,load_rsp_valid;wire [7:0] load_status;
 reg [4:0] ng_row=0,ng_col=0;reg [63:0] ng_value=0,lambda;wire [1663:0] delta;
 integer n,i,j,m,retry,mode;real lam,u,b,den,sumub,sumuu,expected;
 reg [1663:0] snapshot;
 damped_step dut(.clk(clk),.rst_n(rst_n),.load_valid(load_valid),.load_ready(load_ready),.load_stage(stage),
 .load_abort_valid(load_abort),.load_abort_ready(abort_ready),.ng_valid(ng_valid),.ng_ready(ng_ready),.ng_kind(ng_kind),.ng_row(ng_row),.ng_col(ng_col),.ng_fp64(ng_value),.ng_last(ng_last),
 .load_rsp_valid(load_rsp_valid),.load_rsp_ready(load_rsp_ready),.load_rsp_status(load_status),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_lambda_fp64(lambda),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_delta_fp64(delta));
 function real uv;input integer k;begin uv=(k%7-3)*0.1;end endfunction
 function real gv;input integer k;begin gv=(k%5-2)*0.13;end endfunction
 task start_load;begin @(negedge clk);check(load_ready,"load ready");load_valid=1;@(negedge clk);load_valid=0;end endtask
 task load_response;input integer status;begin cycles=0;while(!load_rsp_valid && cycles<1000)begin @(negedge clk);cycles=cycles+1;end
 check(load_rsp_valid && load_status==status,"load status");repeat(5)begin @(negedge clk);check(load_rsp_valid && load_status==status && !cmd_ready,"load response stability");end
 load_rsp_ready=1;@(negedge clk);load_rsp_ready=0;end endtask
 task send;input integer kind,row,col;input real value;input integer last;begin
 @(negedge clk);ng_valid=1;ng_kind=kind;ng_row=row;ng_col=col;ng_value=$realtobits(value);ng_last=last;
 check(ng_ready,"ng ready");@(negedge clk);ng_valid=0;end endtask
 task fill;begin start_load();for(i=0;i<n;i=i+1)for(j=0;j<=i;j=j+1)send(0,i,j,mode==1?(i==j?-1.0:0.0):uv(i)*uv(j)+(i==j?1+i*0.03:0),0);
 for(i=0;i<n;i=i+1)send(1,i,0,gv(i),i==n-1);load_response(0);end endtask
 task solve_check;begin
 lambda=$realtobits(lam);launch();await_response();
 if(mode==1)check(rsp_status==4 && delta==0,"singular failure");else begin
 check(rsp_status==0,"solve status");sumub=0;sumuu=0;
 for(i=0;i<n;i=i+1)begin den=1+i*0.03+lam;sumub=sumub+uv(i)*(-gv(i))/den;sumuu=sumuu+uv(i)*uv(i)/den;end
 for(i=0;i<n;i=i+1)begin den=1+i*0.03+lam;expected=(-gv(i))/den-uv(i)/den*sumub/(1+sumuu);near(delta[i*64+:64],expected,1e-12);end
 for(i=n;i<26;i=i+1)check(delta[i*64+:64]==0,"unused delta zero");end
 snapshot=delta;repeat(9)begin @(negedge clk);check(delta===snapshot && rsp_valid,"delta stability");end
 // solve响应期间不得接收新load。
 load_valid=1;#1;check(!load_ready,"no load during solve response");load_valid=0;consume();end endtask
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;ng_valid=0;load_valid=0;load_abort=0;rsp_ready=0;repeat(3)@(negedge clk);rst_n=1;@(negedge clk);end endtask
 initial begin report=$fopen("damped_step_results.txt","w");reset_dut();check(!cmd_ready,"cannot solve before load");
 mode=0;for(stage=0;stage<3;stage=stage+1)begin n=stage==0?22:stage==1?23:26;fill();for(retry=0;retry<3;retry=retry+1)begin lam=retry==0?0.001:retry==1?0.1:10;solve_check();end end
 stage=0;n=22;mode=1;fill();lam=1;solve_check();mode=0;fill();
 for(retry=0;retry<16;retry=retry+1)begin
 case(retry%4)0:lambda=64'h7ff0000000000000;1:lambda=0;2:lambda=64'hbff0000000000000;3:lambda=64'h7ff8000000000000;endcase
 launch();await_response();check(rsp_status==1,"bad lambda");consume();end
 check(!cmd_ready && load_ready,"16 attempt limit");fill();lam=0.3;solve_check();
 start_load();send(0,0,0,1,1);load_response(1);check(!cmd_ready,"bad load invalidates cache");cases=cases+1;
 start_load();@(negedge clk);load_abort=1;ng_valid=1;#1;check(!ng_ready,"abort priority");@(negedge clk);load_abort=0;ng_valid=0;load_response(4);cases=cases+1;
 stage=3;start_load();load_response(1);cases=cases+1;stage=0;
 start_load();@(negedge clk);ng_valid=1;ng_kind=0;ng_row=0;ng_col=0;ng_last=0;ng_value=64'h7ff0000000000000;
 @(negedge clk);ng_valid=0;load_response(4);cases=cases+1;
 // 对角相加溢出时，高斯子核正等待矩阵：必须取消，再用原缓存重试。
 start_load();for(i=0;i<n;i=i+1)for(j=0;j<=i;j=j+1)send(0,i,j,i==j?1e308:0.0,0);
 for(i=0;i<n;i=i+1)send(1,i,0,gv(i),i==n-1);load_response(0);
 lambda=$realtobits(1e308);launch();await_response();check(rsp_status==4 && delta==0,"diagonal overflow cancels partial solver input");consume();
 lambda=$realtobits(0.01);launch();await_response();check(rsp_status==0,"retry after partial solver cancellation");
 for(i=0;i<n;i=i+1)near(delta[i*64+:64],-gv(i)/1e308,1e-320);consume();
 fill();lam=0.01;lambda=$realtobits(lam);launch();repeat(60)@(negedge clk);reset_dut();check(!rsp_valid && !cmd_ready && load_ready,"reset clears matrix");cases=cases+1;
 fill();solve_check();$fdisplay(report,"RESULT cases=%0d errors=%0d",cases,errors);$fclose(report);done=1;end
endmodule
