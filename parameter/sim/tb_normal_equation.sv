`timescale 1ns/1ps
module tb_normal_equation;
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

 reg abort_valid=0;wire abort_ready;
 reg r_valid=0,j_valid=0;wire r_ready,j_ready;
 reg [7:0] r_index=0,j_row=0;reg [4:0] j_col=0;
 reg [63:0] r_value=0,j_value=0;reg r_last=0,j_last=0;
 wire ng_valid,ng_kind,ng_last;reg ng_ready=0;wire [4:0] ng_row,ng_col;wire [63:0] ng_value,max_gradient;
 integer n,a,b,t,s,mode,got;real total,expected_max;
 reg [76:0] held;reg was_stalled=0;integer tick=0;
 normal_equation dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_stage(stage),
 .abort_valid(abort_valid),.abort_ready(abort_ready),.r_valid(r_valid),.r_ready(r_ready),.r_index(r_index),.r_fp64(r_value),.r_last(r_last),
 .j_valid(j_valid),.j_ready(j_ready),.j_row(j_row),.j_col(j_col),.j_fp64(j_value),.j_last(j_last),
 .ng_valid(ng_valid),.ng_ready(ng_ready),.ng_kind(ng_kind),.ng_row(ng_row),.ng_col(ng_col),.ng_fp64(ng_value),.ng_last(ng_last),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_max_gradient_fp64(max_gradient));
 function real jref;input integer row,col;begin jref=mode==1?0:(mode==2 && row==0 && col==0)?1e308:((row*17+col*13)%31-15)*0.007+(row==col?0.2:0);end endfunction
 function real rref;input integer row;begin rref=((row*7)%19-9)*0.031;end endfunction
 always @(negedge clk)begin tick=tick+1;if(rst_n)ng_ready=(tick%7!=0 && tick%7!=1);else ng_ready=0;end
 always @(posedge clk)if(rst_n)begin
  if(was_stalled && !abort_valid)check(ng_valid && {ng_kind,ng_row,ng_col,ng_value,ng_last}===held,"N/g stall stability");
  was_stalled=ng_valid && !ng_ready;held={ng_kind,ng_row,ng_col,ng_value,ng_last};
  if(ng_valid && ng_ready)begin
   check(ng_kind==s && ng_row==a && ng_col==(s?0:b) && ng_last==(s && a==n-1),"N/g order");
   total=0;for(t=0;t<240;t=t+1)total=total+jref(t,a)*(s?rref(t):jref(t,b));
   near(ng_value,total,3e-13);got=got+1;
   if(s)begin if((total<0?-total:total)>expected_max)expected_max=(total<0?-total:total);a=a+1;end
   else if(a==b)begin b=0;if(a==n-1)begin s=1;a=0;end else a=a+1;end else b=b+1;
  end
 end else was_stalled=0;
 task feed;integer ri,ci,rj;begin
 fork
 begin for(ri=0;ri<240;ri=ri+1)begin @(negedge clk);r_valid=1;r_index=ri;r_value=$realtobits(rref(ri));r_last=ri==239;while(!r_ready)@(negedge clk);@(negedge clk);r_valid=0;end end
 begin for(ci=0;ci<n;ci=ci+1)for(rj=0;rj<240;rj=rj+1)begin @(negedge clk);j_valid=1;j_col=ci;j_row=rj;j_value=$realtobits(jref(rj,ci));j_last=ci==n-1 && rj==239;while(!j_ready)@(negedge clk);@(negedge clk);j_valid=0;end end
 join end endtask
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;r_valid=0;j_valid=0;abort_valid=0;rsp_ready=0;repeat(3)@(negedge clk);rst_n=1;@(negedge clk);end endtask
 initial begin report=$fopen("normal_equation_results.txt","w");reset_dut();
 for(mode=0;mode<2;mode=mode+1)for(stage=0;stage<3;stage=stage+1)begin
 n=stage==0?22:stage==1?23:26;a=0;b=0;s=0;got=0;expected_max=0;
 launch();feed();await_response();check(rsp_status==0 && got==n*(n+1)/2+n,"normal complete");near(max_gradient,expected_max,3e-13);consume();end
 stage=3;launch();await_response();check(rsp_status==1,"bad stage");consume();
 stage=0;launch();@(negedge clk);r_valid=1;r_index=1;@(negedge clk);r_valid=0;await_response();check(rsp_status==1,"bad residual index");consume();
 launch();@(negedge clk);j_valid=1;j_col=0;j_row=0;j_last=1;@(negedge clk);j_valid=0;await_response();check(rsp_status==1,"early J last");consume();
 launch();@(negedge clk);r_valid=1;r_index=0;r_last=0;r_value=64'h7ff0000000000000;@(negedge clk);r_valid=0;await_response();check(rsp_status==4,"nonfinite residual");consume();
 launch();@(negedge clk);abort_valid=1;r_valid=1;#1;check(!r_ready,"abort priority");@(negedge clk);abort_valid=0;r_valid=0;await_response();check(rsp_status==4,"abort partial load");consume();
 mode=0;n=22;a=0;b=0;s=0;expected_max=0;launch();feed();repeat(50)@(negedge clk);abort_valid=1;@(negedge clk);abort_valid=0;await_response();check(rsp_status==4,"abort computing");consume();
 mode=2;n=22;a=0;b=0;s=0;got=0;launch();feed();await_response();check(rsp_status==4 && got==0,"normal arithmetic overflow");consume();mode=0;
 launch();reset_dut();check(cmd_ready && !rsp_valid,"reset cancels load");cases=cases+1;
 n=22;a=0;b=0;s=0;got=0;expected_max=0;launch();feed();await_response();check(rsp_status==0,"recovery");consume();
 $fdisplay(report,"RESULT cases=%0d errors=%0d",cases,errors);$fclose(report);done=1;end
endmodule
