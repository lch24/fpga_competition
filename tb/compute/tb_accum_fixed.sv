`timescale 1ns/1ps
module tb_accum_fixed #(parameter COUNT=100);
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0,in_valid=0,out_ready=0;
 reg [15:0] n_win;
 reg [63:0] in_x,in_y,in_w,in_gx,in_gy;
 wire busy,done,in_ready,out_valid;
 wire [63:0] out_a,out_b,out_c,out_bx,out_by;
 reg [319:0] expected;
 integer fd,rc,checked=0,samples=0,i,j,ticks=0;reg finished=0;
 subpixel_accum_fixed #(.USE_CE(1)) dut(.*);
 always @(negedge clk)begin ticks=ticks+1;ce=(ticks%7!=0);end
 task accept;
  input integer kind;
  begin @(posedge clk);while(!ce || (kind==1&&!in_ready))@(posedge clk);@(negedge clk);#1;end
 endtask
 initial begin
  fd=$fopen("accum.txt","r");if(!fd)$fatal(1,"vectors");
  repeat(3)@(negedge clk);#1;rst_n=1;
  // Cancel an in-flight window and then start a complete new job.
  n_win=5;start=1;accept(0);start=0;
  in_x=0;in_y=0;in_w=64'h3ff0000000000000;in_gx=in_w;in_gy=in_w;
  in_valid=1;accept(1);in_valid=0;repeat(5)@(negedge clk);#1;rst_n=0;
  repeat(2)@(negedge clk);#1;rst_n=1;
  for(i=0;i<COUNT;i=i+1)begin
   rc=$fscanf(fd,"%d %h\n",n_win,expected);if(rc!=2)$fatal(1,"header");
   start=1;accept(0);start=0;
   for(j=0;j<n_win;j=j+1)begin
    rc=$fscanf(fd,"%h %h %h %h %h\n",in_x,in_y,in_w,in_gx,in_gy);if(rc!=5)$fatal(1,"sample");
    in_valid=1;accept(1);in_valid=0;samples=samples+1;
   end
   wait(out_valid);repeat(5)begin @(negedge clk);#1;
    if(!out_valid || {out_a,out_b,out_c,out_bx,out_by}!==expected)$fatal(1,"fixed accum case %0d mismatch",i);
   end
   out_ready=1;accept(0);out_ready=0;checked=checked+1;
   repeat(2)@(negedge clk);#1;
  end
  finished=1;$finish;
 end
 initial begin #200000000;$fatal(1,"watchdog");end
endmodule
