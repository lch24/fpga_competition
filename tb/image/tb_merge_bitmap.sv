`timescale 1ns/1ps
module tb_merge_bitmap;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,ce=1,start=0,res_ready=0;
 reg [15:0] n_in=0;wire busy,done,rd_en,res_valid;
 wire [3:0] rd_addr;reg [31:0] rd_x=0,rd_y=0;
 wire [31:0] res_x,res_y;wire [14:0] res_count;
 reg [31:0] points[0:15];integer ticks=0,checks=0,cases=0,seen=0;
 reg finished=0;
 candidate_merge #(.USE_CE(1),.N_ADDR_W(4)) dut(.ce(ce),.clk(clk),.rst_n(rst_n),
  .start(start),.busy(busy),.done(done),.n_in(n_in),.radius(32'h40a00000),
  .rd_en(rd_en),.rd_addr(rd_addr),.rd_x(rd_x),.rd_y(rd_y),
  .res_valid(res_valid),.res_ready(res_ready),.res_x(res_x),.res_y(res_y),.res_count(res_count));
 function [31:0] as_float(input integer v);
  integer b,k;reg [31:0] mant;
  begin b=0;for(k=0;k<31;k=k+1)if(v[k])b=k;
   mant=v<<(23-b);as_float=v==0?0:((127+b)<<23)|(mant&32'h7fffff);
  end
 endfunction
 always @(negedge clk)begin ticks=ticks+1;ce=(ticks%7!=0 && ticks%7!=1);res_ready=ticks%5==0;end
 always @(posedge clk)if(rst_n&&ce&&rd_en)begin rd_x<=points[rd_addr];rd_y<=0;end
 task run_case(input integer count);
  integer k,expected;
  begin
   for(k=0;k<16;k=k+1)points[k]=as_float((k/2)*16+(k%2)*2);
   @(negedge clk);n_in=count;start=1;
   do @(posedge clk);while(!ce);
   @(negedge clk);start=0;seen=0;
   while(!done)begin
    @(posedge clk);
    if(ce&&res_valid&&res_ready)begin
     expected=seen*16+((seen*2+1<count)?1:0);
     if(res_x!==as_float(expected)||res_y!==0)$fatal(1,"merge bitmap case %d output %d",count,seen);
     seen=seen+1;checks=checks+1;
    end
    @(negedge clk);
   end
   if(seen!=(count+1)/2||res_count!=seen)$fatal(1,"merge bitmap count %d %d",seen,res_count);
   cases=cases+1;
  end
 endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  run_case(16);run_case(7);run_case(16);run_case(1);run_case(0);
  // Reset during bitmap clearing, then rebuild every active bit on retry.
  @(negedge clk);n_in=16;start=1;do @(posedge clk);while(!ce);
  @(negedge clk);start=0;wait(dut.state==14);@(negedge clk);rst_n=0;
  repeat(3)@(negedge clk);rst_n=1;run_case(16);
  finished=1;$finish;
 end
 initial begin #10000000;$fatal(1,"merge bitmap timeout");end
endmodule
