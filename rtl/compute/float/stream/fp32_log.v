`timescale 1ns/1ps
// Existing project-specific log polynomial, with the original FP32 rounding
// order. One transaction, one add/mul/div unit; dependent Horner operations
// reuse arithmetic. CE freezes the complete service; reset cancels it.
module fp32_log #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,in_valid, output wire in_ready,
 input wire [31:0] in_x, output wire out_valid,
 input wire out_ready, output wire [31:0] out_r
);
 localparam IDLE=0, A_REQ=1,A_WAIT=2,D_REQ=3,D_WAIT=4,
            M_REQ=5,M_WAIT=6,OUTPUT=7;
 reg [2:0] state;
 reg [2:0] step,term;
 reg [31:0] x,numerator,z,z2,a,b,result;
 wire ar,av,mr,mv,dr,dv;wire [31:0] ay,my,dy;
 function [31:0] coefficient;
   input [2:0] i;
   begin case(i)
    0:coefficient=32'h3d888889;
    1:coefficient=32'h3dba2e8c;
    2:coefficient=32'h3de38e39;
    3:coefficient=32'h3e124925;
    4:coefficient=32'h3e4ccccd;
    5:coefficient=32'h3eaaaaab;
    default:coefficient=32'h3f800000;
   endcase end
 endfunction
 assign in_ready=rst_n && state==IDLE;
 assign out_valid=rst_n && state==OUTPUT;
 assign out_r=result;
 fp32_add #(.USE_CE(USE_CE)) add(.ce(ce),.clk(clk),.rst_n(rst_n),
  .in_valid(state==A_REQ),.in_ready(ar),.in_a(a),.in_b(b),
  .out_valid(av),.out_ready(state==A_WAIT),.out_r(ay));
 fp32_mul #(.USE_CE(USE_CE)) mul(.ce(ce),.clk(clk),.rst_n(rst_n),
  .in_valid(state==M_REQ),.in_ready(mr),.in_a(a),.in_b(b),
  .out_valid(mv),.out_ready(state==M_WAIT),.out_r(my));
 fp32_div #(.USE_CE(USE_CE)) div(.ce(ce),.clk(clk),.rst_n(rst_n),
  .in_valid(state==D_REQ),.in_ready(dr),.in_a(a),.in_b(b),
  .out_valid(dv),.out_ready(state==D_WAIT),.out_r(dy));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;step<=0;term<=0;end
  else if(!USE_CE||ce)case(state)
   IDLE:if(in_valid)begin x<=in_x;a<=in_x;b<=32'hbf800000;step<=0;state<=A_REQ;end
   A_REQ:if(ar)state<=A_WAIT;
   A_WAIT:if(av)begin
     if(step==0)begin numerator<=ay;a<=x;b<=32'h3f800000;step<=1;state<=A_REQ;end
     else if(step==1)begin a<=numerator;b<=ay;state<=D_REQ;end
     else if(term==6)begin a<=z;b<=ay;step<=4;state<=M_REQ;end
     else begin a<=z2;b<=ay;term<=term+1'b1;state<=M_REQ;end
   end
   D_REQ:if(dr)state<=D_WAIT;
   D_WAIT:if(dv)begin z<=dy;a<=dy;b<=dy;step<=2;state<=M_REQ;end
   M_REQ:if(mr)state<=M_WAIT;
   M_WAIT:if(mv)begin
     if(step==2)begin z2<=my;a<=my;b<=32'h3d9d89d9;term<=0;step<=3;state<=M_REQ;end
     else if(step==3)begin a<=coefficient(term);b<=my;state<=A_REQ;end
     else if(step==4)begin a<=32'h40000000;b<=my;step<=5;state<=M_REQ;end
     else begin result<=my;state<=OUTPUT;end
   end
   OUTPUT:if(out_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
