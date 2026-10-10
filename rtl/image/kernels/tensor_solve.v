`timescale 1ns/1ps
// Compatibility interface for the tensor entry of the shared feature program.
// Arithmetic, branch order, CE and done-hold semantics are owned by that engine.
module tensor_solve #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,start,output wire busy,done,
 input wire [63:0] in_a,in_b,in_c,in_bx,in_by,
 output wire out_ok,output wire [63:0] out_dx,out_dy
);
 feature_program #(.USE_CE(USE_CE)) engine(
  .clk(clk),.rst_n(rst_n),.ce(ce),.start(start),.refine(1'b0),
  .in_a(in_a),.in_b(in_b),.in_c(in_c),.in_bx(in_bx),.in_by(in_by),.in_x(32'd0),.in_y(32'd0),
  .busy(busy),.done(done),.out_ok(out_ok),.out_dx(out_dx),.out_dy(out_dy),.out_x(),.out_y(),.out_convergence(),
  .math_req_valid(),.math_req_ready(1'b0),.math_req_op(),.math_req_a(),.math_req_b(),.math_active(),
  .math_rsp_valid(1'b0),.math_rsp_ready(),.math_result(64'd0),.math_flags(5'd0));
endmodule
