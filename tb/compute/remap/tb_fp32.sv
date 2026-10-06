`timescale 1ns/1ps
module tb_fp32;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 reg req_valid=0;wire req_ready;reg [2:0] req_op;reg [31:0] req_a,req_b;
 wire rsp_valid;reg rsp_ready=0;wire [31:0] rsp_result;wire rsp_error;
 fp32_service dut(.*);
 reg [99:0] vectors[0:2499];reg [31:0] expected;integer i;
 initial begin
  $readmemh("fp_ops.mem",vectors);
  repeat(3)@(negedge clk);rst_n=1;
  for(i=0;i<2500;i=i+1)begin
   @(negedge clk);req_op=vectors[i][98:96];req_a=vectors[i][95:64];req_b=vectors[i][63:32];expected=vectors[i][31:0];req_valid=1;
   do @(posedge clk);while(!req_ready);@(negedge clk);req_valid=0;
   wait(rsp_valid);repeat(i%3)@(negedge clk);
   if(rsp_result!==expected)$fatal(1,"FP mismatch vector %0d op%0d a=%h b=%h got=%h expected=%h",i,req_op,req_a,req_b,rsp_result,expected);
   if(rsp_error!==(expected[30:23]==255))$fatal(1,"FP exception mismatch");
   rsp_ready=1;@(negedge clk);rsp_ready=0;
  end
  $display("PASS FP32: 2500 independent numpy binary32 vectors, subnormal/overflow/backpressure");$finish;
 end
 initial begin #1000000;$fatal(1,"FP timeout");end
endmodule
