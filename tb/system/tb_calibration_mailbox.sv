`timescale 1ns/1ps
module tb_calibration_mailbox;
 reg clk=0;always #5 clk=~clk;reg rst_n=0;
 reg begin_valid=0,param_valid=0,calib_rsp_valid=0,result_ready=0;
 wire begin_ready,param_ready,calib_rsp_ready,result_valid;
 reg [31:0] begin_job_id=1,param_calib_id=1,calib_rsp_id=1;
 reg [15:0] param_width=1280,param_height=720;reg param_camera_valid=1;
 reg [287:0] param_values=288'h123456789abcdef;reg [7:0] calib_rsp_status=0;
 wire [7:0] result_status;wire [31:0] result_calib_id;
 wire [15:0] result_width,result_height;wire [287:0] result_values;
 calibration_mailbox dut(.*);
 task launch(input integer id);
  begin @(negedge clk);begin_job_id=id;begin_valid=1;do @(posedge clk);while(!begin_ready);@(negedge clk);begin_valid=0;end
 endtask
 task packet(input integer id);
  begin @(negedge clk);param_calib_id=id;param_valid=1;do @(posedge clk);while(!param_ready);@(negedge clk);param_valid=0;end
 endtask
 task response(input integer id,input integer status);
  begin @(negedge clk);calib_rsp_id=id;calib_rsp_status=status;calib_rsp_valid=1;do @(posedge clk);while(!calib_rsp_ready);@(negedge clk);calib_rsp_valid=0;end
 endtask
 task result(input integer id,input integer status);
  begin wait(result_valid);repeat(5)@(negedge clk);
   if(result_status!=status||result_calib_id!=id||begin_ready)$fatal(1,"mailbox result");
   if(status==0&&(result_width!=1280||result_height!=720||result_values!==param_values))$fatal(1,"mailbox payload");
   result_ready=1;@(negedge clk);result_ready=0;end
 endtask
 initial begin repeat(4)@(negedge clk);rst_n=1;
  launch(1);packet(1);repeat(4)@(negedge clk);if(result_valid)$fatal(1,"published before completion");response(1,0);result(1,0);
  launch(2);response(2,0);repeat(4)@(negedge clk);if(result_valid)$fatal(1,"published before parameters");packet(2);result(2,0);
  launch(3);response(3,4);result(3,4);
  launch(4);packet(99);response(4,0);result(4,4);
  launch(5);packet(5);response(88,0);result(5,4);
  $display("PASS calibration mailbox: packet/response in either order, IDs, failures and backpressure");$finish;
 end
 initial begin #20000;$fatal(1,"mailbox timeout");end
endmodule
