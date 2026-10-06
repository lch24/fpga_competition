`timescale 1ns/1ps
module tb_fp_pool;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0;reg [3:0] c_req_valid=0,c_active=15,c_rsp_ready=0;
 reg [19:0] c_req_op=0;reg [255:0] c_req_a=0,c_req_b=0;
 wire [3:0] c_req_ready,c_rsp_valid;wire [63:0] result;wire [4:0] flags;
 fp_calibration_pool #(.CLIENTS(4)) dut(.*);
 integer cycle=0,accepted=0,returned=0,j;
 reg [3:0] received=0;reg finished=0;
 reg held=0;reg [63:0] held_result;reg [3:0] held_valid;
 always @(posedge clk) if(rst_n)begin
  cycle<=cycle+1;
  for(j=0;j<4;j=j+1)begin
   if(c_req_valid[j]&&c_req_ready[j])begin accepted<=accepted+1;c_req_valid[j]<=0;end
   if(c_rsp_valid[j]&&c_rsp_ready[j])begin
    if(j==0||received[j])$fatal(1,"cancelled/duplicate delivery");
    case(j)
     1:if(result!==64'h4010000000000000)$fatal(1,"ADD routing");
     2:if(result!==64'h4018000000000000)$fatal(1,"MUL routing");
     3:if(result!==64'h4000000000000000)$fatal(1,"SQRT routing");
    endcase
    returned<=returned+1;received[j]<=1;
   end
  end
  if(held && (result!==held_result || c_rsp_valid!==held_valid))$fatal(1,"pool result changed under backpressure");
  held<=|(c_rsp_valid & ~c_rsp_ready);held_result<=result;held_valid<=c_rsp_valid;
 end
 always @(negedge clk)if(rst_n)c_rsp_ready=(cycle%13>8)?15:0;
 initial begin
  c_req_op={5'd4,5'd2,5'd0,5'd3};
  c_req_a={64'h4010000000000000,64'h4000000000000000,64'h3ff0000000000000,64'h4000000000000000};
  c_req_b={64'd0,64'h4008000000000000,64'h4008000000000000,64'h4008000000000000};
  repeat(3)@(negedge clk);rst_n=1;c_req_valid=15;
  wait(accepted==1);repeat(10)@(negedge clk);c_active[0]=0;
  repeat(2)@(negedge clk);c_active[0]=1;
  wait(returned==3);repeat(5)@(negedge clk);
  if(accepted!=4 || received!=14)$fatal(1,"pool completion");
  finished=1;$finish;
 end
 initial begin #100000;$fatal(1,"pool timeout");end
endmodule
