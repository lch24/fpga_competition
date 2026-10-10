`timescale 1ns/1ps
module tb_calib_engine;
 reg clk=0;always #12.5 clk=~clk;
 reg rst_n=0,start_valid=0,rsp_ready=0;
 reg [15:0] start_pc=0,program_words=0;
 wire start_ready,imem_en,busy,rsp_valid;wire [15:0] imem_addr,debug_pc;
 reg [31:0] imem_data;reg [31:0] rom[0:2047];
 reg host_en=0,host_write=0;reg [31:0] host_addr=0;reg [63:0] host_data=0;
 wire host_ready,host_valid,host_error;wire [63:0] host_result;
 wire [7:0] rsp_status;wire [31:0] cycle_count,instruction_count;
 reg done=0;integer errors=0,checked=0,fd,report,fields,action,addr,cycles;
 reg [63:0] value;reg [7:0] held_status;reg [31:0] held_cycles;
 real got_real,expected_real,difference,tolerance,max_relative=0;
 calib_datapath dut(.shared_req_valid(),.shared_req_ready(1'b0),.shared_req_op(),
  .shared_req_a(),.shared_req_b(),.shared_rsp_ready(),.shared_rsp_valid(1'b0),
  .shared_rsp_result(64'd0),.shared_rsp_flags(5'd0),.svc_valid(),.svc_ready(1'b0),.svc_id(),.*);
 always @(posedge clk)if(imem_en)imem_data<=rom[imem_addr];
 task failure;input [511:0] message;begin errors=errors+1;$display("FAIL action=%0d addr=%h pc=%h: %0s",action,addr,debug_pc,message);end endtask
 task reset;
  begin
   @(negedge clk);rst_n=0;start_valid=0;rsp_ready=0;host_en=0;
   repeat(2)@(negedge clk);rst_n=1;
   @(negedge clk);if(!start_ready || rsp_valid)failure("reset");
  end
 endtask
 initial begin
  $readmemh("checks.mem",rom);fd=$fopen("checks.txt","r");report=$fopen("cycles.csv","w");
  $fdisplay(report,"entry,cycles,instructions,status");
  if(!fd)begin failure("missing transactions");done=1;end
  else begin
   reset;
   while(!$feof(fd))begin
    fields=$fscanf(fd,"%d %h %h\n",action,addr,value);
    if(fields==3)begin
     checked=checked+1;
     case(action)
      0,2,5:begin
       @(negedge clk);if(!host_ready)failure("host not ready");
       host_en=1;host_write=action==0;host_addr=addr;host_data=value;
       @(posedge clk);#1;
       if(!host_valid || host_error)failure("host response");
       if(action==2 && host_result!==value)begin
        $display("got=%h expected=%h",host_result,value);failure("memory mismatch");
       end
       if(action==5)begin
        got_real=$bitstoreal(host_result);expected_real=$bitstoreal(value);
        difference=got_real-expected_real;if(difference<0)difference=-difference;
        tolerance=expected_real;if(tolerance<0)tolerance=-tolerance;
        // Provisional math/projection bound, fixed before running experiments.
        tolerance=2.0e-11+2.0e-11*tolerance;
        if((^host_result)===1'bx || host_result[62:52]==2047 || difference>tolerance)begin
         $display("got=%.17g expected=%.17g delta=%g limit=%g",got_real,expected_real,difference,tolerance);
         failure("numerical tolerance");
        end
        if(expected_real!=0 && difference/(expected_real<0?-expected_real:expected_real)>max_relative)
         max_relative=difference/(expected_real<0?-expected_real:expected_real);
       end
       @(negedge clk);host_en=0;
      end
      1,4:begin
       @(negedge clk);if(!start_ready)failure("start not ready");
       start_pc=addr;start_valid=1;
       @(negedge clk);start_valid=0;
       if(action==4)begin
        repeat(30)@(negedge clk);
        if(!busy)failure("cancel requires active job");reset;
        repeat(150)begin @(negedge clk);if(rsp_valid || !start_ready)failure("stale completion after reset");end
       end else begin
        cycles=0;
        while(!rsp_valid && cycles<1000000)begin @(negedge clk);cycles=cycles+1;end
        if(!rsp_valid)failure("job timeout");
        else begin
         if(rsp_status!==value[7:0])begin $display("status=%h expected=%h",rsp_status,value[7:0]);failure("job status");end
         $display("CASE entry=%0d cycles=%0d instructions=%0d status=%h",addr,cycle_count,instruction_count,rsp_status);
         $fdisplay(report,"%0d,%0d,%0d,%h",addr,cycle_count,instruction_count,rsp_status);
         held_status=rsp_status;held_cycles=cycle_count;
         repeat(3)begin
          @(negedge clk);
          if(!rsp_valid || rsp_status!==held_status || cycle_count!==held_cycles || host_ready || start_ready)failure("completion backpressure");
         end
         rsp_ready=1;@(negedge clk);rsp_ready=0;
        end
       end
      end
      3:program_words=addr;
      default:failure("unknown test action");
     endcase
    end else if(fields!=-1)begin failure("malformed transactions");$finish;end
   end
   $fdisplay(report,"max_relative,%g",max_relative);$fclose(fd);$fclose(report);done=1;
  end
 end
endmodule
