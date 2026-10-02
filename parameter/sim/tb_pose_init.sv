`timescale 1ns/1ps
// 独立软件参考向量；检查命令锁存、数值、失败状态、响应背压、复位与恢复。
module tb_pose_init;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;wire [7:0] rsp_status;
 reg [15:0] width,height,cmd_width,cmd_height;
 reg [1727:0] hom,cmd_hom;reg [255:0] k,cmd_k;
 reg [2559:0] points;
 reg [1:0] view,cmd_view;
 integer drop,reads;reg read_valid=0;reg [31:0] read_x,read_y;
 wire read_en;wire [1:0] read_view;wire [5:0] read_index;
 wire [1727:0] result_bits;reg [1727:0] expected,snapshot;
 reg [7:0] expected_status,status_snapshot;
 reg done=0;integer errors=0,cases=0,protocol_cases=0,max_cycles=0;
 integer fd,report,rc,cycles,j;
 real av,ev,difference,tolerance;
 pose_init dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
   .cmd_width(cmd_width),.cmd_height(cmd_height),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),
   .cmd_h_all_fp64(cmd_hom),.cmd_k_fp64(cmd_k),.rsp_state(result_bits));

 task check;input condition;input [511:0] label;
 begin if(condition!==1'b1)begin errors=errors+1;if(errors<40)$fdisplay(report,"FAIL case=%0d %0s cycle=%0d",cases,label,cycles);end end endtask
 task load_vector;
 begin rc=$fscanf(fd,"%d %d %d %h %h %h\n",expected_status,width,height,hom,k,expected);end endtask
 task launch;
 begin @(negedge clk);check(cmd_ready,"ready before command");reads=0;cmd_width=width;cmd_height=height;cmd_hom=hom;cmd_k=k;cmd_view=view;cmd_valid=1;
   @(negedge clk);cmd_valid=0;cmd_width=0;cmd_height=0;cmd_hom=~hom;cmd_k=~k;cmd_view=3;check(!cmd_ready,"busy ready low");end endtask
 task await_response;
 begin cycles=0;while(!rsp_valid && cycles<1500000)begin @(negedge clk);cycles=cycles+1;check(!cmd_ready,"busy excludes new command");end
   check(rsp_valid,"response timeout");if(!rsp_valid)$fatal(1,"timeout");if(cycles>max_cycles)max_cycles=cycles;end endtask
 task check_result;
 begin check(rsp_status===expected_status,"response status");
   if(expected_status==0)begin
     for(j=0;j<27;j=j+1)begin av=$bitstoreal(result_bits[j*64 +:64]);ev=$bitstoreal(expected[j*64 +:64]);difference=av-ev;if(difference<0)difference=-difference;
       tolerance=2e-9*(1+(ev<0?-ev:ev));
       if(!((^result_bits[j*64 +:64])!==1'bx && result_bits[j*64+52 +:11]!=2047 && difference<=tolerance))begin
         check(0,"numeric result");$fdisplay(report,"element=%0d actual=%.17g expected=%.17g diff=%.9g tolerance=%.9g",j,av,ev,difference,tolerance);end
     end

   end else check(result_bits===0,"failed payload zero");end endtask
 task reset_dut;
 begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;repeat(3)@(negedge clk);check(!rsp_valid && !cmd_ready,"reset gates valid");rst_n=1;@(negedge clk);check(cmd_ready,"reset recovery");end endtask
 initial begin
   report=$fopen("pose_init_results.txt","w");fd=$fopen("../pose_init_vectors.txt","r");if(!fd || !report)$fatal(1,"file open");
   cycles=0;view=0;drop=-1;reset_dut();
   while(!$feof(fd))begin load_vector();if(rc==6)begin
     launch();await_response();check_result();snapshot=result_bits;status_snapshot=rsp_status;cmd_valid=1;
     repeat(7)begin @(negedge clk);check(rsp_valid && !cmd_ready && result_bits===snapshot && rsp_status===status_snapshot,"response stable under stall");end
     cmd_valid=0;rsp_ready=1;@(negedge clk);rsp_ready=0;check(!rsp_valid && cmd_ready,"response consumed");cases=cases+1;
   end else if(rc!=-1)$fatal(1,"malformed vector");end
   $fclose(fd);fd=$fopen("../pose_init_vectors.txt","r");load_vector();$fclose(fd);
   launch();repeat(30)@(negedge clk);reset_dut();repeat(100)begin @(negedge clk);check(!rsp_valid && cmd_ready,"cancelled computation");end protocol_cases=protocol_cases+1;

   launch();await_response();reset_dut();protocol_cases=protocol_cases+1;
   launch();await_response();check_result();rsp_ready=1;@(negedge clk);rsp_ready=0;protocol_cases=protocol_cases+1;
   $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d errors=%0d max_cycles=%0d",cases,protocol_cases,errors,max_cycles);$fclose(report);done=1;
 end
endmodule
