`timescale 1ns/1ps
// 所有计算子模块均为真实RTL。参考向量由原C++ finish_result生成。
module tb_validate_result;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,cmd_valid=0,rsp_ready=0;wire cmd_ready,rsp_valid;
 reg [15:0] width,height,cmd_width,cmd_height;reg converged,cmd_converged;
 reg [63:0] square,cost,cmd_square,cmd_cost;reg [1727:0] state_bits,cmd_state;
 reg [7679:0] points;integer drop,reads=0,mode=0;
 wire read_en;wire [1:0] read_view;wire [5:0] read_index;
 reg read_valid=0;reg [31:0] read_x,read_y;
 wire [7:0] status;wire usable,metrics,geometry_weak;wire [287:0] camera;wire [63:0] rms,max_error;
 wire [191:0] view_rms;wire [2303:0] poses;wire [2623:0] actual={poses,max_error,view_rms,rms};
 reg [287:0] expected_camera;reg [2623:0] expected;
 integer expected_status,expected_usable,expected_metrics,fd,report,rc,cycles,j;
 integer cases=0,protocol_cases=0,errors=0,max_cycles=0,map_checks=0,previous_map_x=0,previous_map_y=0;
 reg done=0;reg [2922:0] snapshot;
 validate_result dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
 .cmd_width(cmd_width),.cmd_height(cmd_height),.cmd_state(cmd_state),.cmd_square_size_fp64(cmd_square),.cmd_best_cost_fp64(cmd_cost),.cmd_converged(cmd_converged),
 .point_rd_en(read_en),.point_rd_view_id(read_view),.point_rd_index(read_index),.point_rd_valid(read_valid),.point_rd_x_fp32(read_x),.point_rd_y_fp32(read_y),
 .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(status),.rsp_camera_usable(usable),.rsp_camera_params(camera),
 .rsp_metrics_valid(metrics),.rsp_weak_geometry(geometry_weak),.rsp_rms_fp64(rms),.rsp_view_rms_fp64(view_rms),.rsp_max_error_fp64(max_error),.rsp_poses_fp64(poses));
 task check;input condition;input [511:0] label;begin if(condition!==1'b1)begin errors=errors+1;if(errors<50)$fdisplay(report,"FAIL case=%0d %0s pc=%0d",cases,label,dut.pc);end end endtask
 task near;input [63:0] bits,reference;real a,b,d;begin
 a=$bitstoreal(bits);b=$bitstoreal(reference);d=a-b;if(d<0)d=-d;
 if(!((^bits)!==1'bx && bits[62:52]!=2047 && d<=2e-9*(1+(b<0?-b:b))))begin check(0,"C++ value mismatch");$fdisplay(report,"item=%0d actual=%.17g expected=%.17g",j,a,b);end end endtask
 always @(posedge clk)begin
  read_valid<=0;
  if(rst_n && read_en)begin
   check(read_view==reads/40 && read_index==reads%40 && reads<120,"read order");
   read_valid<=reads!=drop;read_x<=points[(read_view*40+read_index)*64+:32];read_y<=points[(read_view*40+read_index)*64+32+:32];reads=reads+1;
  end
  if(rst_n && dut.pc==dut.MAP_CHECK)begin
   check(dut.ix==map_checks%33 && dut.iy==map_checks/33,"mapping grid traversal");map_checks=map_checks+1;
  end
 end
 reg injected=0;
 always @(negedge clk)begin
  if(injected)begin release dut.res_index;release dut.rot_status;injected=0;end
  if(mode==1 && dut.pc==dut.RES_WAIT && dut.res_data_valid)begin force dut.res_index=8'd255;injected=1;end
  if(mode==2 && dut.pc==dut.ROT_WAIT && dut.rot_valid)begin force dut.rot_status=8'd4;injected=1;end
 end
 task load_vector;begin rc=$fscanf(fd,"%d %d %d %d %d %d %d %h %h %h %h %h %h\n",expected_status,expected_usable,expected_metrics,width,height,converged,drop,square,cost,state_bits,points,expected_camera,expected);end endtask
 task reset_dut;begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;repeat(3)@(negedge clk);check(!rsp_valid && !read_en,"reset gates outputs");rst_n=1;@(negedge clk);check(cmd_ready,"reset ready");end endtask
 task launch;begin @(negedge clk);check(cmd_ready,"command ready");reads=0;map_checks=0;
 cmd_width=width;cmd_height=height;cmd_state=state_bits;cmd_square=square;cmd_cost=cost;cmd_converged=converged;cmd_valid=1;
 @(negedge clk);cmd_valid=0;cmd_width=1;cmd_height=1;cmd_state=~state_bits;cmd_square=0;cmd_cost=64'h7ff0000000000000;cmd_converged=~converged;end endtask
 task await_response;begin cycles=0;while(!rsp_valid && cycles<3000000)begin @(negedge clk);cycles=cycles+1;check(!cmd_ready,"busy command blocked");end
 check(rsp_valid,"timeout");if(!rsp_valid)$fatal(1,"validate timeout");if(cycles>max_cycles)max_cycles=cycles;end endtask
 task verify;begin
 check(status==expected_status && usable==expected_usable && metrics==expected_metrics,"result flags");
 check(camera===expected_camera,"FP32 camera bits/order");check(geometry_weak==expected_metrics,"three view geometry_weak geometry flag");
 if(metrics)begin for(j=0;j<41;j=j+1)near(actual[64*j+:64],expected[64*j+:64]);check(reads==120,"all observations read");end
 else check(actual==0 && camera==0 && !usable,"failed computation hides partial payload");
 if(usable)check(map_checks==825,"all 825 mapping samples tested");
 end endtask
 task consume;begin
 snapshot={status,usable,metrics,geometry_weak,camera,actual};cmd_valid=1;
 repeat(9)begin @(negedge clk);check(rsp_valid && !cmd_ready && {status,usable,metrics,geometry_weak,camera,actual}===snapshot,"response stable under backpressure");end
 cmd_valid=0;rsp_ready=1;@(negedge clk);rsp_ready=0;check(cmd_ready && !rsp_valid,"response consumed");end endtask
 initial begin
 report=$fopen("validate_result_results.txt","w");fd=$fopen("../validate_result_vectors.txt","r");if(!fd || !report)$fatal(1,"file open");reset_dut();
 while(!$feof(fd))begin load_vector();if(rc==13)begin
 launch();await_response();verify();$fdisplay(report,"case=%0d status=%0d metrics=%0d usable=%0d cycles=%0d map_samples=%0d",cases,status,metrics,usable,cycles,map_checks);consume();cases=cases+1;
 end else if(rc!=-1)$fatal(1,"vector format");end
 $fclose(fd);fd=$fopen("../validate_result_vectors.txt","r");load_vector();$fclose(fd);
 launch();while(reads<5)@(negedge clk);reset_dut();repeat(20)@(negedge clk);check(!rsp_valid,"cancel residual evaluation");protocol_cases=protocol_cases+1;
 launch();while(!(dut.map_phase && dut.pc==dut.FP_WAIT && dut.fp_op==3))@(negedge clk);reset_dut();protocol_cases=protocol_cases+1;
 launch();await_response();reset_dut();protocol_cases=protocol_cases+1;
 mode=1;launch();await_response();check(status==1 && !metrics && !usable,"malformed residual stream");consume();mode=0;protocol_cases=protocol_cases+1;
 mode=2;launch();await_response();check(status==4 && !metrics && !usable,"rotation failure discards partial metrics");consume();mode=0;protocol_cases=protocol_cases+1;
 launch();await_response();verify();consume();protocol_cases=protocol_cases+1;
 $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d errors=%0d max_cycles=%0d",cases,protocol_cases,errors,max_cycles);$fclose(report);done=1;
 end
endmodule
