`timescale 1ns/1ps
module tb_advanced_control;
 reg clk=0,core=0;always #5 clk=~clk;always #7 core=~core;
 reg rst_n=0,crst_n=0,wv=0;reg [7:0] wo;reg [15:0] wd;
 wire busy,cv,cq;wire [31:0] errors;wire [207:0] cp;
 advanced_config_bridge bridge(.clk(clk),.rst_n(rst_n),.write_valid(wv),.write_op(wo),.write_data(wd),.busy(busy),.errors(errors),
  .core_clk(core),.core_rst_n(crst_n),.cfg_valid(cv),.cfg_ready(cq),.cfg_payload(cp));
 reg iv=0,bypass=0,oq=0,pq=0,rv=0;reg [7:0] rs=0;reg [31:0] src=32'h00800000,dst=32'h01000000,pc=123;
 wire iq,ov,pv,rq,pb,pa,pg;wire [7:0] os;wire [31:0] ps,pd,pm,applied,total,first_cycles,second_cycles;
 wire [2:0] mode;wire [191:0] matrix;wire [15:0] flags;
 advanced_frame_scheduler scheduler(.clk(core),.rst_n(crst_n),.cfg_valid(cv),.cfg_ready(cq),.cfg_payload(cp),
  .in_valid(iv),.in_ready(iq),.in_src(src),.in_dst(dst),.in_map(32'h02000000),.in_bypass(bypass),
  .out_rsp_valid(ov),.out_rsp_ready(oq),.out_status(os),.cmd_valid(pv),.cmd_ready(pq),.cmd_src(ps),.cmd_dst(pd),.cmd_map(pm),
  .cmd_bypass(pb),.cmd_affine(pa),.cmd_gamma(pg),.cmd_gamma_mode(mode),.cmd_matrix(matrix),
  .rsp_valid(rv),.rsp_ready(rq),.rsp_status(rs),.pass_cycles(pc),.applied_count(applied),.last_cycles(total),
  .first_cycles(first_cycles),.second_cycles(second_cycles),.active_flags(flags));
 localparam [191:0] ID={32'd0,32'd65536,32'd0,32'd0,32'd0,32'd65536};
 reg [207:0] config1,config2,held;reg [293:0] held_command;reg [15:0] crc;integer i,k;
 function [15:0] checksum(input [207:0] p);
  reg [15:0] c;integer n,b;
  begin c=16'hffff;for(n=0;n<26;n=n+1)begin c=c^{p[n*8+:8],8'b0};for(b=0;b<8;b=b+1)c=c[15]?(c<<1)^16'h1021:c<<1;end checksum=c;end
 endfunction
 task write(input [7:0] op,input [15:0] data);
  begin @(negedge clk);wo=op;wd=data;wv=1;@(negedge clk);wv=0;end
 endtask
 task stage(input [207:0] p,input integer corrupt);
  integer n;begin for(n=0;n<13;n=n+1)write(8'h60+n,p[n*16+:16]);write(8'h6d,checksum(p)^corrupt);end
 endtask
 task start_frame;
  begin @(negedge core);iv=1;do @(posedge core);while(!iq);@(negedge core);iv=0;end
 endtask
 task accept_command;
  begin wait(pv);@(negedge core);pq=1;@(negedge core);pq=0;end
 endtask
 task respond(input [7:0] status,input [31:0] cycles);
  begin wait(rq);@(negedge core);rs=status;pc=cycles;rv=1;@(negedge core);rv=0;end
 endtask
 task take_response(input [7:0] status);
  begin wait(ov);if(os!==status)$fatal(1,"bad logical status");repeat(4)@(negedge core);if(!ov||pv||os!==status)$fatal(1,"response hold");oq=1;@(negedge core);oq=0;end
 endtask
 initial begin
  config1={16'h0003,ID};config2={16'h000a,ID}; // affine+auto gamma; then gamma manual .75
  repeat(4)@(negedge clk);rst_n=1;crst_n=1;
  write(8'h6d,0);repeat(4)@(negedge clk);if(errors!=1)$fatal(1,"incomplete commit");
  stage({16'h0080,ID},0);if(errors!=2)$fatal(1,"reserved flags");
  stage(config1,1);wait(!busy);if(errors!=3||cv)$fatal(1,"CRC rejection");
  stage(config1,0);wait(cv);held=cp;
  repeat(8)@(negedge core);if(applied!=0||!cv||cp!==held)$fatal(1,"config not held to frame boundary");
  write(8'h60,16'habcd);if(errors!=4)$fatal(1,"busy write accepted");
  start_frame();wait(pv);
  if(applied!=1||ps!=src||pd!=32'h01800000||pb||pa||pg)$fatal(1,"lens pass command");
  held_command={pm,pd,ps,matrix,pb,pa,pg,mode};repeat(5)@(negedge core);
  if({pm,pd,ps,matrix,pb,pa,pg,mode}!==held_command)$fatal(1,"command changed under stall");
  accept_command();wait(!busy);
  // Stage next config while current frame is executing. It must not leak in.
  stage(config2,0);wait(cv);respond(0,123);wait(pv);
  if(applied!=1||ps!=32'h01800000||pd!=dst||!pb||!pa||!pg||mode!=0||matrix!==ID)$fatal(1,"affine composition/latching");
  accept_command();respond(0,456);take_response(0);
  if(first_cycles!=123||second_cycles!=456||total<2)$fatal(1,"cycle accounting");
  start_frame();wait(pv);if(applied!=2||pa||pb||!pg||mode!=2||pd!=dst)$fatal(1,"single gamma pass config");
  accept_command();respond(5,88);take_response(5);wait(!busy);
  // Preview must remain an untouched one-pass comparator.
  bypass=1;start_frame();wait(pv);if(!pb||pa||pg||pd!=dst)$fatal(1,"preview changed");
  accept_command();respond(0,99);take_response(0);
  bypass=0;stage(config1,0);wait(cv);start_frame();accept_command();respond(5,17);take_response(5);
  if(second_cycles!=0)$fatal(1,"failed first pass launched second");
  // Joint reset drops a held transaction instead of replaying it.
  wait(!busy);stage(config2,0);wait(cv);
  @(negedge clk);rst_n=0;crst_n=0;repeat(4)@(negedge clk);rst_n=1;crst_n=1;repeat(8)@(negedge core);
  if(cv||busy||applied||errors)$fatal(1,"mailbox joint reset replay");
  $display("PASS advanced atomic CRC/CDC/frame boundary/two-pass composition/backpressure/error/preview/reset");$finish;
 end
 initial begin #1000000;$fatal(1,"control timeout");end
endmodule
