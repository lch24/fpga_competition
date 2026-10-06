`timescale 1ns/1ps
module tb_board_flow;
 reg finished=0;
 reg clk=0,rst=0,camera=0,sready=0,fready=0,release_v=0,capready=0,caprsp=0,rsp=0;
 reg [7:0] capstatus=0,rspstatus=0;
 wire sv,fv,release_r,cv,cr,rr,owner,display_on,waiting;
 wire [7:0] fs,status;wire [3:0] phase;
 always #5 clk=~clk;
 board_flow #(.CAPTURE_WAIT_CYCLES(5)) dut(.clk(clk),.rst_n(rst),.camera_ready(camera),
 .start_valid(sv),.start_ready(sready),.frame_ready(fready),.frame_valid(fv),.frame_status(fs),
 .frame_release_valid(release_v),.frame_release_ready(release_r),.cap_cmd_valid(cv),.cap_cmd_ready(capready),
 .cap_rsp_valid(caprsp),.cap_rsp_ready(cr),.cap_status(capstatus),.rsp_valid(rsp),.rsp_status(rspstatus),.rsp_ready(rr),
 .capture_owner(owner),.display_enable(display_on),.status(status),.waiting(waiting),.phase(phase));
 task tick;begin @(negedge clk);end endtask
 task boot;
  begin rst=0;camera=0;sready=0;fready=0;repeat(3)tick();rst=1;repeat(3)tick();
   if(sv)$fatal(1,"started before camera init");camera=1;repeat(3)tick();
   if(!sv)$fatal(1,"start must persist");sready=1;tick();sready=0;fready=1;
  end
 endtask
 task view(input integer code);
  begin
   repeat(4)begin tick();if(cv)$fatal(1,"capture too early");end
   wait(cv);repeat(3)tick();if(!owner||!cv)$fatal(1,"capture ownership");capready=1;tick();capready=0;
   repeat(3)tick();if(!owner)$fatal(1,"lease during capture");capstatus=code;caprsp=1;tick();caprsp=0;
   fready=0;repeat(3)tick();if(!fv||fs!=code||owner)$fatal(1,"frame publication");fready=1;tick();fready=0;
   if(code==0)begin
    repeat(5)tick();if(!release_r||cv)$fatal(1,"overwrite frozen frame");release_v=1;tick();release_v=0;
   end
  end
 endtask
 integer i;
 initial begin
  boot();for(i=0;i<3;i=i+1)begin fready=1;view(0);end
  rsp=1;rspstatus=0;tick();rsp=0;repeat(8)tick();
  if(!display_on||sv||cv||owner)$fatal(1,"successful transfer to display");
  boot();view(3);rsp=1;rspstatus=3;tick();rsp=0;repeat(8)tick();
  if(display_on||status!=3||sv||cv)$fatal(1,"failed capture displayed");
  finished=1;
  $display("PASS tb_board_flow three views, handshakes, frozen leases, terminal success/failure");$finish;
 end
 initial begin #100000;$fatal(1,"timeout flow");end
endmodule
