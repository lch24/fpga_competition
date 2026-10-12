`timescale 1ns/1ps
module tb_board_flow;
 reg finished=0,clk=0,rst=0,camera=0,sready=0,fready=0,release_v=0,capready=0,caprsp=0,rsp=0,display_idle=1;
 reg [7:0] capstatus=0,rspstatus=0;reg [4:0] pressed=0;
 wire sv,fv,release_r,cv,cr,rr,owner,display_on,waiting,display_owner,remap,calibrated,corrected;
 wire [7:0] fs,status;wire [3:0] phase;wire [2:0] captured;wire [1:0] selected;
 wire [31:0] cap_base,frame_base,start_base,display_base;
 always #5 clk=~clk;
 board_flow dut(.clk(clk),.rst_n(rst),.camera_ready(camera),.pressed(pressed),.display_idle(display_idle),
 .start_valid(sv),.start_ready(sready),.start_remap(remap),.start_base(start_base),
 .frame_ready(fready),.frame_valid(fv),.frame_status(fs),.frame_base(frame_base),
 .frame_release_valid(release_v),.frame_release_ready(release_r),.cap_cmd_valid(cv),.cap_cmd_ready(capready),.cap_base(cap_base),
 .cap_rsp_valid(caprsp),.cap_rsp_ready(cr),.cap_status(capstatus),.rsp_valid(rsp),.rsp_status(rspstatus),.rsp_ready(rr),
 .capture_owner(owner),.display_enable(display_on),.display_owner(display_owner),.display_base(display_base),
 .status(status),.waiting(waiting),.phase(phase),.captured(captured),.calibrated(calibrated),.corrected(corrected),.selected(selected));
 task tick;begin @(negedge clk);end endtask
 task press(input integer key);begin tick();pressed=1<<key;tick();pressed=0;end endtask
 task photo(input integer slot,input integer code);
  begin press(slot);wait(cv);repeat(3)tick();
   if(!owner||cap_base!=slot*32'h200000)$fatal(1,"capture address/ownership");
   capready=1;tick();capready=0;press(3); // busy input must not enqueue
   capstatus=code;caprsp=1;tick();caprsp=0;
   if(status!=code||captured[slot]!=(code==0))$fatal(1,"capture commit");
  end
 endtask
 task result(input integer code);
  begin wait(rr);rspstatus=code;rsp=1;tick();rsp=0;repeat(2)tick();end
 endtask
 task calibration;
  integer j;begin
   press(3);wait(sv);repeat(3)tick();if(remap)$fatal(1,"cal opcode");sready=1;tick();sready=0;
   for(j=0;j<3;j=j+1)begin
    wait(fv);repeat(3)tick();if(frame_base!=j*32'h200000)$fatal(1,"lease address");
    fready=1;tick();fready=0;repeat(3)tick();release_v=1;tick();release_v=0;
   end
  end
 endtask
 initial begin
  repeat(3)tick();rst=1;repeat(10)tick();if(sv||cv)$fatal(1,"auto start");
  press(0);if(status!=8'h80)$fatal(1,"camera not ready guard");
  press(3);if(status!=8'h81)$fatal(1,"missing photos guard");
  press(4);if(status!=8'h82)$fatal(1,"missing parameters guard");camera=1;
  photo(2,3);photo(2,0);photo(0,0);photo(1,0);
  if(captured!=7||!display_on)$fatal(1,"photos missing");
  display_idle=0;press(2);repeat(6)tick();
  if(!display_owner||display_on||cv||sv||selected!=1)$fatal(1,"display drain ownership");
  display_idle=1;tick();if(selected!=2||display_base!=32'h400000||cv)$fatal(1,"photo selection");
  calibration();result(4);if(calibrated||!display_on)$fatal(1,"failed calibration");
  calibration();result(0);if(!calibrated||corrected)$fatal(1,"calibration auto-remapped");
  press(4);wait(sv);tick();if(!remap||start_base!=32'h400000)$fatal(1,"remap descriptor");
  sready=1;tick();sready=0;result(5);if(corrected||!calibrated||display_base!=32'h400000)$fatal(1,"failed remap fallback");
  press(4);wait(sv);tick();sready=1;tick();sready=0;result(0);
  if(!corrected||display_base!=32'h1000000)$fatal(1,"corrected display");
  press(0);wait(waiting);if(display_base!=0||!calibrated)$fatal(1,"return raw");
  press(3);wait(sv);tick(); // global reset while a command is pending
  rst=0;tick();rst=1;tick();if(captured||calibrated||corrected||display_on)$fatal(1,"reset");
  finished=1;$display("PASS tb_board_flow manual controls, failure/retry, drain, reset");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
