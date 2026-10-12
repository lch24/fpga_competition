`timescale 1ns/1ps
module tb_core_debug;
 reg clk=0,rst=0,fn=0;always #5 clk=~clk;
 reg [31:0] state_word=0;reg [4:0] pressed=0;
 reg busy=0,de=0,uf=0,ra=0,rr=0,wa=0,wr=0;
 wire [255:0] snapshot;reg finished=0;
 reg [15:0] argument=16'hffff;
 core_debug #(.VIDEO_ENABLE(1)) dut(.clk(clk),.rst_n(rst),.functional_reset_n(fn),.state_word(state_word),
 .pressed(pressed),.busy(busy),.display_error(de),.underflow(uf),
 .read_accept(ra),.read_return(rr),.write_accept(wa),.write_return(wr),
 .read_addr(32'h123),.write_addr(32'h456),.snapshot(snapshot),.argument(argument),
 .captured_frames(32'd11),.corrected_frames(32'd10),.displayed_frames(32'd9),.failed_frames(32'd1),.skipped_frames(32'd2),.last_remap_cycles(32'd3000000));
 task tick;begin @(negedge clk);end endtask
 initial begin
  tick;rst=1;fn=1;state_word=32'h00810000;de=1;uf=1;pressed=1;busy=1;ra=1;rr=1;
  tick;de=0;uf=0;pressed=0;ra=0;rr=0;wa=1;wr=1;state_word=0;
  tick;wa=0;wr=0;fn=0;tick;
  if(snapshot[63:32]!=2||snapshot[127:96]!=1||snapshot[159:128]!=2||snapshot[191:160]!=1)$fatal(1,"event/activity counters");
  if(snapshot[223:192]!=32'h456)$fatal(1,"last address");
  if(snapshot[255:240]!=1||snapshot[239:232]!=8'h81||snapshot[230:224]!=6)$fatal(1,"sticky failure reset retention");
  argument=16'hfffd;tick;
  if(snapshot[95:64]!=11||snapshot[127:96]!=10||snapshot[159:128]!=9||snapshot[191:160]!=1||snapshot[223:192]!=2||snapshot[255:224]!=3000000)$fatal(1,"video counter page");
  $display("PASS tb_core_debug: counters, errors, busy key, reset retention");finished=1;$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
