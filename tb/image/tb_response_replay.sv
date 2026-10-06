`timescale 1ns/1ps
// Independent NMS oracle: positive FP32 ordering, threshold, border and ties.
// Exercise bubbles, long output stalls, reset cancellation and consecutive jobs.
module response_replay_case #(parameter W=17,H=11)(output reg finished=0);
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0,start=0,thr_valid=0,resp_valid=0,cand_ready=0;
 reg [31:0] thr=32'h40000000,resp_data=0;
 wire resp_ready,first_pass,replay_reset,replay_start,done,cand_valid;
 wire [10:0] cand_x,cand_y;
 wire [15:0] cand_total;
 shi_tomasi_replay #(.IMG_W(W),.IMG_H(H)) dut(.*);
 reg [31:0] pixels[0:W*H-1];
 integer ex[0:W*H-1],ey[0:W*H-1];
 integer expected=0,observed=0,cycles=0,frame,i,x,y,dx,dy;
 reg peak;
 reg [21:0] held;
 reg stalled=0;
 always @(negedge clk) begin
  cycles=cycles+1;
  cand_ready=(cycles%101>=70);
 end
 always @(posedge clk) begin
  if(!rst_n || start) stalled<=0;
  else begin
   if(stalled && (!cand_valid || {cand_x,cand_y}!==held))
    $fatal(1,"candidate changed under backpressure");
   stalled<=cand_valid&&!cand_ready; held<={cand_x,cand_y};
   if(cand_valid&&cand_ready) begin
    if(first_pass || observed>=expected || cand_x!==ex[observed] || cand_y!==ey[observed])
     $fatal(1,"replay NMS mismatch W=%0d index=%0d got=(%0d,%0d)",W,observed,cand_x,cand_y);
    observed=observed+1;
   end
  end
 end
 task send_pixels;
  integer k;
  begin
   for(k=0;k<W*H;k=k+1) begin
    @(negedge clk); resp_valid=0;
    if(k%7==0) repeat(3) @(negedge clk);
    resp_data=pixels[k];resp_valid=1;
    @(posedge clk); while(!resp_ready) @(posedge clk);
   end
   @(negedge clk);resp_valid=0;
  end
 endtask
 initial begin
  repeat(4) @(negedge clk);rst_n=1;
  // Cancel a partially collected first pass; no stale result may survive.
  start=1;@(negedge clk);start=0;resp_valid=1;resp_data=32'h40800000;
  repeat(9) @(negedge clk);rst_n=0;resp_valid=0;
  repeat(3) @(negedge clk);rst_n=1;
  for(frame=0;frame<3;frame=frame+1) begin
   expected=0;observed=0;
   for(i=0;i<W*H;i=i+1)
    pixels[i]=frame==0 ? 32'h40800000 : frame==1 ?
      ((i%13==0)?32'h40800000:(i%5==0)?32'h40000000:32'h3f800000) : 0;
   for(y=2;y<H-2;y=y+1) for(x=2;x<W-2;x=x+1) begin
    peak=pixels[y*W+x]>=thr && pixels[y*W+x]!=0;
    for(dy=-1;dy<=1;dy=dy+1) for(dx=-1;dx<=1;dx=dx+1)
     if(pixels[(y+dy)*W+x+dx]>pixels[y*W+x])peak=0;
    if(peak)begin ex[expected]=x;ey[expected]=y;expected=expected+1;end
   end
   @(negedge clk);start=1;@(negedge clk);start=0;
   send_pixels();
   repeat(5)@(negedge clk);
   if(done||cand_valid||first_pass)$fatal(1,"bad threshold wait");
   thr_valid=1;@(negedge clk);thr_valid=0;
   wait(replay_reset);@(negedge clk);wait(replay_start);
   send_pixels();wait(done);@(negedge clk);
   if(observed!=expected || cand_total!=expected)
    $fatal(1,"candidate count W=%0d expected=%0d got=%0d",W,expected,observed);
  end
  finished=1;
 end
endmodule
module tb_response_replay;
 wire a,b;
 response_replay_case small_a(a);
 response_replay_case #(.W(8),.H(9)) small_b(b);
 initial begin wait(a&&b);$display("PASS response replay: 6 frames, two sizes, stalls and reset");$finish;end
 initial begin #2000000;$fatal(1,"response replay timeout");end
endmodule
