`timescale 1ns/1ps
// Active-low mechanical keys: two-flop CDC, stable press/release debounce,
// one pulse per press. A held key never repeats. Default: 20 ms at 125 MHz.
module board_buttons #(parameter DEBOUNCE_CYCLES=2500000, ENABLE_DISTORT_DEMO=0, LONG_PRESS_CYCLES=250000000)(
 input wire clk,rst_n,input wire [4:0] key_n,output wire [4:0] pressed,output wire toggle_mode
);
 reg [4:0] raw_pressed;
 localparam CW=DEBOUNCE_CYCLES<2?1:$clog2(DEBOUNCE_CYCLES);
 reg [4:0] meta,sync /* synthesis PAP_ASYNC_REG=1 */;
 reg [4:0] stable;
 reg [CW-1:0] count[0:4];
 integer i;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin meta<=5'b11111;sync<=5'b11111;end
  else begin meta<=key_n;sync<=meta;end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   stable<=5'b11111;raw_pressed<=0;
   for(i=0;i<5;i=i+1)count[i]<=0;
  end else begin
   raw_pressed<=0;
   for(i=0;i<5;i=i+1)begin
    if(sync[i]==stable[i])count[i]<=0;
    else if(DEBOUNCE_CYCLES<=1||count[i]==DEBOUNCE_CYCLES-1)begin
     stable[i]<=sync[i];count[i]<=0;raw_pressed[i]<=!sync[i];
    end else count[i]<=count[i]+1'b1;
   end
  end
 end
 generate if(ENABLE_DISTORT_DEMO)begin : g_long_press
  localparam LW=LONG_PRESS_CYCLES<2?1:$clog2(LONG_PRESS_CYCLES);
  reg [LW-1:0] ticks;reg down,fired,short_press,toggle;
  assign pressed={raw_pressed[4],short_press,raw_pressed[2:0]};assign toggle_mode=toggle;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin ticks<=0;down<=0;fired<=0;short_press<=0;toggle<=0;end
   else begin
    short_press<=0;toggle<=0;down<=!stable[3];
    if(!stable[3])begin
     if(!fired)begin
      if(LONG_PRESS_CYCLES<=1||ticks==LONG_PRESS_CYCLES-1)begin fired<=1;toggle<=1;end
      else ticks<=ticks+1'b1;
     end
    end else begin
     if(down&&!fired)short_press<=1;
     ticks<=0;fired<=0;
    end
   end
  end
 end else begin : g_normal_press
  assign pressed=raw_pressed;assign toggle_mode=0;
 end endgenerate
endmodule
