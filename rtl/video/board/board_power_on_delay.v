`timescale 1ns / 1ps
// Sensor startup: power settling -> RESETB low -> SCCB access enabled.
// Defaults retain the board's original delays. At its actual 27 MHz input,
// these are about 9.7 ms, 2.4 ms and 38.8 ms (the port name is historical).
// All outputs reset immediately, including during FPGA warm reconfiguration.
module board_power_on_delay #(
    parameter POWER_WAIT_CYCLES=262144,
    parameter RESET_WAIT_CYCLES=65535,
    parameter CONFIG_WAIT_CYCLES=1048575
)(
    input  clk_50M        ,
    input  reset_n        ,
    output camera1_rstn   ,
    output camera2_rstn   ,
    output camera_pwnd    ,
    output reg initial_en
);
localparam POWER_BITS=POWER_WAIT_CYCLES<2?1:$clog2(POWER_WAIT_CYCLES+1);
localparam RESET_BITS=RESET_WAIT_CYCLES<2?1:$clog2(RESET_WAIT_CYCLES+1);
localparam CONFIG_BITS=CONFIG_WAIT_CYCLES<2?1:$clog2(CONFIG_WAIT_CYCLES+1);
reg [POWER_BITS-1:0]cnt1;
reg [RESET_BITS-1:0]cnt2;
reg [CONFIG_BITS-1:0]cnt3;
reg camera_rstn_reg;
reg camera_pwnd_reg;

assign camera1_rstn=camera_rstn_reg;
assign camera2_rstn=camera_rstn_reg;
assign camera_pwnd=camera_pwnd_reg;

// Hold the sensor reset throughout the initial power-settling interval.
always@(posedge clk_50M or negedge reset_n)begin
  if(reset_n==1'b0) begin
	    cnt1<=0;
		camera_pwnd_reg<=1'b1;// 1'b1 
  end
  else if(cnt1<POWER_WAIT_CYCLES) begin
       cnt1<=cnt1+1'b1;
       camera_pwnd_reg<=1'b1;
  end
  else
     camera_pwnd_reg<=1'b0;         
end

// Continue RESETB low after power settling, then release the sensor.
always@(posedge clk_50M or negedge reset_n)begin
  if(!reset_n)begin cnt2<=0;camera_rstn_reg<=0;end
  else if(camera_pwnd_reg==1)  begin
	 cnt2<=0;
     camera_rstn_reg<=1'b0;  
  end
  else if(cnt2<RESET_WAIT_CYCLES) begin
       cnt2<=cnt2+1'b1;
       camera_rstn_reg<=1'b0;
  end
  else
     camera_rstn_reg<=1'b1;         
end

// OV5640 requires at least 20 ms from RESETB high to SCCB initialization.
always@(posedge clk_50M or negedge reset_n)begin
  if(!reset_n)begin cnt3<=0;initial_en<=0;end
  else if(camera_rstn_reg==0) begin
         cnt3<=0;
         initial_en<=1'b0;
  end
  else if(cnt3<CONFIG_WAIT_CYCLES) begin
        cnt3<=cnt3+1'b1;
        initial_en<=1'b0;
  end
  else
       initial_en<=1'b1;    
end

endmodule
