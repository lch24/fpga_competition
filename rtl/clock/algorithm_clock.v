`timescale 1ns/1ps
// Dedicated algorithm clock: 125 MHz / 5 * 2 * 20 / 25 = 40 MHz.
// PLL VCO = 1000 MHz. DDR and video PLL configurations are unchanged.
// This wrapper is board-only; portable tests drive independent clocks directly.
module algorithm_clock(input wire clkin, output wire clkout, output wire locked);
 GTP_GPLL #(.CLKIN_FREQ(125.0),.STATIC_RATIOI(5),.STATIC_RATIOM(2),
  .STATIC_RATIOF(20.0),.STATIC_RATIO0(25.0),.STATIC_DUTY0(25),
  .INTERNAL_FB("CLKOUTF"),.EXTERNAL_FB("DISABLE")) u_gpll(
  .CLKIN1(clkin),.CLKIN2(1'b0),.CLKIN_SEL(1'b0),.CLKFB(1'b0),
  .CLKOUT0(clkout),.LOCK(locked),.PLL_PWD(1'b0),.RST(1'b0),
  .DPS_CLK(1'b0),.DPS_EN(1'b0),.DPS_DIR(1'b0),
  .CLKOUT0_SYN(1'b0),.CLKOUT1_SYN(1'b0),.CLKOUT2_SYN(1'b0),
  .CLKOUT3_SYN(1'b0),.CLKOUT4_SYN(1'b0),.CLKOUT5_SYN(1'b0),
  .CLKOUT6_SYN(1'b0),.CLKOUTF_SYN(1'b0),
  .APB_CLK(1'b0),.APB_RST_N(1'b0),.APB_ADDR(5'd0),.APB_SEL(1'b0),
  .APB_EN(1'b0),.APB_WRITE(1'b0),.APB_WDATA(16'd0));
endmodule
