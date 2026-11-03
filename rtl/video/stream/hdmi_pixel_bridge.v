// Frame-marked stream -> HDMI clock. VSYNC is active high. Start only at the
// first DE after VSYNC and at an SOF token. Underflow discards the rest of the
// old frame, then waits for a new SOF and a new display frame (never shifts the
// remaining pixels). Frame dimensions must match sync_vg's active dimensions.
// This serial bring-up implementation does not promise 720p live throughput.
module hdmi_pixel_bridge #(parameter FIFO_BITS=10)(
 input wire clk,rst_n,in_valid,output wire in_ready,input wire [15:0] in_pixel,
 input wire in_first,in_last,
 input wire pclk,prst_n,de,vsync,input wire clear_error,
 output wire [7:0] r,g,b,output reg underflow
);
 wire valid;wire [17:0] token;reg running,frame_gate,vs_d;
 wire start=de&&frame_gate&&valid&&token[17];
 wire show=de&&valid&&(start||(running&&!token[17]));
 wire consume=show||(!running&&valid&&!token[17]);
 // One elastic input slot isolates the display-enable/stream-control cone
 // from the large FIFO's RAM write enable. It can drain and refill together
 // (II=1); reset discards the slot together with the rest of the frame.
 wire fifo_ready;
 reg input_pending;
 reg [17:0] input_token;
 assign in_ready=!input_pending||fifo_ready;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)input_pending<=0;
  else if(in_ready)input_pending<=in_valid;
 always @(posedge clk)
  if(in_ready&&in_valid)input_token<={in_first,in_last,in_pixel};
 async_fifo #(.WIDTH(18),.ADDR_BITS(FIFO_BITS)) fifo(
  .wclk(clk),.wrst_n(rst_n),.wvalid(input_pending),.wready(fifo_ready),.wdata(input_token),
  .rclk(pclk),.rrst_n(prst_n),.rvalid(valid),.rready(consume),.rdata(token));
 rgb565_to_rgb888 format(.pixel(show?token[15:0]:16'd0),.r(r),.g(g),.b(b));
 always @(posedge pclk or negedge prst_n)begin
  if(!prst_n)begin underflow<=0;running<=0;frame_gate<=0;vs_d<=0;end
  else begin
   vs_d<=vsync;
   if(clear_error)underflow<=0;
   if(vsync&&!vs_d)begin running<=0;frame_gate<=1;end
   else if(de)begin
    frame_gate<=0;
    if(show)running<=!token[16];
    else begin running<=0;if(frame_gate||running)underflow<=1;end
   end
  end
 end
endmodule
