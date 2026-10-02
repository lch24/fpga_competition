// Exactly the original Demo's zero-extension convention at the HDMI boundary.
module rgb565_to_rgb888(input wire [15:0] pixel,output wire [7:0] r,g,b);
 assign r={pixel[15:11],3'b0};assign g={pixel[10:5],2'b0};assign b={pixel[4:0],3'b0};
endmodule
module camera_byte_unpack(input wire pclk,rst_n,vsync,href,input wire [7:0] data,
 output reg pixel_valid,output reg [15:0] pixel);
 reg phase;reg [7:0] first;
 // First DVP byte then second. Demo swaps the two 5-bit fields before DDR.
 wire [15:0] joined={first,data};
 always @(posedge pclk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;first<=0;pixel_valid<=0;pixel<=0;end
  else begin
   pixel_valid<=0;
   if(vsync||!href)phase<=0;
   else if(!phase)begin first<=data;phase<=1;end
   else begin pixel<={joined[4:0],joined[10:5],joined[15:11]};pixel_valid<=1;phase<=0;end
  end
 end
endmodule
