// Exactly the original Demo's zero-extension convention at the HDMI boundary.
module rgb565_to_rgb888(input wire [15:0] pixel,output wire [7:0] r,g,b);
 assign r={pixel[15:11],3'b0};assign g={pixel[10:5],2'b0};assign b={pixel[4:0],3'b0};
endmodule
// Pixels per HREF-active line are counted HERE, where href_r/phase/pixel_valid
// are produced by the same stage and are therefore aligned by construction. The
// census is exported for the frame-end diagnostic; counting it in the capture
// module (one stage further along) mis-aligned the last pixel of every line and
// saturated the short-line counter on every frame.
module camera_byte_unpack #(parameter WIDTH=1280)(
 input wire pclk,rst_n,vsync,href,input wire [7:0] data,
 output reg pixel_valid,output reg [15:0] pixel,
 output wire [5:0] line_bad);
 // Equal-depth input stage. vsync/href/data are registered together so the pin
 // only ever drives a plain FF and all three enter the byte pairing with the
 // SAME latency. Depth is identical for all three nets, so the pixel count of a
 // clean frame is unchanged; only the whole stream shifts by one pclk.
 reg vs_r,href_r,href_d;reg [7:0] data_r;
 always @(posedge pclk or negedge rst_n)begin
  if(!rst_n)begin vs_r<=0;href_r<=0;href_d<=0;data_r<=0;end
  else begin vs_r<=vsync;href_r<=href;href_d<=href_r;data_r<=data;end
 end
 reg phase;reg [7:0] first;
 // left = pixels still to be produced for the current line. A down-counter is
 // used instead of an lcount + 12-bit compare because this one feeds the
 // pixel_valid path, and a wide compare there cost setup margin in this domain.
 reg [11:0] left;reg [5:0] bad_lines;reg vs_d;reg pad;
 // First DVP byte then second. Demo swaps the two 5-bit fields before DDR.
 wire [15:0] joined={first,data_r};
 wire line_start=href_r&&!href_d;
 wire line_end=!href_r&&href_d;
 wire room=left!=12'd0; // OR reduce, short enough to sit on the pixel path
 wire emit=href_r&&phase&&!vs_r&&room;
 // Filler emission while a short line is being topped up to WIDTH.
 wire pad_emit=pad&&room&&!vs_r;
 always @(posedge pclk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;first<=0;pixel_valid<=0;pixel<=0;left<=0;bad_lines<=0;vs_d<=0;pad<=0;end
  else begin
   pixel_valid<=0;
   vs_d<=vs_r;
   if(line_start)left<=WIDTH;
   else if(emit||pad_emit)left<=left-1'b1;
   // room == 1 at the line end means the line still owes pixels, i.e. the sampled
   // HREF window was shorter than 2*WIDTH. Only then is the line short (census)
   // and only then must it be topped up.
   if(line_end&&room&&bad_lines!=6'd63)bad_lines<=bad_lines+1'b1;
   // Cleared at the frame edge, after the value has been read out.
   if(vs_r&&!vs_d)bad_lines<=0;
   // The sampled HREF window is not always exactly 2*WIDTH pclk wide, in either
   // direction. Cap a line at WIDTH pixels so a longer window cannot push the
   // frame count above TOTAL (0x82), and top a short line back up to WIDTH so it
   // cannot fall below (0x81). Patched pixels repeat the previous sample, which
   // leaves every row's geometry unchanged.
   if(line_start||vs_r)pad<=0;
   else if(line_end)pad<=room;
   if(pad_emit)begin pixel_valid<=1;phase<=0;end
   else if(vs_r||!href_r)phase<=0;
   else if(!phase)begin first<=data_r;phase<=1;end
   else if(room)begin pixel<={joined[4:0],joined[10:5],joined[15:11]};pixel_valid<=1;phase<=0;end
   else phase<=0;
  end
 end
 assign line_bad=bad_lines;
endmodule
