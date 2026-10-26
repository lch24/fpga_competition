// Exactly the original Demo's zero-extension convention at the HDMI boundary.
module rgb565_to_rgb888(input wire [15:0] pixel,output wire [7:0] r,g,b);
 assign r={pixel[15:11],3'b0};assign g={pixel[10:5],2'b0};assign b={pixel[4:0],3'b0};
endmodule
// Pixels per HREF-active line are counted HERE, where href_r/phase/pixel_valid
// are produced by the same stage and are therefore aligned by construction. The
// census is exported for the frame-end diagnostic; counting it in the capture
// module (one stage further along) mis-aligned the last pixel of every line and
// saturated the short-line counter on every frame.
module camera_byte_unpack #(parameter WIDTH=1280,HEIGHT=720,
 parameter CROP_X=0,CROP_Y=0,CROP_WIDTH=WIDTH,CROP_HEIGHT=HEIGHT)(
 input wire pclk,rst_n,vsync,href,input wire [7:0] data,
 output reg pixel_valid,output reg [15:0] pixel,
 output wire [5:0] line_bad,
 output reg [3:0] frame_error,
 output wire capture_vsync);
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
 reg [11:0] left;reg [5:0] bad_lines,frame_bad_lines;reg vs_d;
 reg line_long;
 reg [3:0] frame_faults;
 // Row position comes from HREF, never from a possibly shortened pixel stream.
 // Only output VALID is cropped; pairing and short-line census see all DVP.
 localparam ROW_BITS=$clog2(HEIGHT+2);
 reg [ROW_BITS-1:0] next_row;
 reg row_selected;
 wire column_selected=left<=WIDTH-CROP_X && left>WIDTH-CROP_X-CROP_WIDTH;
 // First DVP byte then second. Demo swaps the two 5-bit fields before DDR.
 wire [15:0] joined={first,data_r};
 wire line_start=href_r&&!href_d;
 wire line_end=!href_r&&href_d;
 wire room=left!=12'd0; // OR reduce, short enough to sit on the pixel path
 wire emit=href_r&&phase&&!vs_r&&room;
 // Never manufacture pixels to make a damaged frame reach the expected count.
 // A short tail outside the crop is harmless; missing ROI data, odd byte
 // pairing, surplus bytes or a wrong row count invalidate the entire frame.
 wire [3:0] line_faults={1'b0,row_selected&&line_long,
     row_selected&&phase,row_selected&&(left>WIDTH-CROP_X-CROP_WIDTH)};
 always @(posedge pclk or negedge rst_n)begin
  if(!rst_n)begin phase<=0;first<=0;pixel_valid<=0;pixel<=0;left<=0;bad_lines<=0;frame_bad_lines<=0;vs_d<=0;next_row<=0;row_selected<=0;line_long<=0;frame_faults<=0;frame_error<=0;end
  else begin
   pixel_valid<=0;
   vs_d<=vs_r;
   if(vs_r)begin next_row<=0;row_selected<=0;end
   else if(line_start)begin
    row_selected<=next_row>=CROP_Y && next_row<CROP_Y+CROP_HEIGHT;
    if(next_row<HEIGHT+1)next_row<=next_row+1'b1;
   end
   if(line_start)begin left<=WIDTH;line_long<=0;end
   else if(emit)left<=left-1'b1;
   else if(href_r&&phase&&!vs_r&&!room)line_long<=1;
   // room == 1 at the line end means the line still owes pixels, i.e. the sampled
   // HREF window was shorter than 2*WIDTH. Record the shortfall without padding.
   if(line_end&&room&&bad_lines!=6'd63)bad_lines<=bad_lines+1'b1;
   if(line_end)frame_faults<=frame_faults|line_faults;
   // Snapshot at the frame edge, including a coincident final HREF fall.
   if(vs_r&&!vs_d)begin
    frame_bad_lines<=bad_lines+(line_end&&room&&bad_lines!=6'd63);
    frame_error<=frame_faults|(line_end?line_faults:4'd0)|
                 (next_row!=HEIGHT?4'b1000:4'd0);
    frame_faults<=0;
    bad_lines<=0;
   end
   if(vs_r||!href_r)phase<=0;
   else if(!phase)begin first<=data_r;phase<=1;end
   else if(room)begin pixel<={joined[4:0],joined[10:5],joined[15:11]};pixel_valid<=row_selected&&column_selected;phase<=0;end
   else phase<=0;
  end
 end
 // Hold the completed frame census across VSYNC; capture consumes it after
 // the input pipeline has drained, not the already-cleared live counter.
 assign line_bad=frame_bad_lines;
 // video_capture_hmic registers this once more internally. Its frame edge is
 // then evaluated after the last registered pixel, with the latched census.
 assign capture_vsync=vs_r;
endmodule
