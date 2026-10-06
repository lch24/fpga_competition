// Blocking four-neighbor gather. No invalid address is ever requested.
// Exactly one logical 2-byte DDR read in flight; caller keeps job descriptors
// stable throughout a frame. Out-of-image samples are zero.
module neighbor_fetch (
 input wire clk,rst_n,
 input wire [31:0] src_base,src_stride,
 input wire in_valid,output wire in_ready,
 input wire signed [31:0] x0,y0,x1,y1,input wire [3:0] mask,
 output wire out_valid,input wire out_ready,output wire [63:0] pixels,
 output reg out_error,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_addr,rd_len,
 output wire [15:0] rd_tag,
 input wire r_valid,output wire r_ready,input wire [31:0] r_data,
 input wire [3:0] r_keep,input wire r_last,r_error,input wire [15:0] r_tag
);
 localparam IDLE=0,SELECT=1,REQUEST=2,RETURN=3,OUTPUT=4;
 reg [2:0] state;reg [1:0] n;reg [3:0] m;
 reg signed [31:0] ax,bx,ay,by;reg [15:0] p[0:3];
 wire [31:0] px=n[0]?bx:ax,py=n[1]?by:ay;
 assign in_ready=state==IDLE;assign out_valid=state==OUTPUT;
 assign pixels={p[3],p[2],p[1],p[0]};
 assign rd_valid=state==REQUEST; assign rd_addr=src_base+py*src_stride+(px<<1);
 assign rd_len=2;assign rd_tag={14'd0,n};assign r_ready=state==RETURN;
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=IDLE;n<=0;m<=0;ax<=0;bx<=0;ay<=0;by<=0;out_error<=0;for(i=0;i<4;i=i+1)p[i]<=0;end
  else case(state)
   IDLE:if(in_valid) begin ax<=x0;bx<=x1;ay<=y0;by<=y1;m<=mask;n<=0;out_error<=0;for(i=0;i<4;i=i+1)p[i]<=0;state<=SELECT;end
   SELECT:if(m[n])state<=REQUEST;else if(n==3)state<=OUTPUT;else n<=n+1;
   REQUEST:if(rd_ready)state<=RETURN;
   RETURN:if(r_valid)begin
    p[n]<=r_data[15:0];if(r_error||!r_last||r_keep!=3||r_tag!={14'd0,n})out_error<=1;
    if(n==3)state<=OUTPUT;else begin n<=n+1;state<=SELECT;end
   end
   OUTPUT:if(out_ready)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
