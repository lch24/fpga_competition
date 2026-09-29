// Combinational FP32 map -> Q16 fractions / four integer neighbors.
// Quantization uses 16 fractional bits (max 1/65536 pixel). Negative coordinates
// use mathematical floor. Non-finite samples always produce four black values.
module sample_coord (
 input wire [31:0] src_x,src_y,input wire [15:0] width,height,
 input wire border_replicate,
 output reg signed [31:0] x0,y0,x1,y1,
 output reg [15:0] dx,dy,output reg [3:0] mask
);
 function signed [63:0] to_fixed;
  input [31:0] f;
  reg [63:0] m,v;
  integer shift;
  begin
   m={40'd0, f[30:23]!=0,f[22:0]};
   shift=((f[30:23]==0)?-126:({24'd0,f[30:23]}-127))-7;
   if(shift>30) v=64'h00007fffffff0000;
   else if(shift>=0) v=m<<shift;
   else if(shift<= -64) v=(m!=0&&f[31])?64'd1:64'd0;
   else begin
    v=m>>(-shift);
    // For negative numbers, round the magnitude UP to implement floor.
    if(f[31] && (m & ((64'd1<<(-shift))-1))!=0) v=v+1;
   end
   to_fixed=f[31]?-$signed(v):$signed(v);
  end
 endfunction
 reg signed [63:0] qx,qy;
 integer i;
 reg signed [31:0] xx,yy;
 always @* begin
  qx=to_fixed(src_x); qy=to_fixed(src_y);
  if(border_replicate) begin
   if(qx<0) qx=0; if(qy<0) qy=0;
   if(qx>($signed({1'b0,width})-1)*65536) qx=($signed({1'b0,width})-1)*65536;
   if(qy>($signed({1'b0,height})-1)*65536) qy=($signed({1'b0,height})-1)*65536;
  end
  x0=qx>>>16; y0=qy>>>16; x1=x0+1; y1=y0+1;
  dx=qx[15:0]; dy=qy[15:0]; mask=0; xx=0;yy=0;
  if(border_replicate) begin
   if(x1>=$signed({1'b0,width})) x1=$signed({1'b0,width})-1;
   if(y1>=$signed({1'b0,height})) y1=$signed({1'b0,height})-1;
  end
  for(i=0;i<4;i=i+1) begin
   xx=(i%2)?x1:x0; yy=(i/2)?y1:y0;
   mask[i]=(src_x[30:23]!=255)&&(src_y[30:23]!=255)&&
        width!=0&&height!=0&&xx>=0&&yy>=0&&
        xx<$signed({1'b0,width})&&yy<$signed({1'b0,height});
  end
 end
endmodule
