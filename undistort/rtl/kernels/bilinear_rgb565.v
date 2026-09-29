// Four RGB565 pixels, Q0.16 weights. Expand channels by bit replication,
// accumulate all four weights at full precision, round once, repack RGB565.
module bilinear_rgb565 (
 input wire [15:0] p00,p10,p01,p11,dx,dy,output wire [15:0] pixel
);
 function [7:0] channel;
  input [15:0] p;input integer c;
  begin
   case(c)
    0:channel={p[4:0],p[4:2]};
    1:channel={p[10:5],p[10:9]};
    default:channel={p[15:11],p[15:13]};
   endcase
  end
 endfunction
 function [7:0] interpolate;
  input [7:0] a,b,c,d; input [15:0] x,y;
  reg [16:0] ix,iy;
  reg [63:0] total;
  begin
   ix=17'd65536-{1'b0,x}; iy=17'd65536-{1'b0,y};
   total=({56'd0,a}*ix*iy)+({56'd0,b}*x*iy)+
         ({56'd0,c}*ix*y)+({56'd0,d}*x*y)+64'h80000000;
   interpolate=total[39:32];
  end
 endfunction
 wire [7:0] b=interpolate(channel(p00,0),channel(p10,0),channel(p01,0),channel(p11,0),dx,dy);
 wire [7:0] g=interpolate(channel(p00,1),channel(p10,1),channel(p01,1),channel(p11,1),dx,dy);
 wire [7:0] r=interpolate(channel(p00,2),channel(p10,2),channel(p01,2),channel(p11,2),dx,dy);
 assign pixel={r[7:3],g[7:2],b[7:3]};
endmodule
