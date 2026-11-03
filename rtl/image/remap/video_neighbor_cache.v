`timescale 1ns/1ps
// Direct-mapped source cache, separate even/odd row banks. 64 pixels per line,
// 4-beat line fill. Tags and pixels use synchronous BRAM. Valid bits use
// single-write-port distributed RAM banks of at most 64 entries. Bank outputs
// and the bank select are registered together, keeping a full 128:1 selector
// off the address calculation path without adding a lookup cycle.
// Valid bits are cleared at every frame start; tags stay in block RAM.
// Both neighbor rows looked up together. Common-case II=2; horizontal 16-pixel
// beat crossings require one additional lookup. Misses stall, never read stale
// source data. All tags invalidated between frames even when a base is reused.
module video_neighbor_cache #(parameter WIDTH=1280,INDEX_BITS=6)(
 input wire clk,rst_n,start,input wire [31:0] src_base,
 input wire in_valid,output wire in_ready,input wire [63:0] descriptor,input wire map_error,
 output wire out_valid,input wire out_ready,output wire [63:0] pixels,
 output wire [15:0] dx,dy,output reg error,output wire out_in_bounds,
 output wire arvalid,input wire arready,output wire [27:0] araddr,output wire [3:0] arlen,
 input wire rvalid,input wire [255:0] rdata,input wire rlast,input wire [3:0] rid
);
 localparam LINES=1<<INDEX_BITS;
 localparam IDLE=0,CHECK=1,CROSS_READ=2,CROSS_CHECK=3,REQUEST=4,FILL=5,RETRY=6,CLEAR=7;
 reg [2:0] state;reg second,fill_odd;
 reg [31:0] base,a0,a1;reg [63:0] desc;reg [15:0] p0,p1,p2,p3;
 (* syn_ramstyle="block_ram" *) reg [22:0] te[0:LINES-1],to_[0:LINES-1];
 (* syn_ramstyle="block_ram" *) reg [255:0] de[0:LINES*4-1],do_[0:LINES*4-1];
 reg [22:0] tqe,tqo;reg [255:0] qe,qo;
 wire vqe,vqo;
 reg [INDEX_BITS-1:0] clear_index;
 reg [31:0] fill_address;reg [1:0] fill_beat;
 wire [3:0] mask=map_error||error?4'b0:desc[63:60];
 assign out_in_bounds=|mask;
 wire [13:0] x=desc[45:32],y=desc[59:46];
 wire row0=|mask[1:0],row1=|mask[3:2];
 wire cross0=mask[0]&&mask[1]&&x[3:0]==15;
 wire cross1=mask[2]&&mask[3]&&x[3:0]==15;
 wire crossing=cross0||cross1;
 // Addresses latched at input; secondary word is the next physical beat.
 wire [31:0] r0=second?{a0[31:5],5'b0}+32:a0;
 wire [31:0] r1=second?{a1[31:5],5'b0}+32:a1;
 wire [31:0] even_address=y[0]?r1:r0,odd_address=y[0]?r0:r1;
 wire need_even=y[0]?(second?cross1:row1):(second?cross0:row0);
 wire need_odd=y[0]?(second?cross0:row0):(second?cross1:row1);
 wire hit_e=!need_even||(vqe&&tqe==even_address[29:7]);
 wire hit_o=!need_odd||(vqo&&tqo==odd_address[29:7]);
 wire [255:0] q0=y[0]?qo:qe,q1=y[0]?qe:qo;
 assign in_ready=state==IDLE;
 wire accept=in_valid&&in_ready;
 wire lookup=accept||state==RETRY||state==CROSS_READ;
 wire [13:0] ix=descriptor[45:32],iy=descriptor[59:46];
 wire signed [31:0] signed_x={{18{ix[13]}},ix},signed_y={{18{iy[13]}},iy};
 wire [31:0] new0=base+signed_y*(WIDTH*2)+((signed_x+(!descriptor[60]))<<1);
 wire [31:0] new1=base+(signed_y+1)*(WIDTH*2)+((signed_x+(!descriptor[62]))<<1);
 wire [31:0] le=accept?(iy[0]?new1:new0):even_address;
 wire [31:0] lo=accept?(iy[0]?new0:new1):odd_address;
 wire [INDEX_BITS+1:0] wa={fill_address[INDEX_BITS+6:7],fill_beat};
 // CLEAR and FILL share a single address/data write port. Two separate
 // procedural writes prevented RAM inference and built a large flop mux.
 localparam VALID_ADDR_BITS=(INDEX_BITS>6)?6:INDEX_BITS;
 localparam VALID_BANKS=1<<(INDEX_BITS-VALID_ADDR_BITS);
 localparam VALID_SEL_BITS=(INDEX_BITS>6)?INDEX_BITS-6:1;
 wire clearing=state==CLEAR;
 wire commit_line=(state==FILL)&&rvalid&&rlast;
 wire [INDEX_BITS-1:0] vi=clearing?clear_index:fill_address[INDEX_BITS+6:7];
 wire [INDEX_BITS-1:0] rei=le[INDEX_BITS+6:7],roi=lo[INDEX_BITS+6:7];
 reg [VALID_SEL_BITS-1:0] select_even,select_odd;
 wire [VALID_BANKS-1:0] valid_even,valid_odd;
 always @(posedge clk)if(lookup)begin
  select_even<=rei>>VALID_ADDR_BITS;select_odd<=roi>>VALID_ADDR_BITS;
 end
 assign vqe=valid_even[select_even];assign vqo=valid_odd[select_odd];
 genvar bank;
 generate for(bank=0;bank<VALID_BANKS;bank=bank+1)begin : g_valid
  (* syn_ramstyle="distributed_ram" *) reg ve[0:(1<<VALID_ADDR_BITS)-1];
  (* syn_ramstyle="distributed_ram" *) reg vo[0:(1<<VALID_ADDR_BITS)-1];
  reg even_q,odd_q;
  wire selected=(vi>>VALID_ADDR_BITS)==bank;
  always @(posedge clk)begin
   if(lookup)begin
    even_q<=ve[rei[VALID_ADDR_BITS-1:0]];
    odd_q<=vo[roi[VALID_ADDR_BITS-1:0]];
   end
   if(selected&&(clearing||(commit_line&&!fill_odd)))ve[vi[VALID_ADDR_BITS-1:0]]<=!clearing;
   if(selected&&(clearing||(commit_line&&fill_odd)))vo[vi[VALID_ADDR_BITS-1:0]]<=!clearing;
  end
  assign valid_even[bank]=even_q;assign valid_odd[bank]=odd_q;
 end endgenerate
 always @(posedge clk)begin
  if(lookup)begin
   tqe<=te[le[INDEX_BITS+6:7]];tqo<=to_[lo[INDEX_BITS+6:7]];
   qe<=de[le[INDEX_BITS+6:5]];qo<=do_[lo[INDEX_BITS+6:5]];
 end
  if(state==FILL&&rvalid)begin
   if(fill_odd)do_[wa]<=rdata;else de[wa]<=rdata;
   if(rlast)begin
    if(fill_odd)to_[fill_address[INDEX_BITS+6:7]]<=fill_address[29:7];
    else te[fill_address[INDEX_BITS+6:7]]<=fill_address[29:7];
   end
  end
 end
 wire checked=state==CHECK||state==CROSS_CHECK;
 assign out_valid=checked&&hit_e&&hit_o&&(!crossing||second);
 wire [15:0] v00=mask[0]?(second?p0:q0[x[3:0]*16+:16]):16'b0;
 wire [15:0] v01=mask[2]?(second?p2:q1[x[3:0]*16+:16]):16'b0;
 wire [3:0] nx=x[3:0]+1'b1;
 wire [15:0] v10=mask[1]?(second&&!cross0?p1:q0[nx*16+:16]):16'b0;
 wire [15:0] v11=mask[3]?(second&&!cross1?p3:q1[nx*16+:16]):16'b0;
 assign pixels={v11,v01,v10,v00};assign dx=desc[15:0];assign dy=desc[31:16];
 // Four 32-byte beats = 128 bytes = 32 HMIC words (clear FIVE word bits).
 assign arvalid=state==REQUEST;assign araddr=fill_address[29:2]&28'hfffffe0;assign arlen=3;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;second<=0;fill_odd<=0;base<=0;a0<=0;a1<=0;desc<=0;p0<=0;p1<=0;p2<=0;p3<=0;clear_index<=0;fill_address<=0;fill_beat<=0;error<=0;end
  else if(start)begin state<=CLEAR;second<=0;base<=src_base;clear_index<=0;error<=0;end
  else begin
   // Sample both neighbor pixels on every cycle instead of gating them with a
   // clock enable. The held sample is only read back while second==1, and
   // second is only set together with the crossing transition, so the enable
   // was redundant. Removing it takes the hit/address compare cone off the
   // p0..p3 enable path (the worst 125 MHz setup path).
   p0<=v00;p1<=v10;p2<=v01;p3<=v11;
   case(state)
   CLEAR:if(clear_index==LINES-1)state<=IDLE;else clear_index<=clear_index+1'b1;
   IDLE:if(accept)begin desc<=descriptor;a0<=new0;a1<=new1;second<=0;state<=CHECK;end
   CHECK,CROSS_CHECK:begin
    if(!hit_e||!hit_o)begin fill_odd<=hit_e;fill_address<=hit_e?odd_address:even_address;state<=REQUEST;end
    else if(crossing&&!second)begin second<=1;state<=CROSS_READ;end
    else if(out_ready)state<=IDLE;
   end
   CROSS_READ:state<=CROSS_CHECK;
   REQUEST:if(arready)begin fill_beat<=0;state<=FILL;end
   FILL:if(rvalid)begin
    fill_beat<=fill_beat+1'b1;
    if(rid!=0||rlast!=(fill_beat==3))error<=1;
    if(rlast)state<=RETRY;
   end
   RETRY:state<=second?CROSS_CHECK:CHECK;
   default:state<=IDLE;
  endcase
  end
 end
endmodule
