`timescale 1ns/1ps
// RNE IEEE-754 adder. Narrow guard/round/sticky arithmetic, throughput one
// request per enabled clock. Default keeps the legacy two-clock latency.
// CE freezes all stages and FIFO; reset cancels queued/in-flight operations.
module fp32_add #(parameter USE_CE=0)(
 input wire ce,clk,rst_n,in_valid, output wire in_ready,
 input wire [31:0] in_a,in_b, output wire out_valid,
 input wire out_ready, output wire [31:0] out_r
);
 fp_add_pipeline #(.BITS(32),.USE_CE(USE_CE)) core(.ce(ce),.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),.in_a(in_a),.in_b(in_b),.out_valid(out_valid),.out_ready(out_ready),.out_r(out_r));
endmodule

// Shared implementation, instantiated separately for each physical adder.
// Three low bits suffice: sticky is jammed BEFORE add/sub, so subtraction
// retains the direction of discarded information. No precision reduction.
module fp_add_pipeline #(parameter BITS=32, USE_CE=0, PIPELINED=0)(
 input wire ce,clk,rst_n,in_valid, output wire in_ready,
 input wire [BITS-1:0] in_a,in_b, output wire out_valid,
 input wire out_ready, output wire [BITS-1:0] out_r
);
 localparam F=(BITS==64)?52:23, E=BITS-F-1, W=F+4;
 localparam [E-1:0] EMAX={E{1'b1}};
 wire [E-1:0] ea=in_a[BITS-2:F], eb=in_b[BITS-2:F];
 wire [E-1:0] ax=(ea==0)?1:ea, bx=(eb==0)?1:eb;
 wire [F:0] am={ea!=0,in_a[F-1:0]}, bm={eb!=0,in_b[F-1:0]};
 wire big_a=(ax>bx)||((ax==bx)&&(am>=bm));
 wire nan_a=(ea==EMAX)&&(|in_a[F-1:0]);
 wire nan_b=(eb==EMAX)&&(|in_b[F-1:0]);
 wire inf_a=(ea==EMAX)&&!(|in_a[F-1:0]);
 wire inf_b=(eb==EMAX)&&!(|in_b[F-1:0]);
 wire special=nan_a||nan_b||inf_a||inf_b;
 wire invalid_inf=inf_a&&inf_b&&(in_a[BITS-1]!=in_b[BITS-1]);
 wire [BITS-1:0] special_value=(nan_a||nan_b||invalid_inf)?
     {1'b0,EMAX,1'b1,{(F-1){1'b0}}}:
     {inf_a?in_a[BITS-1]:in_b[BITS-1],EMAX,{F{1'b0}}};
 wire [5:0] count;
 // Four arithmetic stages + pending FIFO push; leave room for all of them.
 assign in_ready=(count<26);
 reg [3:0] valid;
 reg [W-1:0] large0,small0,large1,small1;
 reg [E-1:0] diff0,exp0,exp1;
 reg sign0,sign1,sign2,same0,same1,zero_sign0,zero_sign1,zero_sign2;
 reg special0,special1,special2;
 reg [BITS-1:0] value0,value1,value2,result3;
 reg [W:0] sum2;
 reg [E:0] exp2;
 // Logarithmic shift-right-jam network: no variable-width masks/subtractions.
 function [W-1:0] jam;
   input [W-1:0] x; input [E-1:0] amount;
   reg [W-1:0] t; integer k;
   begin
     t=x;
     if(amount>=W)t={{(W-1){1'b0}},|x};
     else for(k=0;k<$clog2(W);k=k+1)
       if(amount[k])t=(t>>(1<<k))|((|(t << (W-(1<<k))))?{{(W-1){1'b0}},1'b1}:{W{1'b0}});
     jam=t;
   end
 endfunction
 reg [W-1:0] normalized;
 reg [E:0] normalized_exp;
 reg [F+1:0] rounded;
 reg [BITS-1:0] packed_result;
 integer k;
 always @* begin
   normalized_exp=exp2;
   if(sum2[W])begin
     normalized=sum2[W:1];normalized[0]=sum2[1]|sum2[0];
     normalized_exp=exp2+1'b1;
   end else begin
     normalized=sum2[W-1:0];
     // Stop at exponent 1: gradual underflow keeps the subnormal scale.
     for(k=$clog2(W)-1;k>=0;k=k-1)
       if((normalized>>(W-(1<<k)))==0 && normalized_exp>(1<<k))begin
         normalized=normalized<<(1<<k);normalized_exp=normalized_exp-(1<<k);
       end
   end
   rounded={1'b0,normalized[W-1:3]}+
       (normalized[2]&&(normalized[1]||normalized[0]||normalized[3]));
   if(rounded[F+1])begin rounded=rounded>>1;normalized_exp=normalized_exp+1'b1;end
   if(normalized_exp>=EMAX)packed_result={sign2,EMAX,{F{1'b0}}};
   else if(rounded[F])packed_result={sign2,normalized_exp[E-1:0],rounded[F-1:0]};
   else packed_result={sign2,{E{1'b0}},rounded[F-1:0]};
   if(sum2==0)packed_result={zero_sign2,{(BITS-1){1'b0}}};
   if(special2)packed_result=value2;
 end
 generate if(PIPELINED)begin : four_stage
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)valid<=0;
   else if(!USE_CE||ce)begin
     valid<={valid[2:0],in_valid&&in_ready};
     if(in_valid&&in_ready)begin
       large0<={big_a?am:bm,3'b0};small0<={big_a?bm:am,3'b0};
       exp0<=big_a?ax:bx;diff0<=big_a?(ax-bx):(bx-ax);
       sign0<=big_a?in_a[BITS-1]:in_b[BITS-1];
       same0<=in_a[BITS-1]==in_b[BITS-1];
       zero_sign0<=in_a[BITS-1]&&in_b[BITS-1];
       special0<=special;value0<=special_value;
     end
     if(valid[0])begin
       large1<=large0;small1<=jam(small0,diff0);exp1<=exp0;
       sign1<=sign0;same1<=same0;zero_sign1<=zero_sign0;
       special1<=special0;value1<=value0;
     end
     if(valid[1])begin
       sum2<=same1?({1'b0,large1}+{1'b0,small1}):({1'b0,large1}-{1'b0,small1});
       exp2<={1'b0,exp1};sign2<=sign1;zero_sign2<=zero_sign1;
       special2<=special1;value2<=value1;
     end
     if(valid[2])result3<=packed_result;
   end
 end
 end else begin : legacy_latency
 // Keep the two-clock streaming latency for legacy joins that consume
 // results in lockstep. The narrow arithmetic is identical in both modes.
 always @* begin
   large1=large0;small1=jam(small0,diff0);exp1=exp0;
   sign1=sign0;same1=same0;zero_sign1=zero_sign0;
   special1=special0;value1=value0;
   sum2=same1?({1'b0,large1}+{1'b0,small1}):({1'b0,large1}-{1'b0,small1});
   exp2={1'b0,exp1};sign2=sign1;zero_sign2=zero_sign1;
   special2=special1;value2=value1;result3=packed_result;
 end
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)valid<=0;
   else if(!USE_CE||ce)begin
     valid<={in_valid&&in_ready,3'b0};
     if(in_valid&&in_ready)begin
       large0<={big_a?am:bm,3'b0};small0<={big_a?bm:am,3'b0};
       exp0<=big_a?ax:bx;diff0<=big_a?(ax-bx):(bx-ax);
       sign0<=big_a?in_a[BITS-1]:in_b[BITS-1];
       same0<=in_a[BITS-1]==in_b[BITS-1];
       zero_sign0<=in_a[BITS-1]&&in_b[BITS-1];
       special0<=special;value0<=special_value;
     end
   end
 end
 end endgenerate
 sync_fifo #(.USE_CE(USE_CE),.DATA_WIDTH(BITS),.ADDR_WIDTH(5)) queue(
   .ce(ce),.clk(clk),.rst_n(rst_n),.in_valid(valid[3]),.in_ready(),
   .in_data(result3),.out_valid(out_valid),.out_ready(out_ready),
   .out_data(out_r),.count(count),.empty(),.full());
endmodule
