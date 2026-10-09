`timescale 1ns/1ps
// Binary32 remap service. One transaction in flight; no combinational divider.
// Staged add/sub/mul datapath and the shared portable iterative divider.
// op4 converts an unsigned 32-bit integer with round-to-nearest-even.
// Non-finite operands for op0..3 are rejected as quiet NaN, as before.
module fp32_service (
 input wire clk,rst_n,req_valid,output wire req_ready,
 input wire [2:0] req_op,input wire [31:0] req_a,req_b,
 output reg rsp_valid,input wire rsp_ready,
 output reg [31:0] rsp_result,output reg rsp_error
);
    function [31:0] pack;
        input sign;
        input integer exp;
        input [63:0] magnitude;
        reg [63:0] m,probe;
        reg [24:0] rounded;
        reg sticky;
        reg [7:0] encoded_exp;
        integer e, j, leading, distance;
        begin
            m=magnitude; e=exp;
            leading=0;
            probe=m;
            if(|probe[63:32])begin probe=probe>>32;leading=leading+32;end
            if(|probe[31:16])begin probe=probe>>16;leading=leading+16;end
            if(|probe[15:8])begin probe=probe>>8;leading=leading+8;end
            if(|probe[7:4])begin probe=probe>>4;leading=leading+4;end
            if(|probe[3:2])begin probe=probe>>2;leading=leading+2;end
            if(probe[1])leading=leading+1;
            if(m!=0 && leading>26) begin
                distance=leading-26;
                sticky=|(m & (64'hffffffffffffffff >> (64-distance)));
                m=m>>distance; m[0]=m[0]|sticky; e=e+distance;
            end else if(m!=0 && leading<26 && e> -126) begin
                distance=26-leading;
                if(distance>e+126) distance=e+126;
                m=m<<distance; e=e-distance;
            end
            if(e< -126) begin
                distance=-126-e;
                // Preserve the former bounded loop even for out-of-domain e.
                if(distance>256) distance=256;
                if(distance>=64) m={63'd0,|m};
                else begin
                    sticky=|(m & (64'hffffffffffffffff >> (64-distance)));
                    m=m>>distance; m[0]=m[0]|sticky;
                end
                e=e+distance;
            end
            rounded={1'b0,m[26:3]};
            if(m[2] && (m[1] || m[0] || rounded[0])) rounded=rounded+25'd1;
            if(rounded[24]) begin rounded=rounded>>1; e=e+1; end
            if(e>127) pack={sign,8'hff,23'd0};
            else if(rounded==0) pack={sign,31'd0};
            else if(e== -126 && !rounded[23]) pack={sign,8'd0,rounded[22:0]};
            else begin encoded_exp=e+127; pack={sign,encoded_exp,rounded[22:0]}; end
        end
    endfunction
 // Decode, align, arithmetic and rounding are separate enabled cycles.
 // All products retain 48 bits until the final pack; subnormals are preserved.
 localparam IDLE=0,DECODE=1,ALIGN=2,ARITHMETIC=3,PACK=4,DIV_REQ=5,DIV_WAIT=6;
 reg [2:0] state,op;
 reg [31:0] a,b;
 reg [23:0] ma,mb;
 reg signed [11:0] ea,eb,exponent;
 reg sa,sb,sign;
 reg [63:0] va,vb,magnitude;
 wire ready,valid;wire [31:0] result;
 wire [47:0] product=ma*mb;
 wire [31:0] packed_result=pack(sign,{{20{exponent[11]}},exponent},magnitude);
 function [63:0] shift_jam;
  input [63:0] value;input [11:0] distance;
  reg [63:0] shifted;
  begin
   if(distance==0)shift_jam=value;
   else if(distance>=64)shift_jam={63'd0,|value};
   else begin shifted=value>>distance;shifted[0]=shifted[0] || |(value<<(64-distance));shift_jam=shifted;end
  end
 endfunction
 assign req_ready=rst_n && state==IDLE && !rsp_valid;
 fp_divsqrt #(.FP_W(32)) divider(.ce(1'b1),
  .clk(clk),.rst_n(rst_n),.req_valid(state==DIV_REQ),.req_ready(ready),
  .req_sqrt(1'b0),.req_a(a),.req_b(b),
  .rsp_valid(valid),.rsp_ready(state==DIV_WAIT),.rsp_result(result),.rsp_flags());
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   state<=IDLE;rsp_valid<=0;rsp_result<=0;rsp_error<=0;op<=0;a<=0;b<=0;
   ma<=0;mb<=0;ea<=0;eb<=0;exponent<=0;sa<=0;sb<=0;sign<=0;va<=0;vb<=0;magnitude<=0;
  end else begin
   if(rsp_valid && rsp_ready)rsp_valid<=0;
   case(state)
    IDLE:if(req_valid && req_ready)begin
     a<=req_a;b<=req_b;op<=req_op;
     if(req_op>4 || (req_op!=4 && (req_a[30:23]==255 || req_b[30:23]==255)))begin
      rsp_valid<=1;rsp_result<=32'h7fc00000;rsp_error<=1;
     end else state<=req_op==3?DIV_REQ:DECODE;
    end
    DECODE:begin
     ma<={a[30:23]!=0,a[22:0]};mb<={b[30:23]!=0,b[22:0]};
     ea<=a[30:23]==0?-12'sd126:$signed({4'd0,a[30:23]})-12'sd127;
     eb<=b[30:23]==0?-12'sd126:$signed({4'd0,b[30:23]})-12'sd127;
     sa<=a[31];sb<=b[31]^(op==1);
     if(op==4)begin magnitude<={32'd0,a};sign<=0;exponent<=26;state<=PACK;end
     else state<=ALIGN;
    end
    ALIGN:begin
     if(op==2)begin exponent<=ea+eb-12'sd20;sign<=sa^sb;end
     else begin
      exponent<=ea>=eb?ea:eb;
      va<=ea>=eb?{37'd0,ma,3'd0}:shift_jam({37'd0,ma,3'd0},eb-ea);
      vb<=eb>=ea?{37'd0,mb,3'd0}:shift_jam({37'd0,mb,3'd0},ea-eb);
     end
     state<=ARITHMETIC;
    end
    ARITHMETIC:begin
     if(op==2)magnitude<={16'd0,product};
     else if(sa==sb)begin magnitude<=va+vb;sign<=sa;end
     else if(va>vb)begin magnitude<=va-vb;sign<=sa;end
     else if(vb>va)begin magnitude<=vb-va;sign<=sb;end
     else begin magnitude<=0;sign<=0;end
     state<=PACK;
    end
    PACK:begin rsp_valid<=1;rsp_result<=packed_result;rsp_error<=packed_result[30:23]==255;state<=IDLE;end
    DIV_REQ:if(ready)state<=DIV_WAIT;
    DIV_WAIT:if(valid)begin rsp_valid<=1;rsp_result<=result;rsp_error<=result[30:23]==255;state<=IDLE;end
   endcase
  end
 end
endmodule
