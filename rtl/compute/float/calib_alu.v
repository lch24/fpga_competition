`include "calib_defs.vh"
// Calibration engine scalar ALU. One request in flight; response holds under
// backpressure. IEEE binary64 RNE, gradual underflow, flags match fp_operator.
// Basic operations only: complex functions will be microprogram subroutines.
// Alignment, arithmetic, normalization and rounding have register boundaries.
// A 57-bit GRS path replaces the general 256-bit pack/alignment machinery.
module calib_alu #(parameter USE_CE=0) (
 input wire ce,clk,rst_n,req_valid, output wire req_ready,
 input wire [4:0] req_op, input wire [63:0] req_a,req_b,
 output reg rsp_valid, input wire rsp_ready, output reg [63:0] rsp_result,
 output reg [4:0] rsp_flags,
 output reg rsp_less,rsp_equal,rsp_unordered
);
 localparam IDLE=0,DECODE=1,ALIGN=2,ARITH=3,NORMALIZE=4,ROUND=5,DREQ=6,DWAIT=7;
 reg [3:0] state;
 reg [4:0] op;
 reg [63:0] a,b;
 reg [52:0] ma,mb;
 reg signed [12:0] ea,eb,exponent;
 reg sa,sb,sign,narrow;
 reg [56:0] va,vb,magnitude;
 wire [105:0] product=ma*mb;
 wire na=(&a[62:52]) && |a[51:0], nb=(&b[62:52]) && |b[51:0];
 wire sna=na && !a[51], snb=nb && !b[51];
 wire ia=(&a[62:52]) && !(|a[51:0]), ib=(&b[62:52]) && !(|b[51:0]);
 wire za=!(|a[62:0]), zb=!(|b[62:0]);
 wire div_ready,div_valid; wire [63:0] div_result; wire [4:0] div_flags;
 wire [5:0] target=narrow?6'd26:6'd55;
 wire signed [12:0] emin=narrow?-13'sd126:-13'sd1022;
 wire signed [12:0] emax=narrow?13'sd127:13'sd1023;
 wire [53:0] rounded={1'b0,magnitude[55:3]} +
     {{53{1'b0}},(magnitude[2] && (magnitude[1] || magnitude[0] || magnitude[3]))};
 wire carry=narrow?rounded[24]:rounded[53];
 wire [53:0] rounded_mantissa=carry?(rounded>>1):rounded;
 wire signed [12:0] final_exp=exponent+ (carry?13'sd1:13'sd0);
 wire normal=narrow?rounded_mantissa[23]:rounded_mantissa[52];
 wire [12:0] biased_exp=final_exp+(narrow?13'd127:13'd1023);
 wire lost=|magnitude[2:0];
 function [56:0] jam;
  input [56:0] x; input [12:0] distance;
  reg [56:0] q;
  begin
   if(distance==0)jam=x;
   else if(distance>=57)jam={56'd0,|x};
   else begin q=x>>distance;q[0]=q[0] || |(x<<(57-distance));jam=q;end
  end
 endfunction
 task finish;
  input [63:0] value;input [4:0] flags;
  begin rsp_result<=value;rsp_flags<=flags;rsp_valid<=1;state<=IDLE;end
 endtask
 assign req_ready=rst_n && state==IDLE && !rsp_valid;
 fp_divsqrt #(.FP_W(64),.USE_CE(USE_CE)) divider(.ce(ce),.clk(clk),.rst_n(rst_n),
  .req_valid(rst_n && state==DREQ),.req_ready(div_ready),.req_sqrt(op==`PAR_FP_SQRT),
  .req_a(a),.req_b(b),.rsp_valid(div_valid),.rsp_ready(rst_n && state==DWAIT),
  .rsp_result(div_result),.rsp_flags(div_flags));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;rsp_valid<=0;rsp_result<=0;rsp_flags<=0;
   rsp_less<=0;rsp_equal<=0;rsp_unordered<=0;
   a<=0;b<=0;op<=0;ma<=0;mb<=0;ea<=0;eb<=0;exponent<=0;
   sa<=0;sb<=0;sign<=0;narrow<=0;va<=0;vb<=0;magnitude<=0;
  end else if(!USE_CE || ce)begin
   if(rsp_valid && rsp_ready)rsp_valid<=0;
   case(state)
    IDLE:if(req_valid && req_ready)begin
     a<=req_a;b<=req_b;op<=req_op;state<=DECODE;
     rsp_less<=0;rsp_equal<=0;rsp_unordered<=0;narrow<=0;
    end
    DECODE:begin
     ma<={|a[62:52],a[51:0]};mb<={|b[62:52],b[51:0]};
     ea<=a[62:52]==0?-13'sd1022:$signed({2'b0,a[62:52]})-13'sd1023;
     eb<=b[62:52]==0?-13'sd1022:$signed({2'b0,b[62:52]})-13'sd1023;
     sa<=a[63];sb<=b[63]^(op==`PAR_FP_SUB);
     if(op==`PAR_FP_DIV || op==`PAR_FP_SQRT)state<=DREQ;
     else if(op==`PAR_FP_COMPARE)begin
      rsp_unordered<=na||nb;rsp_equal<=!(na||nb) && (a==b || (za&&zb));
      rsp_less<=!(na||nb) && !(za&&zb) && ((a[63]!=b[63])?a[63]:(a[63]?(a>b):(a<b)));
      finish(0,{4'd0,sna||snb});
     end else if(op==`PAR_FP_F32_TO_F64)begin
      if(a[30:23]==255)begin
       if(|a[22:0])finish(64'h7ff8000000000000,{4'd0,!a[22]});
       else finish({a[31],11'h7ff,52'd0},0);
      end else begin
       magnitude<={1'b0,|a[30:23],a[22:0],32'd0};sign<=a[31];
       exponent<=a[30:23]==0?-13'sd126:$signed({5'd0,a[30:23]})-13'sd127;
       state<=NORMALIZE;
      end
     end else if(op==`PAR_FP_F64_TO_F32)begin
      if(na)finish(64'h000000007fc00000,{4'd0,sna});
      else if(ia)finish({32'd0,a[63],8'hff,23'd0},0);
      else begin
       magnitude<={30'd0,|a[62:52],a[51:29],a[28:27],|a[26:0]};
       exponent<=a[62:52]==0?-13'sd1022:$signed({2'b0,a[62:52]})-13'sd1023;
       sign<=a[63];narrow<=1;state<=NORMALIZE;
      end
     end else if(op<=`PAR_FP_MUL)begin
      if(na||nb)finish(64'h7ff8000000000000,{4'd0,sna||snb});
      else if(op==`PAR_FP_MUL)begin
       if((ia&&zb)||(ib&&za))finish(64'h7ff8000000000000,5'b00001);
       else if(ia||ib)finish({a[63]^b[63],11'h7ff,52'd0},0);
       else state<=ALIGN;
      end else if(ia&&ib&&(a[63]!=(b[63]^(op==`PAR_FP_SUB))))finish(64'h7ff8000000000000,1);
      else if(ia)finish({a[63],11'h7ff,52'd0},0);
      else if(ib)finish({b[63]^(op==`PAR_FP_SUB),11'h7ff,52'd0},0);
      else state<=ALIGN;
     end else finish(64'h7ff8000000000000,1);
    end
    ALIGN:begin
     if(op==`PAR_FP_MUL)begin
      // Normalize subnormal operands BEFORE reducing the 106-bit product.
      // Otherwise a tiny operand times a huge one loses significant bits.
      if(ma!=0 && !ma[52])begin ma<=ma<<1;ea<=ea-13'sd1;end
      if(mb!=0 && !mb[52])begin mb<=mb<<1;eb<=eb-13'sd1;end
      if((ma==0 || ma[52]) && (mb==0 || mb[52]))begin
       exponent<=ea+eb+13'sd1;sign<=sa^sb;state<=ARITH;
      end
     end
     else begin
      exponent<=ea>=eb?ea:eb;
      va<=ea>=eb?{1'b0,ma,3'd0}:jam({1'b0,ma,3'd0},eb-ea);
      vb<=eb>=ea?{1'b0,mb,3'd0}:jam({1'b0,mb,3'd0},ea-eb);
      state<=ARITH;
     end
    end
    ARITH:begin
     if(op==`PAR_FP_MUL)magnitude<={1'b0,product[105:51],|product[50:0]};
     else if(sa==sb)begin magnitude<=va+vb;sign<=sa;end
     else if(va>=vb)begin magnitude<=va-vb;sign<=sa && (va!=vb || sb);end
     else begin magnitude<=vb-va;sign<=sb;end
     state<=NORMALIZE;
    end
    NORMALIZE:begin
     // Cancellation/subnormals are uncommon. A one-bit shifter saves a large
     // normalization barrel; exponent clamps bound extreme underflow latency.
     if(magnitude==0)state<=ROUND;
     else if(exponent<emin-13'sd57)begin magnitude<=1;exponent<=emin;state<=ROUND;end
     else if(exponent<emin || magnitude[target+1'b1])begin
      magnitude<={1'b0,magnitude[56:2],magnitude[1]|magnitude[0]};exponent<=exponent+13'sd1;
     end else if(!magnitude[target] && exponent>emin)begin magnitude<=magnitude<<1;exponent<=exponent-13'sd1;end
     else state<=ROUND;
    end
    ROUND:begin
     if(magnitude==0)finish(narrow?{32'd0,sign,31'd0}:{sign,63'd0},0);
     else if(final_exp>emax)finish(narrow?{32'd0,sign,8'hff,23'd0}:{sign,11'h7ff,52'd0},5'b10100);
     else if(narrow)finish({32'd0,sign,(normal?biased_exp[7:0]:8'd0),rounded_mantissa[22:0]}, {lost,lost&&!normal,3'd0});
     else finish({sign,(normal?biased_exp[10:0]:11'd0),rounded_mantissa[51:0]}, {lost,lost&&!normal,3'd0});
    end
    DREQ:if(div_ready)state<=DWAIT;
    DWAIT:if(div_valid)finish(div_result,div_flags);
    default:state<=IDLE;
   endcase
  end
 end
endmodule
