`timescale 1ns / 1ps
//==============================================================================
// subpixel_accum.v — M5 亚像素：高斯加权梯度五项 FP64 累加核
//------------------------------------------------------------------------------
// 语义（位级权威 tests/rtl/export_m5.cpp::export_acc，逐行一致）：
//   窗口样本按 y 外循环、x 内循环逐项喂入（顺序即 m5_acc.bin 段内顺序）。
//     xx = w*gx*gx   xy = w*gx*gy   yy = w*gy*gy   （左结合；共享 w*gx、w*gy）
//     a += xx;  b += xy;  c += yy;
//     bx += xx*x + xy*y   （t1=xx*x; t2=xy*y; t3=t1+t2; bx+=t3，先乘后加再累加）
//     by += xy*x + yy*y   （同构）
//   x/y 为窗口偏移整数，以 fp64 位模式直通（不做转换）。
//
// 数据通路：3×fp64_mul + 2×fp64_add；各结果保持到本阶段全部完成才接收。
//   逐样本处理：每个处理阶段同拍发出，允许不同周期完成；五个累加器
//   有反馈依赖（文档规划 §6.2 首版语义：等加法写回再消费下一样本）。
//
// 接口：start/busy/done；start 清零五项并锁存 n_win；SAMPLE_WAIT 态 in_ready=1
//   逐样本握手；收齐 n_win 项后 out_valid 一次给出 {a,b,c,bx,by}，out_ready 应答
//   后 done 单拍脉冲、busy 拉低。
//==============================================================================
module subpixel_accum #(parameter USE_CE=0,
    parameter N_ADDR_W = 8        // 预留：窗口样本寻址位宽（subpixel_ctrl 可选）
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,

    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         busy,
    output reg         done,
    input  wire [15:0] n_win,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_x,
    input  wire [63:0] in_y,
    input  wire [63:0] in_w,
    input  wire [63:0] in_gx,
    input  wire [63:0] in_gy,
    output reg         out_valid,
    input  wire        out_ready,
    output wire [63:0] out_a,
    output wire [63:0] out_b,
    output wire [63:0] out_c,
    output wire [63:0] out_bx,
    output wire [63:0] out_by
);

    localparam ST_IDLE = 4'd0;
    localparam ST_WAIT = 4'd1;   // 等样本（in_valid&&in_ready 接受）
    localparam ST_WG   = 4'd2;   // w*gx -> m1；w*gy -> m2
    localparam ST_OP   = 4'd3;   // m1*gx -> xx；m1*gy -> xy；m2*gy -> yy
    localparam ST_ACC  = 4'd4;   // a+=xx；b+=xy；t1=xx*x；t2=xy*y；t4=xy*x
    localparam ST_C    = 4'd5;   // c+=yy；t5=yy*y；t3=t1+t2
    localparam ST_BX   = 4'd6;   // t6=t4+t5；bx+=t3
    localparam ST_BY   = 4'd7;   // by+=t6
    localparam ST_OUT  = 4'd8;   // 输出 5 项，等 out_ready

    reg  [3:0]  state;
    reg  [3:0]  pstate;
    reg  [15:0] n_win_r;
    reg  [15:0] samples_done;

    reg  [63:0] in_x_r, in_y_r, in_w_r, in_gx_r, in_gy_r;
    reg  [63:0] m1, m2;                 // w*gx, w*gy
    reg  [63:0] xx, xy, yy;
    reg  [63:0] t1, t2, t3, t4, t5, t6;
    reg  [63:0] acc_a, acc_b, acc_c, acc_bx, acc_by;

    assign in_ready = (state == ST_WAIT);
    assign out_a  = acc_a;
    assign out_b  = acc_b;
    assign out_c  = acc_c;
    assign out_bx = acc_bx;
    assign out_by = acc_by;

    // 状态刚进入的那一拍（entry=1）：算术块只在该拍发出，避免整阶段重复发出
    wire entry = (state != pstate);

    //----------------------------------------------------------------------
    // 算术块操作数选择（每阶段各自的乘/加）
    //----------------------------------------------------------------------
    wire [63:0] mu1_a = (state == ST_WG)  ? in_w_r  :
                        (state == ST_OP)  ? m1      :
                        (state == ST_ACC) ? xx      :
                        (state == ST_C)   ? yy      : 64'd0;
    wire [63:0] mu1_b = (state == ST_WG)  ? in_gx_r :
                        (state == ST_OP)  ? in_gx_r :
                        (state == ST_ACC) ? in_x_r  :
                        (state == ST_C)   ? in_y_r  : 64'd0;

    wire [63:0] mu2_a = (state == ST_WG)  ? in_w_r  :
                        (state == ST_OP)  ? m1      :
                        (state == ST_ACC) ? xy      : 64'd0;
    wire [63:0] mu2_b = (state == ST_WG)  ? in_gy_r :
                        (state == ST_OP)  ? in_gy_r :
                        (state == ST_ACC) ? in_y_r  : 64'd0;

    wire [63:0] mu3_a = (state == ST_OP)  ? m2      :
                        (state == ST_ACC) ? xy      : 64'd0;
    wire [63:0] mu3_b = (state == ST_OP)  ? in_gy_r :
                        (state == ST_ACC) ? in_x_r  : 64'd0;

    wire [63:0] ad1_a = (state == ST_ACC) ? acc_a  :
                        (state == ST_C)   ? acc_c  :
                        (state == ST_BX)  ? acc_bx :
                        (state == ST_BY)  ? acc_by : 64'd0;
    wire [63:0] ad1_b = (state == ST_ACC) ? xx :
                        (state == ST_C)   ? yy :
                        (state == ST_BX)  ? t3 :
                        (state == ST_BY)  ? t6 : 64'd0;

    wire [63:0] ad2_a = (state == ST_ACC) ? acc_b :
                        (state == ST_C)   ? t1    :
                        (state == ST_BX)  ? t4    : 64'd0;
    wire [63:0] ad2_b = (state == ST_ACC) ? xy :
                        (state == ST_C)   ? t2 :
                        (state == ST_BX)  ? t5 : 64'd0;

    // 发出拍：仅状态刚进入时 1 拍
    wire mu1_iv = entry && (state == ST_WG || state == ST_OP || state == ST_ACC || state == ST_C);
    wire mu2_iv = entry && (state == ST_WG || state == ST_OP || state == ST_ACC);
    wire mu3_iv = entry && (state == ST_OP  || state == ST_ACC);
    wire ad1_iv = entry && (state == ST_ACC || state == ST_C || state == ST_BX || state == ST_BY);
    wire ad2_iv = entry && (state == ST_ACC || state == ST_C || state == ST_BX);

    wire        mu1_v, mu2_v, mu3_v, ad1_v, ad2_v;
    wire [63:0] mu1_r, mu2_r, mu3_r, ad1_r, ad2_r;

    // Consume a stage atomically: zero multiplies and additions may finish
    // earlier than the iterative multiplier. Hold every result until all arrive.
    wire stage_done = (state == ST_WG) ? (mu1_v && mu2_v) :
                      (state == ST_OP) ? (mu1_v && mu2_v && mu3_v) :
                      (state == ST_ACC) ? (ad1_v && ad2_v && mu1_v && mu2_v && mu3_v) :
                      (state == ST_C) ? (ad1_v && mu1_v && ad2_v) :
                      (state == ST_BX) ? (ad1_v && ad2_v) :
                      (state == ST_BY) ? ad1_v : 1'b0;

    fp64_mul #(.USE_CE(USE_CE)) u_mu1 (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu1_iv),
        .in_ready (),
        .in_a     (mu1_a),
        .in_b     (mu1_b),
        .out_valid(mu1_v),
        .out_ready(stage_done),
        .out_r    (mu1_r)
    );
    fp64_mul #(.USE_CE(USE_CE)) u_mu2 (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu2_iv),
        .in_ready (),
        .in_a     (mu2_a),
        .in_b     (mu2_b),
        .out_valid(mu2_v),
        .out_ready(stage_done),
        .out_r    (mu2_r)
    );
    fp64_mul #(.USE_CE(USE_CE)) u_mu3 (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (mu3_iv),
        .in_ready (),
        .in_a     (mu3_a),
        .in_b     (mu3_b),
        .out_valid(mu3_v),
        .out_ready(stage_done),
        .out_r    (mu3_r)
    );
    fp64_add #(.USE_CE(USE_CE)) u_ad1 (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (ad1_iv),
        .in_ready (),
        .in_a     (ad1_a),
        .in_b     (ad1_b),
        .out_valid(ad1_v),
        .out_ready(stage_done),
        .out_r    (ad1_r)
    );
    fp64_add #(.USE_CE(USE_CE)) u_ad2 (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (ad2_iv),
        .in_ready (),
        .in_a     (ad2_a),
        .in_b     (ad2_b),
        .out_valid(ad2_v),
        .out_ready(stage_done),
        .out_r    (ad2_r)
    );

    //----------------------------------------------------------------------
    // 主 FSM
    //----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            pstate       <= ST_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            out_valid    <= 1'b0;
            n_win_r      <= 16'd0;
            samples_done <= 16'd0;
            in_x_r       <= 64'd0;
            in_y_r       <= 64'd0;
            in_w_r       <= 64'd0;
            in_gx_r      <= 64'd0;
            in_gy_r      <= 64'd0;
            m1           <= 64'd0;
            m2           <= 64'd0;
            xx           <= 64'd0;
            xy           <= 64'd0;
            yy           <= 64'd0;
            t1           <= 64'd0;
            t2           <= 64'd0;
            t3           <= 64'd0;
            t4           <= 64'd0;
            t5           <= 64'd0;
            t6           <= 64'd0;
            acc_a        <= 64'd0;
            acc_b        <= 64'd0;
            acc_c        <= 64'd0;
            acc_bx       <= 64'd0;
            acc_by       <= 64'd0;
        end else if(!USE_CE || ce) begin begin
            done   <= 1'b0;              // done 单拍脉冲
            pstate <= state;
            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy      <= 1'b1;
                        n_win_r   <= n_win;
                        samples_done <= 16'd0;
                        acc_a     <= 64'd0;
                        acc_b     <= 64'd0;
                        acc_c     <= 64'd0;
                        acc_bx    <= 64'd0;
                        acc_by    <= 64'd0;
                        if (n_win == 16'd0) begin
                            out_valid <= 1'b1;
                            state     <= ST_OUT;
                        end else
                            state <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (in_valid && in_ready) begin
                        in_x_r  <= in_x;
                        in_y_r  <= in_y;
                        in_w_r  <= in_w;
                        in_gx_r <= in_gx;
                        in_gy_r <= in_gy;
                        state   <= ST_WG;
                    end
                end

                ST_WG: begin
                    if (mu1_v && mu2_v) begin
                        m1    <= mu1_r;
                        m2    <= mu2_r;
                        state <= ST_OP;
                    end
                end

                ST_OP: begin
                    if (mu1_v && mu2_v && mu3_v) begin
                        xx    <= mu1_r;
                        xy    <= mu2_r;
                        yy    <= mu3_r;
                        state <= ST_ACC;
                    end
                end

                ST_ACC: begin
                    if (ad1_v && ad2_v && mu1_v && mu2_v && mu3_v) begin
                        acc_a <= ad1_r;
                        acc_b <= ad2_r;
                        t1    <= mu1_r;
                        t2    <= mu2_r;
                        t4    <= mu3_r;
                        state <= ST_C;
                    end
                end

                ST_C: begin
                    if (ad1_v && mu1_v && ad2_v) begin
                        acc_c <= ad1_r;
                        t5    <= mu1_r;
                        t3    <= ad2_r;
                        state <= ST_BX;
                    end
                end

                ST_BX: begin
                    if (ad2_v && ad1_v) begin
                        t6     <= ad2_r;
                        acc_bx <= ad1_r;
                        state  <= ST_BY;
                    end
                end

                ST_BY: begin
                    if (ad1_v) begin
                        acc_by       <= ad1_r;
                        samples_done <= samples_done + 16'd1;
                        if (samples_done + 16'd1 == n_win_r) begin
                            out_valid <= 1'b1;
                            state     <= ST_OUT;
                        end else
                            state <= ST_WAIT;
                    end
                end

                ST_OUT: begin
                    if (out_valid && out_ready) begin
                        done      <= 1'b1;
                        busy      <= 1'b0;
                        out_valid <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end // synchronous clock enable
    end

endmodule

// Optional fixed-point backend for byte-image subpixel windows.
// Contract: finite |gx|,|gy|<=255, 0<=w<=1, integer |x|,|y|<=15,
// n_win<=961. Gradients use signed Q16, weights unsigned Q24, all five
// accumulators signed Q24 in 64 bits. One 50x26 multiplier is reused.
// Weighted products truncate toward -infinity after the final multiplication;
// no intermediate gradient product rounding. Outputs convert to FP64 RNE.
// It approximates the floating algorithm; enable only with numerical checks.
module subpixel_accum_fixed #(parameter USE_CE=0, N_ADDR_W=8)(
 input wire ce,clk,rst_n,start,output reg busy,done,
 input wire [15:0] n_win,input wire in_valid,output wire in_ready,
 input wire [63:0] in_x,in_y,in_w,in_gx,in_gy,
 output wire out_valid,input wire out_ready,
 output wire [63:0] out_a,out_b,out_c,out_bx,out_by
);
 localparam IDLE=0,WAIT_SAMPLE=1,PRODUCT=2,WEIGHT=3,WEIGHTED=4,
            ADD_TENSOR=5,OFFSET_X=6,ADD_X=7,OFFSET_Y=8,ADD_Y=9,
            NEXT_TERM=10,PACK=11,OUTPUT=12;
 reg [3:0] state;
 reg [1:0] term;
 reg [2:0] pack_index;
 reg [15:0] count,total;
 reg signed [24:0] gx,gy;
 reg signed [25:0] weight;
 reg signed [5:0] x,y;
 reg signed [49:0] mul_a;
 reg signed [25:0] mul_b;
 reg signed [75:0] product;
 reg signed [63:0] weighted;
 reg signed [63:0] acc[0:4];
 reg [63:0] packed_values[0:4];
 integer i;
 assign in_ready=rst_n && state==WAIT_SAMPLE;
 assign out_valid=rst_n && state==OUTPUT;
 assign out_a=packed_values[0];assign out_b=packed_values[1];assign out_c=packed_values[2];
 assign out_bx=packed_values[3];assign out_by=packed_values[4];
 function signed [63:0] fixed_value;
  input [63:0] v;input integer frac;
  reg [63:0] mag;integer shift;
  begin
   mag={11'd0,(v[62:52]!=0),v[51:0]};
   shift=$signed({1'b0,v[62:52]})-1023-52+frac;
   if(v[62:52]==0)mag=0;
   else if(shift<0)mag=mag>>(-shift);else mag=mag<<shift;
   fixed_value=v[63]?-$signed(mag):$signed(mag);
  end
 endfunction
 function [63:0] pack_q24;
  input signed [63:0] v;
  reg [63:0] mag,probe,shifted;
  reg [53:0] mant;
  reg [10:0] exponent;
  integer n,k,drop;
  reg sticky,guard_bit;
  begin
   mag=v[63]?-v:v;probe=mag;n=0;
   for(k=5;k>=0;k=k-1)if(probe>>(1<<k))begin probe=probe>>(1<<k);n=n+(1<<k);end
   drop=n-52;shifted=0;guard_bit=0;sticky=0;
   if(drop>0)begin
    shifted=mag>>drop;guard_bit=mag[drop-1];
    for(k=0;k<11;k=k+1)if(k<drop-1)sticky=sticky|mag[k];
   end else shifted=mag<<(-drop);
   mant={1'b0,shifted[52:0]}+(guard_bit&&(sticky||shifted[0]));
   exponent=1023+n-24;
   if(mant[53])begin mant=mant>>1;exponent=exponent+1'b1;end
   pack_q24=(mag==0)?64'd0:{v[63],exponent,mant[51:0]};
  end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;busy<=0;done<=0;count<=0;total<=0;term<=0;pack_index<=0;end
  else if(!USE_CE||ce)begin
   done<=0;
   case(state)
    IDLE:if(start)begin
     busy<=1;total<=n_win;count<=0;for(i=0;i<5;i=i+1)acc[i]<=0;
     pack_index<=0;state<=(n_win==0)?PACK:WAIT_SAMPLE;
    end
    WAIT_SAMPLE:if(in_valid)begin
     gx<=fixed_value(in_gx,16);gy<=fixed_value(in_gy,16);weight<=fixed_value(in_w,24);
     x<=fixed_value(in_x,0);y<=fixed_value(in_y,0);
     mul_a<=fixed_value(in_gx,16);mul_b<=fixed_value(in_gx,16);term<=0;state<=PRODUCT;
    end
    PRODUCT:begin product<=mul_a*mul_b;state<=WEIGHT;end
    WEIGHT:begin mul_a<=product[49:0];mul_b<=weight;state<=WEIGHTED;end
    WEIGHTED:begin product<=mul_a*mul_b;state<=ADD_TENSOR;end
    ADD_TENSOR:begin
     weighted<=product>>>32;acc[term]<=acc[term]+(product>>>32);
     mul_a<=product>>>32;mul_b<=x;state<=OFFSET_X;
    end
    OFFSET_X:begin product<=mul_a*mul_b;state<=ADD_X;end
    ADD_X:begin
     if(term==0)acc[3]<=acc[3]+product;
     if(term==1)acc[4]<=acc[4]+product;
     mul_a<=weighted;mul_b<=y;state<=OFFSET_Y;
    end
    OFFSET_Y:begin product<=mul_a*mul_b;state<=ADD_Y;end
    ADD_Y:begin
     if(term==1)acc[3]<=acc[3]+product;
     if(term==2)acc[4]<=acc[4]+product;
     state<=NEXT_TERM;
    end
    NEXT_TERM:begin
     if(term==2)begin
      if(count+1==total)begin pack_index<=0;state<=PACK;end
      else begin count<=count+1'b1;state<=WAIT_SAMPLE;end
     end else begin
      term<=term+1'b1;mul_a<=(term==0)?gx:gy;mul_b<=gy;state<=PRODUCT;
     end
    end
    PACK:begin packed_values[pack_index]<=pack_q24(acc[pack_index]);
     if(pack_index==4)state<=OUTPUT;else pack_index<=pack_index+1'b1;end
    OUTPUT:if(out_ready)begin busy<=0;done<=1;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
