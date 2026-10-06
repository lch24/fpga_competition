// 可综合浮点位运算函数。仅使用整数/位操作，无real、DPI或仿真数学函数。
// 统一返回[68:64]=异常标志，[63:0]=结果；FP32结果在低32位。
// pack按最近偶数舍入，保留非规格化数；underflow按舍入后微小且不精确判定。
function [68:0] fp_pack;
    input sign;
    input [255:0] magnitude;
    input integer lsb_exp;
    input tail;
    input integer width;
    integer f, bias, emin, emax, lead, e, shift, i, packed_exp;
    reg [63:0] q;
    reg [255:0] lead_work;
    reg guard_bit, lower, lost;
    reg [63:0] bits, sign_bit, frac_mask;
    reg [4:0] flags;
    begin
        f = (width == 32) ? 23 : 52;
        bias = (width == 32) ? 127 : 1023;
        emin = 1-bias; emax = bias;
        sign_bit = sign ? (64'd1 << (width-1)) : 64'd0;
        frac_mask = (64'd1 << f)-1;
        flags=0; bits=sign_bit; lead=-1;
        e=0; shift=0; packed_exp=0; q=0; guard_bit=0; lower=0; lost=0;
        // Binary leading-bit encoder instead of a 256-stage priority chain.
        lead_work=magnitude; lead=0;
        if (|lead_work[255:128]) begin lead_work=lead_work>>128; lead=lead+128; end
        if (|lead_work[127:64]) begin lead_work=lead_work>>64; lead=lead+64; end
        if (|lead_work[63:32]) begin lead_work=lead_work>>32; lead=lead+32; end
        if (|lead_work[31:16]) begin lead_work=lead_work>>16; lead=lead+16; end
        if (|lead_work[15:8]) begin lead_work=lead_work>>8; lead=lead+8; end
        if (|lead_work[7:4]) begin lead_work=lead_work>>4; lead=lead+4; end
        if (|lead_work[3:2]) begin lead_work=lead_work>>2; lead=lead+2; end
        if (lead_work[1]) lead=lead+1;
        if (magnitude==0) lead=-1;
        if (lead >= 0) begin
            e=lead+lsb_exp;
            shift=(e<emin) ? (emin-f-lsb_exp) : (lead-f);
            if (e<emin) e=emin;
            guard_bit=0; lower=tail; q=0;
            if (shift>0) begin
                if (shift<=256) guard_bit=((magnitude >> (shift-1)) & 1'b1);
                if (shift>256) lower=lower|(|magnitude);
                else if (shift>1) lower=lower|(|(magnitude & ({256{1'b1}} >> (257-shift))));
                if (shift<256) q=magnitude >> shift;
            end else q=magnitude << (-shift);
            lost=guard_bit|lower;
            if (guard_bit && (lower || q[0])) q=q+1'b1;
            if (q[f+1]) begin q=q>>1; e=e+1; end
            flags[4]=lost;
            if (e>emax) begin
                bits=sign_bit | (((64'd1 << (width-f-1))-1) << f);
                flags[2]=1; flags[4]=1;
            end else if (!q[f]) begin
                bits=sign_bit | (q[63:0]&frac_mask);
                flags[3]=lost;
            end else begin
                packed_exp=e+bias;
                bits=sign_bit | ({32'd0,packed_exp[31:0]} << f) | (q[63:0]&frac_mask);
            end
        end else if (tail) begin flags[3]=1; flags[4]=1; end
        fp_pack={flags,bits};
    end
endfunction

function [255:0] fp_shr_jam;
    input [255:0] value;
    input integer shift;
    integer i;
    reg sticky;
    reg [255:0] q;
    begin
        sticky=0;
        if (shift<=0) q=value;
        else begin
            q=(shift<256) ? (value>>shift) : 256'd0;
            if (shift>=256) sticky=|value;
            else sticky=|(value & ({256{1'b1}} >> (256-shift)));
            q[0]=q[0]|sticky;
        end
        fp_shr_jam=q;
    end
endfunction

function [68:0] fp_eval;
    input [4:0] op;
    input [63:0] a, b;
    input integer width;
    integer f, bias, emin, ea, eb, xa, xb, common_exp;
    reg sa, sb, na, nb, sna, snb, ia, ib, za, zb, result_sign;
    reg [63:0] fracmask, expmask, signmask, qa, qb, ma, mb, inf, nan;
    reg [255:0] aa, bb, mag;
    reg [68:0] answer;
    reg [4:0] flags;
    reg pack_enable;
    integer pack_exp;
    begin
        f=(width==32)?23:52; bias=(width==32)?127:1023; emin=1-bias;
        fracmask=(64'd1<<f)-1; expmask=(64'd1<<(width-f-1))-1;
        signmask=64'd1<<(width-1);
        sa=(a&signmask)!=0; sb=(b&signmask)!=0;
        xa=(a>>f)&expmask; xb=(b>>f)&expmask;
        qa=a&fracmask; qb=b&fracmask;
        na=(xa==expmask)&&(qa!=0); nb=(xb==expmask)&&(qb!=0);
        sna=na && !qa[f-1]; snb=nb && !qb[f-1];
        ia=(xa==expmask)&&(qa==0); ib=(xb==expmask)&&(qb==0);
        za=(xa==0)&&(qa==0); zb=(xb==0)&&(qb==0);
        ma=qa; mb=qb;
        if (xa!=0) ma[f]=1;
        if (xb!=0) mb[f]=1;
        ea=((xa==0)?emin:(xa-bias))-f;
        eb=((xb==0)?emin:(xb-bias))-f;
        inf=expmask<<f; nan=inf|(64'd1<<(f-1)); flags=0;
        answer=0; aa=0; bb=0; mag=0; result_sign=0; common_exp=0; pack_enable=0; pack_exp=0;
        case (op)
        `PAR_FP_ADD, `PAR_FP_SUB: begin
            if (op==`PAR_FP_SUB) sb=~sb;
            if (na||nb) answer={4'd0,(sna||snb),nan};
            else if (ia&&ib&&(sa!=sb)) answer={5'b00001,nan};
            else if (ia) answer={5'd0,((sa?signmask:64'd0)|inf)};
            else if (ib) answer={5'd0,((sb?signmask:64'd0)|inf)};
            else begin
                common_exp=(ea>eb)?ea:eb;
                aa=ma; aa=fp_shr_jam(aa<<3,common_exp-ea);
                bb=mb; bb=fp_shr_jam(bb<<3,common_exp-eb);
                if (sa==sb) begin mag=aa+bb; result_sign=sa; end
                else if (aa>=bb) begin mag=aa-bb; result_sign=sa; end
                else begin mag=bb-aa; result_sign=sb; end
                if (mag==0) result_sign=sa&&sb;
                pack_enable=1; pack_exp=common_exp-3;
            end
        end
        `PAR_FP_MUL: begin
            result_sign=sa^sb;
            if (na||nb) answer={4'd0,(sna||snb),nan};
            else if ((ia&&zb)||(ib&&za)) answer={5'b00001,nan};
            else if (ia||ib) answer={5'd0,((result_sign?signmask:64'd0)|inf)};
            else begin
                mag={192'd0,ma}*{192'd0,mb};
                pack_enable=1; pack_exp=ea+eb;
            end
        end
        // DIV/SQRT由fp_divsqrt逐拍执行，不在组合函数中提供。
        `PAR_FP_F32_TO_F64: begin
            if (width!=64) answer={5'b00001,nan};
            else begin
                sa=a[31]; qa={41'd0,a[22:0]}; xa=a[30:23]; ma=qa;
                if (xa==255) begin
                    if (qa==0) answer={5'd0,{sa,11'h7ff,52'd0}};
                    else answer={4'd0,!a[22],64'h7ff8000000000000};
                end else begin
                    if (xa!=0) ma[23]=1;
                    ea=((xa==0)?-126:(xa-127))-23;
                    mag=ma; result_sign=sa; pack_enable=1; pack_exp=ea;
                end
            end
        end
        `PAR_FP_F64_TO_F32: begin
            if (width!=64) answer={5'b00001,nan};
            else if (na) answer={4'd0,sna,64'h000000007fc00000};
            else if (ia) answer={5'd0,(sa?64'h00000000ff800000:64'h000000007f800000)};
            else begin mag=ma; answer=fp_pack(sa,mag,ea,1'b0,32); end
        end
        `PAR_FP_COMPARE: answer={4'd0,(sna||snb),64'd0};
        default: answer={5'b00001,nan};
        endcase
        if (pack_enable) answer=fp_pack(result_sign,mag,pack_exp,1'b0,width);
        fp_eval=answer;
    end
endfunction

// 将有限且幅度可表示的数转换到有符号Q128；调用方已完成范围检查。
function signed [191:0] fp_fixed;
    input [63:0] a;
    input integer width;
    integer f,bias,x,e;
    reg [63:0] m;
    reg signed [191:0] q;
    begin
        f=(width==32)?23:52; bias=(width==32)?127:1023;
        x=(a>>f)&((64'd1<<(width-f-1))-1);
        m=a&((64'd1<<f)-1);
        if (x!=0) m[f]=1;
        e=((x==0)?(1-bias):(x-bias))-f+128;
        q=$signed({64'd0,m});
        if (e>=0) q=q<<e; else q=q>>(-e);
        fp_fixed=((a>>(width-1))&1)?-q:q;
    end
endfunction

function [68:0] fixed_pack;
    input signed [191:0] q;
    input integer scale;
    input integer width;
    reg [255:0] mag;
    begin
        mag=q[191] ? -q : q;
        fixed_pack=fp_pack(q[191],mag,scale-128,1'b0,width);
    end
endfunction
