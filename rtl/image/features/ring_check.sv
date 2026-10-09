`timescale 1ns/1ps
module ring_check #(parameter USE_CE=0, parameter VARIABLE_SCALE=0,
    parameter W          = 32,
    parameter H          = 24,
    parameter GRAY_ADDR_W = 14,
    parameter ROM_FILE   = "data/rom/ring_cos_sin.mem"
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,
    input wire [2:0] cfg_scale,

    input  wire                    clk,
    input  wire                    rst_n,
    // 输入点（流式握手，一个点一次调用）
    input  wire                    in_valid,
    output wire                    in_ready,
    input  wire [31:0]             in_x,
    input  wire [31:0]             in_y,
    input  wire [31:0]             in_radius,
    // 灰度读口（外部 RAM，1 拍延迟：rd_en 下一拍 rd_data 有效）
    output reg                     rd_en,
    output reg  [GRAY_ADDR_W-1:0]  rd_addr,
    input  wire [7:0]              rd_data,
    // 结果输出
    output reg                     out_valid,
    input  wire                    out_ready,
    output reg  [31:0]             out_hi,
    output reg  [31:0]             out_lo,
    output reg  [31:0]             out_thr,
    output reg  [31:0]             out_ntrans,
    output reg  [31:0]             out_opp_err,
    output reg                     out_sector_ok,
    output reg                     out_pass
);

    // Fixed-radius integer ring: rounded center plus constant offsets.
    `include "lround_function.vh"
    localparam IDLE=0, BOUND=1, ADDRESS=2, WAIT_DATA=3, SAMPLE=4,
               SMOOTH=5, CONTRAST=6, COMPARE=7, FINISH=8, OUTPUT=9;
    reg [3:0] state;
    reg signed [31:0] cx,cy;
    reg [3:0] radius;
    reg [1:0] bank;
    reg [4:0] index,first_change,last_change;
    reg [5:0] transitions;
    reg [7:0] values[0:31];
    reg [9:0] smoothed[0:31];
    reg [9:0] lo,hi;
    reg [14:0] opposite;
    reg sectors;
    wire [4:0] prev=index-5'd1,next=index+5'd1,opp=index+5'd16;
    wire [9:0] smooth_value={2'b0,values[prev]}+{1'b0,values[index],1'b0}+{2'b0,values[next]};
    wire [10:0] threshold={1'b0,hi}+{1'b0,lo};
    wire change=({smoothed[index],1'b0}>threshold)!=({smoothed[prev],1'b0}>threshold);
    wire [9:0] difference=smoothed[index]>=smoothed[opp] ?
        smoothed[index]-smoothed[opp] : smoothed[opp]-smoothed[index];
    wire [4:0] gap=index-last_change,wrap_gap=first_change-last_change;
    wire [19:0] opposite25=({5'b0,opposite}<<4)+({5'b0,opposite}<<3)+opposite;
    wire [19:0] contrast224=({10'b0,(hi-lo)}<<8)-({10'b0,(hi-lo)}<<5);
    wire signed [31:0] active_w=VARIABLE_SCALE?(W>>cfg_scale):W;
    wire signed [31:0] active_h=VARIABLE_SCALE?(H>>cfg_scale):H;
    function automatic signed [4:0] offset(input [1:0] b,input [4:0] i,input axis);
        begin case({b,axis,i})
            8'd0:offset=5'd4;
            8'd1:offset=5'd4;
            8'd2:offset=5'd4;
            8'd3:offset=5'd3;
            8'd4:offset=5'd3;
            8'd5:offset=5'd2;
            8'd6:offset=5'd2;
            8'd7:offset=5'd1;
            8'd8:offset=5'd0;
            8'd9:offset=5'd31;
            8'd10:offset=5'd30;
            8'd11:offset=5'd30;
            8'd12:offset=5'd29;
            8'd13:offset=5'd29;
            8'd14:offset=5'd28;
            8'd15:offset=5'd28;
            8'd16:offset=5'd28;
            8'd17:offset=5'd28;
            8'd18:offset=5'd28;
            8'd19:offset=5'd29;
            8'd20:offset=5'd29;
            8'd21:offset=5'd30;
            8'd22:offset=5'd30;
            8'd23:offset=5'd31;
            8'd24:offset=5'd0;
            8'd25:offset=5'd1;
            8'd26:offset=5'd2;
            8'd27:offset=5'd2;
            8'd28:offset=5'd3;
            8'd29:offset=5'd3;
            8'd30:offset=5'd4;
            8'd31:offset=5'd4;
            8'd32:offset=5'd0;
            8'd33:offset=5'd1;
            8'd34:offset=5'd2;
            8'd35:offset=5'd2;
            8'd36:offset=5'd3;
            8'd37:offset=5'd3;
            8'd38:offset=5'd4;
            8'd39:offset=5'd4;
            8'd40:offset=5'd4;
            8'd41:offset=5'd4;
            8'd42:offset=5'd4;
            8'd43:offset=5'd3;
            8'd44:offset=5'd3;
            8'd45:offset=5'd2;
            8'd46:offset=5'd2;
            8'd47:offset=5'd1;
            8'd48:offset=5'd0;
            8'd49:offset=5'd31;
            8'd50:offset=5'd30;
            8'd51:offset=5'd30;
            8'd52:offset=5'd29;
            8'd53:offset=5'd29;
            8'd54:offset=5'd28;
            8'd55:offset=5'd28;
            8'd56:offset=5'd28;
            8'd57:offset=5'd28;
            8'd58:offset=5'd28;
            8'd59:offset=5'd29;
            8'd60:offset=5'd29;
            8'd61:offset=5'd30;
            8'd62:offset=5'd30;
            8'd63:offset=5'd31;
            8'd64:offset=5'd6;
            8'd65:offset=5'd6;
            8'd66:offset=5'd6;
            8'd67:offset=5'd5;
            8'd68:offset=5'd4;
            8'd69:offset=5'd3;
            8'd70:offset=5'd2;
            8'd71:offset=5'd1;
            8'd72:offset=5'd0;
            8'd73:offset=5'd31;
            8'd74:offset=5'd30;
            8'd75:offset=5'd29;
            8'd76:offset=5'd28;
            8'd77:offset=5'd27;
            8'd78:offset=5'd26;
            8'd79:offset=5'd26;
            8'd80:offset=5'd26;
            8'd81:offset=5'd26;
            8'd82:offset=5'd26;
            8'd83:offset=5'd27;
            8'd84:offset=5'd28;
            8'd85:offset=5'd29;
            8'd86:offset=5'd30;
            8'd87:offset=5'd31;
            8'd88:offset=5'd0;
            8'd89:offset=5'd1;
            8'd90:offset=5'd2;
            8'd91:offset=5'd3;
            8'd92:offset=5'd4;
            8'd93:offset=5'd5;
            8'd94:offset=5'd6;
            8'd95:offset=5'd6;
            8'd96:offset=5'd0;
            8'd97:offset=5'd1;
            8'd98:offset=5'd2;
            8'd99:offset=5'd3;
            8'd100:offset=5'd4;
            8'd101:offset=5'd5;
            8'd102:offset=5'd6;
            8'd103:offset=5'd6;
            8'd104:offset=5'd6;
            8'd105:offset=5'd6;
            8'd106:offset=5'd6;
            8'd107:offset=5'd5;
            8'd108:offset=5'd4;
            8'd109:offset=5'd3;
            8'd110:offset=5'd2;
            8'd111:offset=5'd1;
            8'd112:offset=5'd0;
            8'd113:offset=5'd31;
            8'd114:offset=5'd30;
            8'd115:offset=5'd29;
            8'd116:offset=5'd28;
            8'd117:offset=5'd27;
            8'd118:offset=5'd26;
            8'd119:offset=5'd26;
            8'd120:offset=5'd26;
            8'd121:offset=5'd26;
            8'd122:offset=5'd26;
            8'd123:offset=5'd27;
            8'd124:offset=5'd28;
            8'd125:offset=5'd29;
            8'd126:offset=5'd30;
            8'd127:offset=5'd31;
            8'd128:offset=5'd8;
            8'd129:offset=5'd8;
            8'd130:offset=5'd7;
            8'd131:offset=5'd7;
            8'd132:offset=5'd6;
            8'd133:offset=5'd4;
            8'd134:offset=5'd3;
            8'd135:offset=5'd2;
            8'd136:offset=5'd0;
            8'd137:offset=5'd30;
            8'd138:offset=5'd29;
            8'd139:offset=5'd28;
            8'd140:offset=5'd26;
            8'd141:offset=5'd25;
            8'd142:offset=5'd25;
            8'd143:offset=5'd24;
            8'd144:offset=5'd24;
            8'd145:offset=5'd24;
            8'd146:offset=5'd25;
            8'd147:offset=5'd25;
            8'd148:offset=5'd26;
            8'd149:offset=5'd28;
            8'd150:offset=5'd29;
            8'd151:offset=5'd30;
            8'd152:offset=5'd0;
            8'd153:offset=5'd2;
            8'd154:offset=5'd3;
            8'd155:offset=5'd4;
            8'd156:offset=5'd6;
            8'd157:offset=5'd7;
            8'd158:offset=5'd7;
            8'd159:offset=5'd8;
            8'd160:offset=5'd0;
            8'd161:offset=5'd2;
            8'd162:offset=5'd3;
            8'd163:offset=5'd4;
            8'd164:offset=5'd6;
            8'd165:offset=5'd7;
            8'd166:offset=5'd7;
            8'd167:offset=5'd8;
            8'd168:offset=5'd8;
            8'd169:offset=5'd8;
            8'd170:offset=5'd7;
            8'd171:offset=5'd7;
            8'd172:offset=5'd6;
            8'd173:offset=5'd4;
            8'd174:offset=5'd3;
            8'd175:offset=5'd2;
            8'd176:offset=5'd0;
            8'd177:offset=5'd30;
            8'd178:offset=5'd29;
            8'd179:offset=5'd28;
            8'd180:offset=5'd26;
            8'd181:offset=5'd25;
            8'd182:offset=5'd25;
            8'd183:offset=5'd24;
            8'd184:offset=5'd24;
            8'd185:offset=5'd24;
            8'd186:offset=5'd25;
            8'd187:offset=5'd25;
            8'd188:offset=5'd26;
            8'd189:offset=5'd28;
            8'd190:offset=5'd29;
            8'd191:offset=5'd30;
            default:offset=0;
        endcase end
    endfunction
    // Exact conversion for small unsigned diagnostic values divided by 2^shift.
    function automatic [31:0] diagnostic(input [15:0] x,input [2:0] shift);
        integer k,top;reg [7:0] e;reg [31:0] m;
        begin top=0;for(k=0;k<16;k=k+1)if(x[k])top=k;
            e=127+top-shift;m={16'b0,x}<<(23-top);
            diagnostic=x==0?0:{1'b0,e,m[22:0]};end
    endfunction
    assign in_ready=rst_n && state==IDLE;
    always @(posedge clk) begin
        if(!rst_n) begin state<=IDLE;rd_en<=0;rd_addr<=0;out_valid<=0;
            out_hi<=0;out_lo<=0;out_thr<=0;out_ntrans<=0;out_opp_err<=0;
            out_sector_ok<=0;out_pass<=0;end
        else if(!USE_CE || ce) begin
            rd_en<=0;
            case(state)
                IDLE:if(in_valid)begin
                    cx<=lround_f32(in_x);cy<=lround_f32(in_y);
                    radius<=in_radius==32'h40800000 ? 4:in_radius==32'h40c00000 ? 6:8;
                    bank<=in_radius==32'h40800000 ? 0:in_radius==32'h40c00000 ? 1:2;
                    out_hi<=0;out_lo<=0;out_thr<=0;out_ntrans<=0;out_opp_err<=0;
                    out_sector_ok<=0;out_pass<=0;index<=0;lo<=1020;hi<=0;
                    opposite<=0;transitions<=0;sectors<=1;first_change<=0;last_change<=0;
                    if((in_radius!=32'h40800000 && in_radius!=32'h40c00000 && in_radius!=32'h41000000) ||
                       in_x[30:23]==255 || in_y[30:23]==255)state<=OUTPUT;
                    else state<=BOUND;
                end
                BOUND:if(cx<radius+1 || cy<radius+1 || cx>=active_w-radius-1 || cy>=active_h-radius-1)state<=OUTPUT;
                    else state<=ADDRESS;
                ADDRESS:begin
                    rd_addr<=(cy+offset(bank,index,1'b1))*active_w+cx+offset(bank,index,1'b0);
                    rd_en<=1;state<=WAIT_DATA;
                end
                WAIT_DATA:state<=SAMPLE;
                SAMPLE:begin values[index]<=rd_data;
                    if(index==31)begin index<=0;state<=SMOOTH;end
                    else begin index<=index+1'b1;state<=ADDRESS;end
                end
                SMOOTH:begin smoothed[index]<=smooth_value;
                    if(smooth_value<lo)lo<=smooth_value;if(smooth_value>hi)hi<=smooth_value;
                    if(index==31)begin index<=0;state<=CONTRAST;end else index<=index+1'b1;
                end
                CONTRAST:if(hi-lo<80)state<=OUTPUT;else state<=COMPARE;
                COMPARE:begin
                    opposite<=opposite+difference;
                    if(change)begin
                        transitions<=transitions+1'b1;
                        if(transitions==0)first_change<=index;
                        else if(gap<3 || gap>13)sectors<=0;
                        last_change<=index;
                    end
                    if(index==31)state<=FINISH;else index<=index+1'b1;
                end
                FINISH:begin
                    out_hi<=diagnostic({6'b0,hi},2);out_lo<=diagnostic({6'b0,lo},2);
                    out_thr<=diagnostic({5'b0,threshold},3);out_ntrans<=transitions;
                    out_opp_err<=diagnostic({1'b0,opposite},2);
                    out_sector_ok<=transitions==4 && opposite25<=contrast224;
                    out_pass<=transitions==4 && opposite25<=contrast224 && sectors && wrap_gap>=3 && wrap_gap<=13;
                    state<=OUTPUT;
                end
                OUTPUT:begin out_valid<=1;
                    if(out_valid && out_ready)begin out_valid<=0;state<=IDLE;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
