`timescale 1ns/1ps
// Logical byte transactions -> the Demo HMIC 256-bit port (NOT standard AXI).
// One transaction globally outstanding, single-beat physical bursts. Reads
// capture the complete physical return before allowing logical backpressure.
// Writes stage each complete physical payload BEFORE presenting AWVALID.
// Completion is axi_wusero_last, matching the Demo. Board integration must
// confirm its visibility guarantee. Reset jointly with HMIC if in flight.
module hmic_ddr_adapter #(parameter ADDR_WIDTH=28) (
    input wire clk,rst_n,ddr_ready,
    input wire rd_valid, output wire rd_ready,
    input wire [31:0] rd_addr,rd_len, input wire [15:0] rd_tag,
    output wire r_valid, input wire r_ready,
    output reg [31:0] r_data, output reg [3:0] r_keep,
    output reg [15:0] r_tag, output reg r_last,r_error,
    input wire wr_valid, output wire wr_ready,
    input wire [31:0] wr_addr,wr_len, input wire [15:0] wr_tag,
    input wire w_valid, output wire w_ready,
    input wire [31:0] w_data, input wire [3:0] w_keep, input wire w_last,
    output wire b_valid, input wire b_ready,
    output reg [15:0] b_tag, output reg b_error,
    output wire [ADDR_WIDTH-1:0] axi_araddr,
    output wire [3:0] axi_aruser_id,axi_arlen,
    output wire axi_aruser_ap,axi_arvalid, input wire axi_arready,
    input wire [255:0] axi_rdata, input wire axi_rvalid,axi_rlast,
    input wire [3:0] axi_rid,
    output wire [ADDR_WIDTH-1:0] axi_awaddr,
    output wire [3:0] axi_awuser_id,axi_awlen,
    output wire axi_awuser_ap,axi_awvalid, input wire axi_awready,
    output wire [255:0] axi_wdata, output wire [31:0] axi_wstrb,
    input wire axi_wready,axi_wusero_last, input wire [3:0] axi_wusero_id
);
    localparam IDLE=0,RADDR=1,RWAIT=2,RBYTE=3,ROUT=4,
               WIN=5,WPACK=6,WADDR=7,WTRANSFER=8,WFINISH=9,BOUT=10;
    reg [3:0] state;
    reg [31:0] addr,remaining;
    reg [255:0] read_block,write_block;
    reg [31:0] write_mask,word_data;
    reg [3:0] word_keep;
    reg [2:0] byte_index,word_count,part_count;
    reg last_word;
    reg write_turn;
    reg [255:0] next_block;
    reg [31:0] next_mask;
    integer j,c;
    wire bad_read=(rd_len==0)||({1'b0,rd_addr}+{1'b0,rd_len}>(33'd1<<(ADDR_WIDTH+2)));
    wire bad_write=(wr_len==0)||({1'b0,wr_addr}+{1'b0,wr_len}>(33'd1<<(ADDR_WIDTH+2)));
    assign wr_ready=(state==IDLE)&&ddr_ready&&(!rd_valid||write_turn);
    assign rd_ready=(state==IDLE)&&ddr_ready&&(!wr_valid||!write_turn);
    assign r_valid=state==ROUT;
    assign b_valid=state==BOUT;
    assign w_ready=state==WIN;
    // Demo addresses count 32-bit words: aligned byte address divided by four.
    assign axi_araddr=addr[ADDR_WIDTH+1:2]&~{{(ADDR_WIDTH-3){1'b0}},3'b111};
    assign axi_awaddr=axi_araddr;
    assign axi_aruser_id=0; assign axi_awuser_id=0;
    assign axi_arlen=0; assign axi_awlen=0;
    assign axi_aruser_ap=0; assign axi_awuser_ap=0;
    assign axi_arvalid=state==RADDR;
    assign axi_awvalid=state==WADDR;
    assign axi_wdata=write_block; assign axi_wstrb=write_mask;
    // Split at physical 32-byte boundaries. Valid lanes are one (Demo convention).
    always @* begin
        next_block=0; next_mask=0; c=0;
        for(j=0;j<4;j=j+1) begin
            if(j>=byte_index && j<word_count && (j-byte_index+addr[4:0])<32) begin
                next_block[(j-byte_index+addr[4:0])*8+:8]=word_data[j*8+:8];
                next_mask[j-byte_index+addr[4:0]]=word_keep[j]; c=c+1;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; addr<=0; remaining<=0; read_block<=0; write_block<=0;
            write_mask<=0; word_data<=0; word_keep<=0; byte_index<=0;
            word_count<=0; part_count<=0; last_word<=0; write_turn<=0;
            r_data<=0; r_keep<=0; r_tag<=0; r_last<=0; r_error<=0;
            b_tag<=0; b_error<=0;
        end else case(state)
            IDLE: begin
                if(rd_valid&&rd_ready) begin
                    addr<=rd_addr; remaining<=rd_len; r_tag<=rd_tag;
                    r_error<=bad_read; r_data<=0; r_keep<=0; r_last<=bad_read;
                    byte_index<=0; write_turn<=1;
                    state<=bad_read?ROUT:RADDR;
                end else if(wr_valid&&wr_ready) begin
                    addr<=wr_addr; remaining<=wr_len; b_tag<=wr_tag;
                    b_error<=bad_write; write_turn<=0;
                    state<=bad_write?BOUT:WIN;
                end
            end
            RADDR: if(axi_arready) state<=RWAIT;
            RWAIT: if(axi_rvalid) begin
                read_block<=axi_rdata;
                if(!axi_rlast||axi_rid!=0) r_error<=1;
                state<=RBYTE;
            end
            RBYTE: begin
                r_data[byte_index*8+:8]<=read_block[addr[4:0]*8+:8];
                r_keep[byte_index]<=1;
                remaining<=remaining-1; addr<=addr+1;
                if(byte_index==3||remaining==1) begin r_last<=remaining==1; state<=ROUT; end
                else begin
                    byte_index<=byte_index+1;
                    if(addr[4:0]==31) state<=RADDR;
                end
            end
            ROUT: if(r_ready) begin
                if(r_last) state<=IDLE;
                else begin byte_index<=0; r_keep<=0; r_data<=0; state<=RADDR; end
            end
            WIN: if(w_valid) begin
                word_data<=w_data; word_keep<=w_keep;
                word_count<=(remaining>=4)?3'd4:remaining[2:0];
                last_word<=remaining<=4; byte_index<=0;
                if(w_last!=(remaining<=4)) b_error<=1;
                if(w_keep!=((remaining>=4)?4'hf:((4'b1<<remaining[2:0])-1'b1))) b_error<=1;
                state<=WPACK;
            end
            WPACK: begin write_block<=next_block; write_mask<=next_mask; part_count<=c[2:0]; state<=WADDR; end
            WADDR: if(axi_awready) state<=WTRANSFER;
            WTRANSFER: if(axi_wready) begin
                if(axi_wusero_last) begin
                    if(axi_wusero_id!=0) b_error<=1;
                    state<=WFINISH;
                end else state<=WFINISH;
                // HMIC's last is the completion event; hold until it arrives.
                if(!axi_wusero_last) state<=11;
            end
            11: if(axi_wusero_last) begin
                if(axi_wusero_id!=0) b_error<=1;
                state<=WFINISH;
            end
            WFINISH: begin
                addr<=addr+part_count; remaining<=remaining-part_count;
                if(byte_index+part_count<word_count) begin byte_index<=byte_index+part_count; state<=WPACK; end
                else state<=last_word?BOUT:WIN;
            end
            BOUT: if(b_ready) state<=IDLE;
            default: state<=IDLE;
        endcase
    end
endmodule
