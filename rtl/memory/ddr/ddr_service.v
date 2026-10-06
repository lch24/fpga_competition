`timescale 1ns/1ps
// Round-robin arbitration. Separate read/write ownership persists through last
// return / write completion, including downstream and client backpressure.
module ddr_service #(parameter CLIENTS=4) (
    input wire clk,rst_n,
    input wire [CLIENTS-1:0] c_rd_valid, output reg [CLIENTS-1:0] c_rd_ready,
    input wire [CLIENTS*32-1:0] c_rd_addr,c_rd_len,
    input wire [CLIENTS*16-1:0] c_rd_tag,
    output reg [CLIENTS-1:0] c_r_valid, input wire [CLIENTS-1:0] c_r_ready,
    output wire [31:0] c_r_data, output wire [3:0] c_r_keep,
    output wire [15:0] c_r_tag, output wire c_r_last,c_r_error,
    input wire [CLIENTS-1:0] c_wr_valid, output reg [CLIENTS-1:0] c_wr_ready,
    input wire [CLIENTS*32-1:0] c_wr_addr,c_wr_len,
    input wire [CLIENTS*16-1:0] c_wr_tag,
    input wire [CLIENTS-1:0] c_w_valid, output reg [CLIENTS-1:0] c_w_ready,
    input wire [CLIENTS*32-1:0] c_w_data, input wire [CLIENTS*4-1:0] c_w_keep,
    input wire [CLIENTS-1:0] c_w_last,
    output reg [CLIENTS-1:0] c_b_valid, input wire [CLIENTS-1:0] c_b_ready,
    output wire [15:0] c_b_tag, output wire c_b_error,
    output wire rd_valid,input wire rd_ready, output wire [31:0] rd_addr,rd_len,
    output wire [15:0] rd_tag,
    input wire r_valid,output wire r_ready,input wire [31:0] r_data,
    input wire [3:0] r_keep,input wire [15:0] r_tag,input wire r_last,r_error,
    output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
    output wire [15:0] wr_tag,
    output wire w_valid,input wire w_ready,output wire [31:0] w_data,
    output wire [3:0] w_keep,output wire w_last,
    input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
    localparam PTR_W=(CLIENTS>1)?$clog2(CLIENTS):1;
    reg [PTR_W-1:0] rp,wp,ro,wo;
    integer rg,wg,j,index;
    reg rb,wb,read_locked,write_locked;
    integer read_selection,write_selection;
    always @* begin
        rg=-1; wg=-1;
        for(j=CLIENTS-1;j>=0;j=j-1) begin
            // Both operands are bounded to 0..CLIENTS-1: one subtraction
            // implements wrap for any client count, without signed remainder.
            index=rp+j; if(index>=CLIENTS) index=index-CLIENTS;
            if(c_rd_valid[index]) rg=index;
            index=wp+j; if(index>=CLIENTS) index=index-CLIENTS;
            if(c_wr_valid[index]) wg=index;
        end
        if(read_locked) rg=read_selection;
        if(write_locked) wg=write_selection;
        c_rd_ready=0; c_wr_ready=0; c_r_valid=0; c_w_ready=0; c_b_valid=0;
        if(!rb && rg>=0) c_rd_ready[rg]=rd_ready;
        if(!wb && wg>=0) c_wr_ready[wg]=wr_ready;
        if(rb) c_r_valid[ro]=r_valid;
        if(wb) begin c_w_ready[wo]=w_ready; c_b_valid[wo]=b_valid; end
    end
    assign rd_valid=!rb && rg>=0;
    assign rd_addr=rg>=0?c_rd_addr[rg*32+:32]:32'd0;
    assign rd_len=rg>=0?c_rd_len[rg*32+:32]:32'd0;
    assign rd_tag=rg>=0?c_rd_tag[rg*16+:16]:16'd0;
    assign wr_valid=!wb && wg>=0;
    assign wr_addr=wg>=0?c_wr_addr[wg*32+:32]:32'd0;
    assign wr_len=wg>=0?c_wr_len[wg*32+:32]:32'd0;
    assign wr_tag=wg>=0?c_wr_tag[wg*16+:16]:16'd0;
    assign r_ready=rb&&c_r_ready[ro];
    assign c_r_data=r_data; assign c_r_keep=r_keep; assign c_r_tag=r_tag;
    assign c_r_last=r_last; assign c_r_error=r_error;
    assign w_valid=wb&&c_w_valid[wo]; assign w_data=c_w_data[wo*32+:32];
    assign w_keep=c_w_keep[wo*4+:4]; assign w_last=c_w_last[wo];
    assign b_ready=wb&&c_b_ready[wo]; assign c_b_tag=b_tag; assign c_b_error=b_error;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rp<=0;wp<=0;ro<=0;wo<=0;rb<=0;wb<=0;read_locked<=0;write_locked<=0;read_selection<=0;write_selection<=0; end
        else begin
            if(rd_valid&&!rd_ready) begin read_locked<=1; read_selection<=rg; end
            if(wr_valid&&!wr_ready) begin write_locked<=1; write_selection<=wg; end
            if(rd_valid&&rd_ready) begin ro<=rg;rb<=1;read_locked<=0; end
            if(wr_valid&&wr_ready) begin wo<=wg;wb<=1;write_locked<=0; end
            if(r_valid&&r_ready&&r_last) begin rb<=0;rp<=(ro==CLIENTS-1)?0:ro+1'b1; end
            if(b_valid&&b_ready) begin wb<=0;wp<=(wo==CLIENTS-1)?0:wo+1'b1; end
        end
    end
endmodule
