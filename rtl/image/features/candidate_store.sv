`timescale 1ns / 1ps
//==============================================================================
// candidate_store.sv — 候选点 A/B 双缓冲；EXTERNAL_RAM=1 使用 DDR 后备存储。
// DDR 模式按有效 ce 周期保持同步读接口，父级负责暂停计算，候选上限不变。
//------------------------------------------------------------------------------
// 语义：两个 64bit/地址 的同步读双口 RAM（A/B），每地址 {y[31:0], x[31:0]}
//   （fp32 位模式）。phase 选择写/读对象：
//     phase=0 : 写 A、读 B
//     phase=1 : 写 B、读 A
//   供主控 MERGE 阶段 A→B、B→A 轮换：merge 读当前满的一侧，结果流式
//   写进另一侧，一轮完成后翻转 phase 即可。
//
// 流式写口：wr_valid/wr_ready（wr_ready = 当前写侧未满 && !clr），内部自增
//   写指针；A/B 各自独立写指针与 count（phase 切换不串扰）。
//   clr 清零当前 phase 写侧的写指针/count（clr 优先，同拍不写）。
// 点随机读口：rd_en/rd_addr（对侧 buffer），1 拍延迟出整个点 {rd_x,rd_y}
//   （dual_port_ram 语义：rd_en=0 时输出保持上一次的值）。
//==============================================================================
module candidate_store #(parameter USE_CE=0, parameter EXTERNAL_RAM=0,
    parameter N_ADDR_W = 14                       // 深度 = 2**N_ADDR_W = 16384
) (
    // Global synchronous stall for variable-latency backing memory.
    input wire ce,
    output wire scratch_rd_en,scratch_wr_en,
    output wire [14:0] scratch_rd_addr,scratch_wr_addr,
    output wire [63:0] scratch_wr_data,
    input wire [63:0] scratch_rd_data,


    input  wire                    clk,
    input  wire                    rst_n,
    // 控制
    input  wire                    clr,          // 清当前写侧（写指针/count）
    input  wire                    phase,        // 0:写A读B  1:写B读A
    // 流式写口
    input  wire                    wr_valid,
    output wire                    wr_ready,
    input  wire [31:0]             wr_x,
    input  wire [31:0]             wr_y,
    output wire [15:0]             count,        // 当前写侧已存点数
    // 点随机读口（对侧 buffer，1 拍延迟出整个点）
    input  wire                    rd_en,
    input  wire [N_ADDR_W-1:0]     rd_addr,
    output wire [31:0]             rd_x,
    output wire [31:0]             rd_y
);

    localparam DEPTH = (1 << N_ADDR_W);

    reg [N_ADDR_W-1:0] wr_ptr_a, wr_ptr_b;
    reg [15:0]         cnt_a, cnt_b;

    wire wr_fire = wr_valid && wr_ready;

    // 当前写侧指针/count（phase 选择）
    wire [N_ADDR_W-1:0] wr_ptr_sel = phase ? wr_ptr_b : wr_ptr_a;
    wire [15:0]         cnt_sel    = phase ? cnt_b    : cnt_a;

    assign wr_ready = !clr && (cnt_sel < DEPTH[15:0]);

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr_a <= {N_ADDR_W{1'b0}};
            wr_ptr_b <= {N_ADDR_W{1'b0}};
            cnt_a    <= 16'd0;
            cnt_b    <= 16'd0;
        end else if(!USE_CE || ce) begin if (clr) begin
            // 一次清两侧写指针/count：phase 可能随后切换，确保新写侧从 0 起
            wr_ptr_a <= {N_ADDR_W{1'b0}};
            wr_ptr_b <= {N_ADDR_W{1'b0}};
            cnt_a    <= 16'd0;
            cnt_b    <= 16'd0;
        end else if (wr_fire) begin
            if (phase) begin
                wr_ptr_b <= wr_ptr_b + 1'b1;
                cnt_b    <= cnt_b + 16'd1;
            end else begin
                wr_ptr_a <= wr_ptr_a + 1'b1;
                cnt_a    <= cnt_a + 16'd1;
            end
        end
    end // synchronous clock enable
    end

    assign count = cnt_sel;

    //--------------------------------------------------------------------
    // A/B 两个存储体（64bit/地址：{y,x}）
    //--------------------------------------------------------------------
    wire [63:0] wr_data = {wr_y, wr_x};
    wire        wa_en   = wr_fire && ~phase;      // 写 A
    wire        wb_en   = wr_fire &&  phase;      // 写 B
    wire        ra_en   = rd_en   &&  phase;      // 读 A（phase=1 时）
    wire        rb_en   = rd_en   && ~phase;      // 读 B（phase=0 时）

    wire [63:0] rd_a, rd_b;

    generate if(EXTERNAL_RAM) begin : g_external
        // Logical word address: bank A then bank B; each word is {y,x}.
        // The parent advances ce only after the DDR operation is complete.
        assign scratch_wr_en=wr_fire;
        assign scratch_rd_en=rd_en;
        assign scratch_wr_addr={phase,wr_ptr_sel};
        assign scratch_rd_addr={~phase,rd_addr};
        assign scratch_wr_data=wr_data;
        assign rd_a=scratch_rd_data;
        assign rd_b=scratch_rd_data;
    end else begin : g_local
        assign scratch_wr_en=0;assign scratch_rd_en=0;
        assign scratch_wr_addr=0;assign scratch_rd_addr=0;assign scratch_wr_data=0;
    dual_port_ram #(.USE_CE(USE_CE),.DATA_WIDTH(64), .ADDR_WIDTH(N_ADDR_W)) u_ram_a (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (wa_en),
        .wr_addr  (wr_ptr_a),
        .wr_data  (wr_data),
        .rd_en    (ra_en),
        .rd_addr  (rd_addr),
        .rd_data  (rd_a)
    );

    dual_port_ram #(.USE_CE(USE_CE),.DATA_WIDTH(64), .ADDR_WIDTH(N_ADDR_W)) u_ram_b (.ce(ce),
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (wb_en),
        .wr_addr  (wr_ptr_b),
        .wr_data  (wr_data),
        .rd_en    (rb_en),
        .rd_addr  (rd_addr),
        .rd_data  (rd_b)
    );

    end endgenerate

    assign rd_x = phase ? rd_a[31:0]  : rd_b[31:0];
    assign rd_y = phase ? rd_a[63:32] : rd_b[63:32];

endmodule

// DDR backing for the two candidate lists: 256 KiB at the default 14-bit
// point index. Only a 4 KiB read cache and one pending point stay on chip.
// Writes are acknowledged before the logical RAM cycle commits. Reads are
// sampled before writes (including same-address read/write). Independent gray
// and candidate caches may stall; advance commits both services together.
// clear is legal only between jobs, after all bus transactions have drained.
module candidate_ddr_cache #(parameter ADDR_W=15, LINE_BITS=6)(
 input wire clk,rst_n,clear,advance,
 input wire [31:0] backing_base,
 input wire rd_en,wr_en,
 input wire [ADDR_W-1:0] rd_addr,wr_addr,
 input wire [63:0] wr_data,output reg [63:0] rd_data,
 output wire step_en,output wire idle,output reg error,
 output wire rd_valid,input wire rd_ready,output wire [31:0] rd_address,rd_length,
 output wire [15:0] rd_tag,input wire r_valid,output wire r_ready,
 input wire [31:0] r_data,input wire [3:0] r_keep,input wire [15:0] r_tag,
 input wire r_last,r_error,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_address,wr_length,
 output wire [15:0] wr_tag,output wire w_valid,input wire w_ready,
 output wire [31:0] w_data,output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 localparam LINES=1<<LINE_BITS,WORDS=LINES*8;
 localparam IDLE=0,RREQ=1,RDATA=2,WREQ=3,WDATA=4,WDONE=5;
 reg [2:0] state;
 assign idle=(state==IDLE);
 reg [LINES-1:0] valid;
 reg [ADDR_W-4:0] tags[0:LINES-1];
 reg [31:0] low_mem[0:WORDS-1],high_mem[0:WORDS-1];
 reg read_done,write_done,fill_error;
 reg [63:0] pending_read,write_q;
 reg [ADDR_W-1:0] fill_addr,write_addr;
 reg [4:0] beat;
 reg write_high;
 wire [LINE_BITS-1:0] ri=rd_addr[3+:LINE_BITS];
 wire hit=valid[ri] && tags[ri]==rd_addr[ADDR_W-1:3];
 wire [LINE_BITS+2:0] read_index=rd_addr[LINE_BITS+2:0];
 wire [LINE_BITS+2:0] fill_index={fill_addr[3+:LINE_BITS],beat[3:1]};
 assign step_en=rst_n && !clear && !error && state==IDLE &&
     (!rd_en || read_done) && (!wr_en || write_done);
 assign rd_valid=rst_n && state==RREQ;
 assign rd_address=backing_base+({{(32-ADDR_W){1'b0}},fill_addr}<<3);
 assign rd_length=64;assign rd_tag=16'hcb01;
 assign r_ready=rst_n && state==RDATA;
 assign wr_valid=rst_n && state==WREQ;
 assign wr_address=backing_base+({{(32-ADDR_W){1'b0}},write_addr}<<3);
 assign wr_length=8;assign wr_tag=16'hcb02;
 assign w_valid=rst_n && state==WDATA;
 assign w_data=write_high?write_q[63:32]:write_q[31:0];
 assign w_keep=4'hf;assign w_last=write_high;
 assign b_ready=rst_n && state==WDONE;
 // Synchronous RAM, no array reset and no asynchronous multiport lookup.
 always @(posedge clk) begin
     if(state==RDATA && r_valid && r_ready && beat<16) begin
         if(beat[0]) high_mem[fill_index]<=r_data;
         else low_mem[fill_index]<=r_data;
     end
     if(state==IDLE && rd_en && !read_done && hit && !error)
         pending_read<={high_mem[read_index],low_mem[read_index]};
 end
 always @(posedge clk or negedge rst_n) begin
     if(!rst_n) begin
         state<=IDLE;valid<=0;error<=0;read_done<=0;write_done<=0;
         fill_error<=0;beat<=0;fill_addr<=0;write_addr<=0;write_q<=0;
         write_high<=0;rd_data<=0;
     end else if(clear) begin
         state<=IDLE;valid<=0;error<=0;read_done<=0;write_done<=0;fill_error<=0;
     end else begin
         case(state)
             IDLE: if(!error) begin
                 if(step_en && advance) begin
                     if(rd_en) rd_data<=pending_read;
                     read_done<=0;write_done<=0;
                 end else if(rd_en && !read_done) begin
                     if(hit) read_done<=1;
                     else begin
                         fill_addr<={rd_addr[ADDR_W-1:3],3'b0};
                         beat<=0;fill_error<=0;state<=RREQ;
                     end
                 end else if(wr_en && !write_done) begin
                     write_addr<=wr_addr;write_q<=wr_data;write_high<=0;
                     // Write-through invalidation, so a later read refills
                     // from DDR only after this write response was received.
                     if(valid[wr_addr[3+:LINE_BITS]] &&
                        tags[wr_addr[3+:LINE_BITS]]==wr_addr[ADDR_W-1:3])
                         valid[wr_addr[3+:LINE_BITS]]<=0;
                     state<=WREQ;
                 end
             end
             RREQ: if(rd_ready) state<=RDATA;
             RDATA: if(r_valid) begin
                 if(r_error || r_tag!=16'hcb01 || r_keep!=4'hf || beat>=16)
                     fill_error<=1;
                 if(r_last) begin
                     state<=IDLE;
                     if(fill_error || r_error || r_tag!=16'hcb01 || r_keep!=4'hf || beat!=15)
                         error<=1;
                     else begin
                         valid[fill_addr[3+:LINE_BITS]]<=1;
                         tags[fill_addr[3+:LINE_BITS]]<=fill_addr[ADDR_W-1:3];
                     end
                 end else if(beat<16) beat<=beat+1'b1;
             end
             WREQ: if(wr_ready) state<=WDATA;
             WDATA: if(w_ready) begin
                 if(write_high) state<=WDONE;else write_high<=1;
             end
             WDONE: if(b_valid) begin
                 state<=IDLE;write_done<=1;
                 if(b_error || b_tag!=16'hcb02) error<=1;
             end
             default: state<=IDLE;
         endcase
     end
 end
endmodule
