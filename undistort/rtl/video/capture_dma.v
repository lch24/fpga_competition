// Frame-triggered camera -> RGB565 DDR. Command descriptor is a stable mailbox
// while the PCLK domain captures one whole frame. Pixels cannot be backpressured:
// FIFO overflow invalidates the entire output frame. No partial frame is published.
module capture_dma #(parameter FIFO_BITS=10)(
 input wire clk,rst_n,pclk,prst_n,
 input wire camera_vsync,camera_valid,input wire [15:0] camera_pixel,
 input wire cmd_valid,output wire cmd_ready,input wire [31:0] cmd_base,cmd_stride,
 input wire [15:0] cmd_width,cmd_height,
 output wire rsp_valid,input wire rsp_ready,output reg [7:0] rsp_status,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,wr_len,
 output wire [15:0] wr_tag,output wire w_valid,input wire w_ready,
 output wire [31:0] w_data,output wire [3:0] w_keep,output wire w_last,
 input wire b_valid,output wire b_ready,input wire [15:0] b_tag,input wire b_error
);
 reg [2:0] state;reg [31:0] base,stride,total,written;reg [15:0] width,x,y;
 reg start_toggle;
 (* ASYNC_REG="TRUE" *) reg [1:0] start_sync,done_sync;
 reg done_toggle,seen_start,vs_d,armed,capturing,overflow;
 reg [31:0] captured,accepted_count;
 reg [31:0] finished_accepted,finished_count;reg finished_error;
 wire fv,fr,fw;wire [15:0] fd;
 wire wi_ready,wo_valid,wo_error;
 assign fw=capturing&&camera_valid&&captured<total;
 async_fifo #(.WIDTH(16),.ADDR_BITS(FIFO_BITS)) fifo(
  .wclk(pclk),.wrst_n(prst_n),.wvalid(fw),.wready(fr),.wdata(camera_pixel),
  .rclk(clk),.rrst_n(rst_n),.rvalid(fv),.rready(state==1&&wi_ready),.rdata(fd));
 output_writer writer(.clk(clk),.rst_n(rst_n),.in_valid(state==1&&fv),.in_ready(wi_ready),
  .in_addr(base+y*stride+({16'd0,x}<<1)),.in_data({16'd0,fd}),.in_bytes(3'd2),
  .out_valid(wo_valid),.out_ready(state==2),.out_error(wo_error),
  .wr_valid(wr_valid),.wr_ready(wr_ready),.wr_addr(wr_addr),.wr_len(wr_len),.wr_tag(wr_tag),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_keep(w_keep),.w_last(w_last),
  .b_valid(b_valid),.b_ready(b_ready),.b_tag(b_tag),.b_error(b_error));
 assign cmd_ready=state==0;assign rsp_valid=state==3;
 always @(posedge pclk or negedge prst_n)begin
  if(!prst_n)begin start_sync<=0;done_toggle<=0;seen_start<=0;vs_d<=0;armed<=0;capturing<=0;overflow<=0;captured<=0;accepted_count<=0;finished_accepted<=0;finished_count<=0;finished_error<=0;end
  else begin
   start_sync<={start_sync[0],start_toggle};vs_d<=camera_vsync;
   if(start_sync[1]!=seen_start)begin seen_start<=start_sync[1];armed<=1;captured<=0;accepted_count<=0;overflow<=0;end
   if(camera_vsync&&!vs_d)begin
    if(capturing)begin
     capturing<=0;finished_count<=captured;finished_accepted<=accepted_count;
     finished_error<=overflow||captured!=total;done_toggle<=~done_toggle;
    end else if(armed)begin armed<=0;capturing<=1;end
   end
   if(capturing&&camera_valid)begin
    captured<=captured+1;
    if(captured>=total||!fr)overflow<=1;
    if(fw&&fr)accepted_count<=accepted_count+1;
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;base<=0;stride<=0;total<=0;written<=0;width<=0;x<=0;y<=0;start_toggle<=0;done_sync<=0;rsp_status<=0;end
  else begin
   done_sync<={done_sync[0],done_toggle};
   case(state)
    0:if(cmd_valid)begin
     base<=cmd_base;stride<=cmd_stride;width<=cmd_width;total<={16'd0,cmd_width}*{16'd0,cmd_height};written<=0;x<=0;y<=0;rsp_status<=0;
     if(cmd_width==0||cmd_height==0||cmd_stride<({16'd0,cmd_width}<<1))begin rsp_status<=1;state<=3;end
     else begin start_toggle<=~start_toggle;state<=1;end
    end
    1:begin
     if(fv&&wi_ready)state<=2;
     else if(done_sync[1]==start_toggle && !fv && written==finished_accepted)begin
      if((finished_error||written!=total)&&rsp_status==0)rsp_status<=3;state<=3;
     end
    end
    2:if(wo_valid)begin
     written<=written+1;if(wo_error)rsp_status<=5;
     if(x==width-1)begin x<=0;y<=y+1;end else x<=x+1;
     state<=1;
    end
    3:if(rsp_ready)state<=0;
    default:state<=0;
   endcase
  end
 end
endmodule
