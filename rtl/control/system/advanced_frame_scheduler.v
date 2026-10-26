`timescale 1ns/1ps
// Optional two-pass composition, sharing ONE physical remapper:
// distorted raw -> undistorted scratch -> inverse affine -> output.
// Config is applied only at the start of a complete upstream frame command.
module advanced_frame_scheduler #(parameter SCRATCH_BASE=32'h01800000)(
 input wire clk,rst_n,
 input wire cfg_valid,output wire cfg_ready,input wire [207:0] cfg_payload,
 input wire in_valid,output wire in_ready,input wire [31:0] in_src,in_dst,in_map,input wire in_bypass,
 output wire out_rsp_valid,input wire out_rsp_ready,output reg [7:0] out_status,
 output wire cmd_valid,input wire cmd_ready,output reg [31:0] cmd_src,cmd_dst,cmd_map,
 output reg cmd_bypass,cmd_affine,cmd_gamma,output reg [2:0] cmd_gamma_mode,output wire [191:0] cmd_matrix,
 input wire rsp_valid,output wire rsp_ready,input wire [7:0] rsp_status,input wire [31:0] pass_cycles,
 output reg [31:0] applied_count,last_cycles,first_cycles,second_cycles,
 output wire [15:0] active_flags
);
 localparam [191:0] IDENTITY={32'd0,32'd65536,32'd0,32'd0,32'd0,32'd65536};
 localparam IDLE=0,SEND=1,WAIT=2,RESPONSE=3;
 reg [1:0] state;reg two_pass,second;reg [31:0] final_dst,elapsed,job_first_cycles;
 reg [207:0] active;
 wire [207:0] selected=cfg_valid?cfg_payload:active;
 assign active_flags=active[207:192];
 // active is immutable until this whole job completes. Reuse it directly,
 // instead of another 192-bit matrix bank on the command port.
 assign cmd_matrix=active[191:0];
 assign in_ready=state==IDLE;
 assign cfg_ready=state==IDLE&&in_valid;
 assign cmd_valid=state==SEND;assign rsp_ready=state==WAIT;
 assign out_rsp_valid=state==RESPONSE;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;active<={16'd0,IDENTITY};applied_count<=0;last_cycles<=0;first_cycles<=0;second_cycles<=0;
   cmd_src<=0;cmd_dst<=0;cmd_map<=0;cmd_bypass<=0;cmd_affine<=0;cmd_gamma<=0;cmd_gamma_mode<=0;
   two_pass<=0;second<=0;final_dst<=0;out_status<=0;elapsed<=0;job_first_cycles<=0;
  end else begin
   if(state==SEND||state==WAIT)elapsed<=elapsed+1'b1;
   case(state)
    IDLE:if(in_valid)begin
     if(cfg_valid)begin active<=cfg_payload;applied_count<=applied_count+1'b1;end
     two_pass<=selected[192]&&!in_bypass;second<=0;
     final_dst<=in_dst;cmd_src<=in_src;cmd_map<=in_map;
     cmd_dst<=selected[192]&&!in_bypass?SCRATCH_BASE:in_dst;
     cmd_bypass<=in_bypass;cmd_affine<=0;
     cmd_gamma<=selected[193]&&!selected[192]&&!in_bypass;
     cmd_gamma_mode<=selected[196:194];elapsed<=0;out_status<=0;job_first_cycles<=0;
     state<=SEND;
    end
    SEND:if(cmd_ready)state<=WAIT;
    WAIT:if(rsp_valid)begin
     if(two_pass&&!second&&rsp_status==0)begin
      job_first_cycles<=pass_cycles;
      second<=1;cmd_src<=SCRATCH_BASE;cmd_dst<=final_dst;cmd_bypass<=1;cmd_affine<=1;
      cmd_gamma<=active[193];state<=SEND;
     end else begin
      // Publish the three durations atomically at logical completion.
      first_cycles<=second?job_first_cycles:pass_cycles;
      second_cycles<=second?pass_cycles:0;
      out_status<=rsp_status;last_cycles<=elapsed+1'b1;state<=RESPONSE;
     end
    end
    RESPONSE:if(out_rsp_ready)state<=IDLE;
   endcase
  end
 end
endmodule
