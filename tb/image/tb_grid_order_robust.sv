`timescale 1ns/1ps
// Repeated jobs, boundary/row outliers, and a degenerate non-grid. Compare the
// complete point membership, not only the count. SHARED exercises the board's
// shared add/validator ports with synchronous CE stalls between operations.
module tb_grid_order_robust #(parameter SHARED=0,CASE_ONLY=0,EXTERNAL=0);
    `include "detection_services.vh"
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,ce=1,start=0,pts_valid=0,pts_done=0,out_ready=1;
    reg [31:0] pts_x=0,pts_y=0;
    wire pts_ready,busy,done,out_valid,out_grid_ok;
    wire [1:0] status;
    wire [31:0] out_x,out_y;
    wire [15:0] out_total;
    wire add_rsp_ready,seq_done,val_start,val_rd_en,math_hyp_valid,math_hyp_rsp_ready;
    wire [63:0] add_req_a,add_req_b,add_result;
    wire [31:0] seq_result,val_rd_x,val_rd_y;
    wire add_req_valid,add_req_sub,add_req_pair,add_req_ready,add_rsp_valid;
    wire [5:0] val_rd_addr;
    wire [31:0] math_hyp_a,math_hyp_b;
    reg seq_valid=0,seq_enter=0;reg [7:0] seq_id=0;reg [15:0] seq_arg=0;

    fp32_pair_add_pool #(.CLIENTS(1),.USE_CE(1)) pool(.clk(clk),.rst_n(rst_n),.ce(ce),
        .req_valid(add_req_valid),.req_sub(add_req_sub),.req_pair(add_req_pair),.req_ready(add_req_ready),
        .req_a(add_req_a),.req_b(add_req_b),.rsp_valid(add_rsp_valid),.rsp_ready(add_rsp_ready),.result(add_result));
    wire val_busy,val_done,val_valid;wire [31:0] val_cost;
    grid_validate #(.USE_CE(1)) validator(.clk(clk),.rst_n(rst_n),.ce(ce),
        .start(val_start),.busy(val_busy),.done(val_done),.valid_out(val_valid),
        .cost_out(val_cost),.n_in(16'd40),.rd_en(val_rd_en),.rd_addr(val_rd_addr),.rd_x(val_rd_x),.rd_y(val_rd_y));
    grid_order_ctrl #(.SHARED_ADD(SHARED),.SHARED_VALIDATE(SHARED),.EXTERNAL_SEQ(EXTERNAL),.USE_CE(1)) dut(
        .add_req_valid(add_req_valid),.add_req_sub(add_req_sub),.add_req_pair(add_req_pair),.add_req_ready(add_req_ready),
        .add_req_a(add_req_a),.add_req_b(add_req_b),.add_rsp_valid(add_rsp_valid),
        .add_rsp_ready(add_rsp_ready),.add_result(add_result),
        .seq_valid(seq_valid),.seq_enter(seq_enter),.seq_id(seq_id),.seq_arg(seq_arg),
        .seq_done(seq_done),.seq_result(seq_result),
        .val_start(val_start),.val_busy(val_busy),.val_done(val_done),.val_valid(val_valid),
        .val_cost(val_cost),.val_rd_en(val_rd_en),.val_rd_addr(val_rd_addr),
        .val_rd_x(val_rd_x),.val_rd_y(val_rd_y),
        .math_hyp_valid(math_hyp_valid),.math_hyp_ready(1'b1),
        .math_hyp_a(math_hyp_a),.math_hyp_b(math_hyp_b),
        .math_hyp_rsp_valid(1'b0),.math_hyp_rsp_ready(math_hyp_rsp_ready),
        .math_hyp_result(32'd0),
        .ce(ce),.clk(clk),.rst_n(rst_n),.start(start),.busy(busy),.done(done),
        .status(status),
        .pts_valid(pts_valid),.pts_ready(pts_ready),.pts_x(pts_x),.pts_y(pts_y),
        .pts_done(pts_done),
        .out_valid(out_valid),.out_ready(out_ready),.out_x(out_x),.out_y(out_y),
        .out_total(out_total),.out_grid_ok(out_grid_ok));

    reg [31:0] px[0:127];
    reg [31:0] py[0:127];
    reg        seen[0:39];
    integer npts,idx,k,i,r,c,errors,cyc,found,cases,case_err,angle,begin_tick;
    reg finished=0;
    integer case_errors[0:7];
    integer case_cycles[0:7];
    integer ticks=0;
    // Pause only the compute phase; stimulus/output are checked separately.
    always @(negedge clk)begin
        ticks=ticks+1;
        ce=!(SHARED && dut.state==2 && dut.run_angle && ticks%17==0);
    end

    function [31:0] as_float(input integer v);
        integer b; reg [31:0] frac;
        begin
            if(v==0) as_float=0;
            else begin
                b=0; while((v>>b)>1) b=b+1;
                frac=v<<(23-b);
                as_float=((127+b)<<23)|(frac & 32'h007fffff);
            end
        end
    endfunction

    task instruction(input [7:0] id,input [15:0] arg);
        begin
            @(negedge clk);#1;seq_valid=1;seq_enter=1;seq_id=id;seq_arg=arg;
            @(posedge clk);while(!ce)@(posedge clk);
            @(negedge clk);#1;seq_enter=0;
            cyc=0;
            while(!seq_done)begin
                @(negedge clk);#1;cyc=cyc+1;
                if(cyc>12000000)$fatal(1,"instruction timeout %0d angle %0d",id,arg);
            end
            seq_valid=0;
        end
    endtask

    task feed_and_check(input integer n,input [8*16-1:0] name);
        begin if(CASE_ONLY==0 || CASE_ONLY==cases)begin
            case_err=0;
            for(i=0;i<40;i=i+1) seen[i]=1'b0;
            @(negedge clk); start=1;
            @(negedge clk); start=0;
            idx=0;
            while(idx<n) begin
                pts_valid=1; pts_x=px[idx]; pts_y=py[idx];
                @(negedge clk);
                if(pts_ready) idx=idx+1;
            end
            pts_valid=0;
            @(negedge clk); pts_done=1;
            @(negedge clk); pts_done=0;
            begin_tick=ticks;
            if(EXTERNAL)begin
                for(angle=0;angle<90;angle=angle+1)instruction(DET_ANGLE,angle);
                instruction(DET_ORDER_FINISH,0);
            end
            cyc=0;
            while(!done) begin @(negedge clk); cyc=cyc+1; if(cyc>12000000) $fatal(1,"timeout %0s",name); end
            case_cycles[cases-1]=ticks-begin_tick;
            if(cases==8)begin
                if(out_grid_ok || status!=2'b10)begin errors=errors+1;case_err=case_err+1;end
            end else if(!out_grid_ok || status!=2'b01 || out_total!=16'd40) begin
                errors=errors+1; case_err=case_err+1;
                $display("  %0s: grid_ok=%0d status=%0d total=%0d",name,out_grid_ok,status,out_total);
            end else begin
                for(k=0;k<40;k=k+1) begin
                    cyc=0;
                    while(!out_valid) begin @(negedge clk); cyc=cyc+1; if(cyc>1000) $fatal(1,"output stall"); end
                    found=0;
                    for(i=0;i<40;i=i+1) begin
                        if(!seen[i] && out_x==px[i] && out_y==py[i]) begin
                            seen[i]=1; found=1; i=40;
                        end
                    end
                    if(!found) begin
                        errors=errors+1; case_err=case_err+1;
                        $display("  %0s: unexpected output point %0d,%0d",name,out_x,out_y);
                    end
                    @(negedge clk);
                end
                for(i=0;i<40;i=i+1) if(!seen[i]) begin
                    errors=errors+1; case_err=case_err+1;
                    $display("  %0s: missing grid point %0d",name,i);
                end
            end
            case_errors[cases-1]=case_err;
            if(case_err==0) $display("PASS %0s",name);
            else            $display("FAIL %0s",name);
            repeat(4) @(negedge clk);
        end end
    endtask

    initial begin
        errors=0; cases=0;
        for(i=0;i<8;i=i+1)begin case_errors[i]=0;case_cycles[i]=0;end
        repeat(4) @(negedge clk); rst_n=1; repeat(4) @(negedge clk);

        // 40 inner corners of a mildly sheared/perspective 8x5 grid.
        for(r=0;r<5;r=r+1) for(c=0;c<8;c=c+1) begin
            i=r*8+c;
            px[i]=as_float(300+c*62+r*4);
            py[i]=as_float(170+r*62+c*3);
        end

        // Case 1: clean 40.
        npts=40; cases=cases+1; feed_and_check(npts,"clean-40");

        // Case 2: 3 strays above the board.
        px[40]=as_float(250); py[40]=as_float(40);
        px[41]=as_float(700); py[41]=as_float(30);
        px[42]=as_float(900); py[42]=as_float(60);
        npts=43; cases=cases+1; feed_and_check(npts,"3-strays-above");

        // Case 3: 2 strays below-right of the board.
        px[40]=as_float(1150); py[40]=as_float(650);
        px[41]=as_float(980);  py[41]=as_float(690);
        npts=42; cases=cases+1; feed_and_check(npts,"2-strays-below");

        // Case 4: barrel/bezel corners just outside the outer grid corners.
        px[40]=as_float(270); py[40]=as_float(140);
        px[41]=as_float(764); py[41]=as_float(142);
        px[42]=as_float(272); py[42]=as_float(446);
        npts=43; cases=cases+1; feed_and_check(npts,"bezel-corners");

        // Case 5: one stray inside a row, between columns of row 2.
        px[40]=as_float(300+4*62+2*4+31); py[40]=as_float(170+2*62-30);
        npts=41; cases=cases+1; feed_and_check(npts,"1-stray-mid-row");

        // Two nearby distractors in the same row cannot be removed by a
        // single-skip window. Both true end points still belong to the grid.
        px[40]=as_float(300+2*62+2*4+25); py[40]=as_float(170+2*62+2*3);
        px[41]=as_float(300+5*62+2*4+25); py[41]=as_float(170+2*62+5*3);
        npts=42; cases=cases+1; feed_and_check(npts,"2-strays-in-row");

        // 13 strays: two inside every row plus three isolated points. Counts
        // like 53 must not force us to either fail or accept a false grid.
        for(r=0;r<5;r=r+1)begin
            px[40+2*r]=as_float(300+2*62+r*4+25);py[40+2*r]=as_float(170+r*62+2*3);
            px[41+2*r]=as_float(300+5*62+r*4+25);py[41+2*r]=as_float(170+r*62+5*3);
        end
        px[50]=as_float(20);py[50]=as_float(20);
        px[51]=as_float(1000);py[51]=as_float(20);
        px[52]=as_float(20);py[52]=as_float(700);
        npts=53;cases=cases+1;feed_and_check(npts,"13-strays");

        // A sufficient count alone does not constitute a two-dimensional grid.
        for(i=0;i<40;i=i+1)begin px[i]=as_float(300+8*i);py[i]=as_float(100);end
        cases=cases+1;feed_and_check(40,"collinear-reject");

        if(errors!=0) $fatal(1,"GRID_ROBUST_FAIL errors=%0d",errors);
        finished=1;
        $display("PASS grid_order_robust cases=%0d",cases);
        $finish;
    end
    initial begin #400000000; $fatal(1,"watchdog"); end
endmodule
