`timescale 1ns/1ps
`include "calib_defs.vh"
module fp_checker #(parameter W=64, PROFILE=0,
    ENABLE_EXP=1, ENABLE_LOG=1, ENABLE_SINCOS=1, ENABLE_ATAN_ACOS=1, CALIB_ALU=0)
    (output reg done=0, output reg [31:0] errors=0, output reg [31:0] checked=0);
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, req_valid=0, rsp_ready=0;
    reg [4:0] op=0;
    reg [W-1:0] a=0,b=0;
    wire req_ready,rsp_valid,less,equal,unordered;
    wire [W-1:0] result;
    wire [4:0] flags;
    generate if(CALIB_ALU) begin : compact
    calib_alu dut (.ce(1'b1),.clk(clk),.rst_n(rst_n),.req_valid(req_valid),
      .req_ready(req_ready),.req_op(op),.req_a(a),.req_b(b),.rsp_valid(rsp_valid),
      .rsp_ready(rsp_ready),.rsp_result(result),.rsp_flags(flags),
      .rsp_less(less),.rsp_equal(equal),.rsp_unordered(unordered));
    end else begin : original
    fp_operator #(.FP_W(W),.ENABLE_EXP(ENABLE_EXP),.ENABLE_LOG(ENABLE_LOG),
      .ENABLE_SINCOS(ENABLE_SINCOS),.ENABLE_ATAN_ACOS(ENABLE_ATAN_ACOS)) dut (.clk(clk),.rst_n(rst_n),.req_valid(req_valid),
      .req_ready(req_ready),.req_op(op),.req_a(a),.req_b(b),.rsp_valid(rsp_valid),
      .rsp_ready(rsp_ready),.rsp_result(result),.rsp_flags(flags),
      .rsp_less(less),.rsp_equal(equal),.rsp_unordered(unordered));
    end endgenerate
    integer fd, report, fields, cycles, i, wait_cycles;
    reg [63:0] va,vb,expected,tolerance,got,distance;
    reg [4:0] command,expected_flags;
    reg [2:0] expected_compare;
    reg [W-1:0] held;
    reg [4:0] held_flags;
    reg [2:0] held_compare;
    reg failed;
    integer op_count [0:13];
    reg [63:0] max_ulp [0:13];

    task problem;
        input [1023:0] message;
        begin
            errors=errors+1;
            if(errors<200) $fdisplay(report,"FAIL W=%0d vector=%0d op=%0d a=%h b=%h expected=%h got=%h flags=%h/%h : %0s",
                W,checked,command,va,vb,expected,result,flags,expected_flags,message);
        end
    endtask

    // Cancel while the new integer divider or range-reduction datapath is busy.
    task cancel_transcendental;
        input [4:0] command_to_cancel;
        input integer delay_cycles;
        begin
            @(negedge clk);req_valid=1;op=command_to_cancel;
            a=(W==64)?64'h4000000000000000:64'h40000000;
            b=0;
            @(posedge clk);#1;
            @(negedge clk);req_valid=0;
            repeat(delay_cycles) @(negedge clk);
            if(req_ready || rsp_valid) problem("expected active transcendental before reset");
            reset_dut;
            repeat(26000) begin
                @(posedge clk);#1;
                if(rsp_valid || !req_ready) problem("stale transcendental after reset");
            end
        end
    endtask
    task reset_dut;
        begin
            @(negedge clk); rst_n=0; req_valid=0; rsp_ready=0;
            #1;if(req_ready!==0 || rsp_valid!==0)problem("reset did not cancel response");
            @(negedge clk);rst_n=1;
        end
    endtask

    initial begin
        if(PROFILE!=0)begin
            fd=$fopen(W==64?"../../../data/calibration/vectors64.txt":"../../../data/calibration/vectors32.txt","r");
            report=$fopen($sformatf("fp%0d_profile%0d_results.txt",W,PROFILE),"w");
        end
        else if(W==64)begin fd=$fopen("../../../data/calibration/vectors64.txt","r");report=$fopen("fp64_results.txt","w");end
        else begin fd=$fopen("../../../data/calibration/vectors32.txt","r");report=$fopen("fp32_results.txt","w");end
        for(i=0;i<14;i=i+1)begin op_count[i]=0;max_ulp[i]=0;end
        if(!fd || !report)begin errors=1;done=1;end
        else begin
            reset_dut;
            // Cancel a multi-cycle operation, then ensure no stale response leaks.
            @(negedge clk);req_valid=1;op=`PAR_FP_SIN;
            if(W==64)a=64'h3ff0000000000000;else a=32'h3f800000;
            @(posedge clk);#1;
            @(negedge clk);req_valid=0;
            repeat(5)@(posedge clk);
            reset_dut;
            repeat(105)begin @(posedge clk);#1;if(rsp_valid)problem("stale result after reset");end
            if(ENABLE_EXP) begin
                cancel_transcendental(`PAR_FP_EXP,200);
                cancel_transcendental(`PAR_FP_EXP,600);
                cancel_transcendental(`PAR_FP_EXP,450);
            end
            if(ENABLE_LOG) cancel_transcendental(`PAR_FP_LOG,200);
            if(ENABLE_SINCOS) begin
                cancel_transcendental(`PAR_FP_SIN,55);
                cancel_transcendental(`PAR_FP_SIN,150);
            end

            while(!$feof(fd))begin
                fields=$fscanf(fd,"%h %h %h %h %h %h %h\n",
                    command,va,vb,expected,expected_flags,expected_compare,tolerance);
                if(fields==7)begin
                    // Enabled operations retain the independent reference vectors.
                    // Disabled operations must fail explicitly, including NaN inputs.
                    if ((!ENABLE_EXP && command==8) || (!ENABLE_LOG && command==9) ||
                        (!ENABLE_SINCOS && (command==5 || command==6)) ||
                        (!ENABLE_ATAN_ACOS && (command==7 || command==10))) begin
                        expected=(W==64)?64'h7ff8000000000000:64'h7fc00000;
                        expected_flags=1;expected_compare=0;tolerance=0;
                    end
                    checked=checked+1;
                    @(negedge clk);
                    if(req_ready!==1)problem("not ready between requests");
                    req_valid=1;op=command;a=va[W-1:0];b=vb[W-1:0];rsp_ready=0;
                    @(posedge clk);#1;
                    @(negedge clk);req_valid=0;
                    cycles=0;
                    // Bound includes serial divides, limb products, packing
                    // and CORDIC shifts; never assume a fixed response latency.
                    while(rsp_valid!==1 && cycles<26000)begin @(posedge clk);#1;cycles=cycles+1;end
                    if(rsp_valid!==1)begin problem("operation timeout");reset_dut;end
                    else begin
                        got=0;got[W-1:0]=result;
                        distance=(got>expected)?got-expected:expected-got;
                        failed=0;
                        if((^result)===1'bx) failed=1;
                        if(tolerance==0)begin if(got!==expected)failed=1;end
                        else if(distance>tolerance || got[W-1]!==expected[W-1])failed=1;
                        if(failed)problem("result mismatch or excessive ULP error");
                        if(flags!==expected_flags)problem("exception flags mismatch");
                        if({unordered,equal,less}!==expected_compare)problem("comparison mismatch");
                        if(command<14)begin
                            op_count[command]=op_count[command]+1;
                            if(distance>max_ulp[command])max_ulp[command]=distance;
                        end
                        held=result;held_flags=flags;held_compare={unordered,equal,less};
                        wait_cycles=(checked%4);
                        repeat(wait_cycles)begin
                            @(posedge clk);#1;
                            if(!rsp_valid || req_ready || result!==held ||
                               flags!==held_flags || {unordered,equal,less}!==held_compare)
                                problem("response changed under backpressure");
                        end
                        @(negedge clk);rsp_ready=1;
                        @(posedge clk);#1;
                        if(rsp_valid)problem("response not consumed");
                        @(negedge clk);rsp_ready=0;
                    end
                end else if(fields!=-1)begin
                    problem("malformed vector file");$fclose(fd);done=1;$stop;
                end
            end
            // Reset while a response is held.
            @(negedge clk);req_valid=1;op=`PAR_FP_ADD;a=0;b=0;
            @(posedge clk);#1;
            reset_dut;
            repeat(3)begin @(posedge clk);#1;if(rsp_valid)problem("held response survived reset");end
            for(i=0;i<14;i=i+1)$fdisplay(report,"OP %0d vectors=%0d max_ulp=%0d",i,op_count[i],max_ulp[i]);
            $fdisplay(report,"RESULT W=%0d vectors=%0d errors=%0d",W,checked,errors);
            $fclose(fd);$fclose(report);done=1;
        end
    end
endmodule

module tb_fp_operator;
    wire done32,done64;
    wire [31:0] errors32,errors64,checked32,checked64;
    fp_checker #(.W(32)) check32(done32,errors32,checked32);
    fp_checker #(.W(64)) check64(done64,errors64,checked64);
    wire core_done32,core_done64;
    wire [31:0] core_errors32,core_errors64,core_cases32,core_cases64;
    fp_divsqrt_checker #(.W(32)) core32(core_done32,core_errors32,core_cases32);
    fp_divsqrt_checker #(.W(64)) core64(core_done64,core_errors64,core_cases64);
    wire done=done32&&done64&&core_done32&&core_done64;
    wire [32:0] errors={1'b0,errors32}+{1'b0,errors64}+core_errors32+core_errors64;
    wire [32:0] checked={1'b0,checked32}+{1'b0,checked64};
endmodule


module fp_divsqrt_checker #(parameter W=64)(
    output reg done=0,output reg [31:0] errors=0,output reg [31:0] cases=0);
    localparam F=(W==32)?23:52;
    localparam [W-1:0] ONE=(W==32)?64'h3f800000:64'h3ff0000000000000;
    localparam [W-1:0] TWO=(W==32)?64'h40000000:64'h4000000000000000;
    localparam [W-1:0] THREE=(W==32)?64'h40400000:64'h4008000000000000;
    localparam [W-1:0] FOUR=(W==32)?64'h40800000:64'h4010000000000000;
    localparam [W-1:0] ONE_HALF=(W==32)?64'h3fc00000:64'h3ff8000000000000;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,valid=0,is_sqrt=0,ready_out=0;
    reg [W-1:0] a=0,b=0;
    wire ready,valid_out;
    wire [W-1:0] result;
    wire [4:0] flags;
    integer cycles,rounds,report,i;
    reg [W-1:0] held;
    reg [4:0] held_flags;
    fp_divsqrt #(.FP_W(W)) dut(.clk(clk),.rst_n(rst_n),.req_valid(valid),.req_ready(ready),
      .req_sqrt(is_sqrt),.req_a(a),.req_b(b),.rsp_valid(valid_out),.rsp_ready(ready_out),
      .rsp_result(result),.rsp_flags(flags));
    task check;
        input condition;input [1023:0] message;
        begin if(condition!==1'b1)begin errors=errors+1;$fdisplay(report,"FAIL W=%0d case=%0d %0s",W,cases,message);end end
    endtask
    task reset_core;
        begin
            @(negedge clk);rst_n=0;valid=0;ready_out=0;
            #1;check(!ready&&!valid_out,"reset cancels core");
            @(negedge clk);rst_n=1;
        end
    endtask
    task start;
        input sqrt_op;input [W-1:0] x,y;
        begin
            @(negedge clk);valid=1;is_sqrt=sqrt_op;a=x;b=y;
            #1;check(ready,"core request ready");
            @(posedge clk);#1;
            @(negedge clk);valid=0;a=0;b=0;is_sqrt=!sqrt_op;
        end
    endtask
    task run_case;
        input sqrt_op;input [W-1:0] x,y,expected;
        input integer expected_cycles;
        begin
            cases=cases+1;start(sqrt_op,x,y);cycles=0;rounds=0;
            while(!valid_out&&cycles<200)begin
                // Count visits before the active edge: each is a distinct registered iteration.
                if(dut.state==3||dut.state==4)rounds=rounds+1;
                @(posedge clk);#1;cycles=cycles+1;check(!ready,"busy blocks new request");
            end
            check(valid_out&&result===expected&&flags===0,"core exact result");
            check(cycles==expected_cycles,"expected multi-cycle latency including normalization");
            check(rounds==(sqrt_op?F+4:F+5),"one registered step per quotient/root digit");
            held=result;held_flags=flags;
            @(negedge clk);valid=1;a=THREE;b=ONE;
            repeat(4)begin @(posedge clk);#1;check(valid_out&&!ready&&result===held&&flags===held_flags,"core backpressure holds result");end
            @(negedge clk);valid=0;ready_out=1;
            @(posedge clk);#1;check(!valid_out&&ready,"core response consumed");
            @(negedge clk);ready_out=0;
        end
    endtask
    initial begin
        if(W==64)report=$fopen("divsqrt64_results.txt","w");else report=$fopen("divsqrt32_results.txt","w");
        if(!report)begin errors=1;done=1;end else begin
            reset_core;
            run_case(0,THREE,TWO,ONE_HALF,F+8);
            run_case(1,FOUR,0,TWO,F+7);
            run_case(0,{{(W-1){1'b0}},1'b1},ONE,{{(W-1){1'b0}},1'b1},2*F+8);
            // Reset in quotient iteration, root iteration, and subnormal normalization.
            for(i=0;i<3;i=i+1)begin
                cases=cases+1;
                start(i==1,i==2?{{(W-1){1'b0}},1'b1}:FOUR,THREE);
                repeat(5)@(posedge clk);
                reset_core;
                repeat(130)begin @(posedge clk);#1;check(!valid_out&&ready,"no stale result after core reset");end
            end
            run_case(0,THREE,TWO,ONE_HALF,F+8);
            $fdisplay(report,"RESULT W=%0d cases=%0d errors=%0d",W,cases,errors);
            $fclose(report);done=1;
        end
    end
endmodule
