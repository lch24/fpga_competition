`timescale 1ns/1ps
// 数值向量独立生成；每次命令后污染输入，逐项检查响应、背压、复位与恢复。
module brown_checker #(parameter W=64)(output reg done, output integer errors, output integer cases, output integer protocol_cases);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,rsp_ready=0;
    wire cmd_ready,rsp_valid;
    wire [7:0] rsp_status;
    reg [7*W-1:0] command_bits,saved_command;
    wire [2*W-1:0] result_bits;
    reg [2*W-1:0] expected,snapshot;
    reg [7:0] expected_status,status_snapshot;

    integer fd,report,rc,cycles,j;
    real actual_value,expected_value,difference,tolerance;

    brown_distort #(.FP_W(W)) dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.cmd_x(command_bits[0 +: W]),.cmd_y(command_bits[W +: W]),.cmd_dist(command_bits[2*W +: 5*W]),.rsp_xd(result_bits[0 +: W]),.rsp_yd(result_bits[W +: W]));
    task check;
        input condition;
        input [255:0] label;
        begin if(condition!==1'b1) begin errors=errors+1;
            if(errors<30) $fdisplay(report,"FAIL case=%0d %0s cycle=%0d",cases,label,cycles);
        end end
    endtask
    task launch;
        begin
            @(negedge clk);check(cmd_ready,"ready before request");
            command_bits=saved_command;cmd_valid=1;
            @(negedge clk);cmd_valid=0;command_bits=~saved_command;
            check(!cmd_ready,"busy excludes command");
        end
    endtask
    task await_response;
        begin
            cycles=0;
            while(!rsp_valid && cycles<10000) begin
                @(negedge clk);cycles=cycles+1;check(!cmd_ready,"busy ready low");
            end
            check(rsp_valid,"response timeout");
        end
    endtask
    task reset_dut;
        begin @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;
            repeat(3) @(negedge clk);
            check(!rsp_valid && !cmd_ready,"reset clears valid");
            rst_n=1;@(negedge clk);check(cmd_ready,"reset recovery");
        end
    endtask
    initial begin
        done=0;errors=0;cases=0;protocol_cases=0;
        report=$fopen(W==32?"brown32_results.txt":"brown64_results.txt","w");
        fd=$fopen(W==32?"../../../data/calibration/brown32_vectors.txt":"../../../data/calibration/brown64_vectors.txt","r");
        if(fd==0 || report==0) $fatal(1,"cannot open vectors/results");
        command_bits=0;saved_command=0;cycles=0;reset_dut();
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%h %h %h\n",expected_status,saved_command,expected);
            if(rc==3) begin
                launch();await_response();
                check(rsp_status===expected_status,"status");
                check(result_bits===expected,"bit exact Brown result");
                snapshot=result_bits;status_snapshot=rsp_status;
                // 下一命令 valid 提前拉高，但响应背压时禁止接受。
                cmd_valid=1;
                repeat(5) begin @(negedge clk);check(rsp_valid && !cmd_ready && result_bits===snapshot && rsp_status===status_snapshot,"response stable under stall");end
                cmd_valid=0;rsp_ready=1;@(negedge clk);rsp_ready=0;
                check(!rsp_valid && cmd_ready,"response consumed");
                cases=cases+1;
            end else if(rc!=-1) begin check(0,"malformed vector");$fatal(1,"malformed vector");end
        end
        $fclose(fd);
        // 使用第一条正常向量做复位/恢复，避免异常快速返回掩盖在途取消错误。
        fd=$fopen(W==32?"../../../data/calibration/brown32_vectors.txt":"../../../data/calibration/brown64_vectors.txt","r");
        rc=$fscanf(fd,"%h %h %h\n",expected_status,saved_command,expected);$fclose(fd);
        launch();repeat(20) @(negedge clk);reset_dut();
        repeat(2200) begin @(negedge clk);check(!rsp_valid && cmd_ready,"no stale result after reset");end
        protocol_cases=protocol_cases+1;
        launch();await_response();reset_dut();
        protocol_cases=protocol_cases+1;
        launch();await_response();check(rsp_status==0,"success after reset");
        check(result_bits===expected,"recovery numeric result");
        rsp_ready=1;@(negedge clk);rsp_ready=0;protocol_cases=protocol_cases+1;
        $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d errors=%0d",cases,protocol_cases,errors);
        $fclose(report);done=1;
    end
endmodule

module tb_brown_distort;
    wire done32,done64;
    wire [31:0] errors32,errors64,cases32,cases64,protocol32,protocol64;
    wire done=done32 && done64;
    wire [31:0] errors=errors32+errors64,cases=cases32+cases64,protocol_cases=protocol32+protocol64;
    brown_checker #(.W(32)) b32(done32,errors32,cases32,protocol32);
    brown_checker #(.W(64)) b64(done64,errors64,cases64,protocol64);
endmodule
