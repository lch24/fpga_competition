`timescale 1ns/1ps
// 数值向量独立生成；每次命令后污染输入，逐项检查响应、背压、复位与恢复。
module tb_rotation;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,rsp_ready=0;
    wire cmd_ready,rsp_valid;
    wire [7:0] rsp_status;
    reg [769-1:0] command_bits,saved_command;
    wire [768-1:0] result_bits;
    reg [768-1:0] expected,snapshot;
    reg [7:0] expected_status,status_snapshot;
    reg done=0;integer errors=0,cases=0,protocol_cases=0;
    integer fd,report,rc,cycles,j;
    real actual_value,expected_value,difference,tolerance;

    rotation  dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.cmd_mode(command_bits[768]),.cmd_rotvec_fp64(command_bits[0 +: 192]),.cmd_r_fp64(command_bits[192 +: 576]),.rsp_rotvec_fp64(result_bits[0 +: 192]),.rsp_r_fp64(result_bits[192 +: 576]));
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

        report=$fopen("rotation_results.txt","w");
        fd=$fopen("../../../data/calibration/rotation_vectors.txt","r");
        if(fd==0 || report==0) $fatal(1,"cannot open vectors/results");
        command_bits=0;saved_command=0;cycles=0;reset_dut();
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%h %h %h\n",expected_status,saved_command,expected);
            if(rc==3) begin
                launch();await_response();
                check(rsp_status===expected_status,"status");

                if(expected_status==0) begin
                    for(j=0;j<768/64;j=j+1) begin
                        actual_value=$bitstoreal(result_bits[j*64 +: 64]);
                        expected_value=$bitstoreal(expected[j*64 +: 64]);
                        difference=actual_value-expected_value;if(difference<0) difference=-difference;
                        tolerance=2e-11*(1+(expected_value<0?-expected_value:expected_value));
                        check((^result_bits[j*64 +: 64])!==1'bx && result_bits[j*64+52 +: 11]!=2047 && difference<=tolerance,"numeric result");
                    end
                end else check(result_bits===0,"failure payload zero");
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
        fd=$fopen("../../../data/calibration/rotation_vectors.txt","r");
        rc=$fscanf(fd,"%h %h %h\n",expected_status,saved_command,expected);$fclose(fd);
        launch();repeat(20) @(negedge clk);reset_dut();
        repeat(2200) begin @(negedge clk);check(!rsp_valid && cmd_ready,"no stale result after reset");end
        protocol_cases=protocol_cases+1;
        launch();await_response();reset_dut();
        protocol_cases=protocol_cases+1;
        launch();await_response();check(rsp_status==0,"success after reset");

        rsp_ready=1;@(negedge clk);rsp_ready=0;protocol_cases=protocol_cases+1;
        $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d errors=%0d",cases,protocol_cases,errors);
        $fclose(report);done=1;
    end
endmodule
