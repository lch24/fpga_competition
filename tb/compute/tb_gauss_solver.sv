`timescale 1ns/1ps
`include "calib_defs.vh"
module tb_gauss_solver;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,matrix_valid=0,matrix_last=0,rsp_ready=0;
    reg [4:0] cmd_n=0;
    reg [63:0] matrix_fp64=0;
    wire cmd_ready,matrix_ready,rsp_valid;
    wire [7:0] rsp_status;
    wire [`PAR_SCALE_W-1:0] solution;
    gauss_solver dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
      .cmd_n(cmd_n),.matrix_valid(matrix_valid),.matrix_ready(matrix_ready),
      .matrix_fp64(matrix_fp64),.matrix_last(matrix_last),.rsp_valid(rsp_valid),
      .rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_solution_fp64(solution));
    integer errors=0,checks=0,cases=0,protocol_cases=0,max_cycles=0;
    reg done=0;
    integer fd,report,fields,n,i,j,cycles,total,seed;
    reg [7:0] expected_status;
    reg [63:0] expected_word;
    reg [63:0] inputs [0:701];
    reg [`PAR_SCALE_W-1:0] expected,held;
    reg [7:0] held_status;

    task check;
        input condition;
        input [1023:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1)begin
                errors=errors+1;
                if(errors<100)$fdisplay(report,"FAIL case=%0d t=%0t %0s status=%0d",cases,$time,message,rsp_status);
            end
        end
    endtask
    task reset_solver;
        begin
            @(negedge clk);rst_n=0;cmd_valid=0;matrix_valid=0;rsp_ready=0;
            #1;check(!cmd_ready&&!matrix_ready&&!rsp_valid,"reset gates all channels");
            @(negedge clk);rst_n=1;
            @(posedge clk);#1;check(cmd_ready&&!rsp_valid,"ready after reset");
        end
    endtask
    task command;
        input integer size;
        begin
            @(negedge clk);cmd_valid=1;cmd_n=size;
            #1;check(cmd_ready,"command accepted");
            @(posedge clk);#1;check(!cmd_ready,"single task in flight");
            @(negedge clk);cmd_valid=0;
            // Change command payload after handshake to check it was latched.
            cmd_n=31;
        end
    endtask
    task send_element;
        input [63:0] value;
        input last;
        begin
            @(negedge clk);matrix_valid=1;matrix_fp64=value;matrix_last=last;
            #1;check(matrix_ready,"matrix element accepted");
            @(posedge clk);#1;
            @(negedge clk);matrix_valid=0;
        end
    endtask
    task await_result;
        input [7:0] status;
        input [`PAR_SCALE_W-1:0] expected_solution;
        begin
            cycles=0;
            while(rsp_valid!==1 && cycles<400000)begin
                @(posedge clk);#1;cycles=cycles+1;
                check(!cmd_ready,"no command accepted while computing");
            end
            if(cycles>max_cycles)max_cycles=cycles;
            check(rsp_valid,"bounded solve completion");
            check(rsp_status===status,"expected success/failure status");
            check(solution===expected_solution,"all 26 solution slots match reference bit-for-bit");
            if(solution!==expected_solution && errors<100)
                for(j=0;j<26;j=j+1)
                    if(solution[64*j +: 64]!==expected_solution[64*j +: 64])
                        $fdisplay(report,"  x[%0d] expected=%h got=%h",j,expected_solution[64*j +:64],solution[64*j +:64]);
            held=solution;held_status=rsp_status;
            // A pending next command and matrix input must not alter the held response.
            @(negedge clk);cmd_valid=1;cmd_n=1;matrix_valid=1;matrix_fp64=64'h7ff8000000000000;
            repeat(4)begin
                @(posedge clk);#1;
                check(rsp_valid&&!cmd_ready&&!matrix_ready&&solution===held&&rsp_status===held_status,
                      "response stable under backpressure, new inputs blocked");
            end
            @(negedge clk);cmd_valid=0;matrix_valid=0;rsp_ready=1;
            @(posedge clk);#1;check(!rsp_valid&&cmd_ready,"response consumed");
            @(negedge clk);rsp_ready=0;
        end
    endtask

    initial begin
        report=$fopen("gauss_results.txt","w");fd=$fopen("../../../data/calibration/gauss_vectors.txt","r");
        if(!fd||!report)begin errors=1;done=1;end
        else begin
            reset_solver;
            while(!$feof(fd))begin
                fields=$fscanf(fd,"%d %h\n",n,expected_status);
                if(fields==2)begin
                    cases=cases+1;total=n*(n+1);expected=0;
                    for(i=0;i<total;i=i+1)begin fields=$fscanf(fd,"%h\n",inputs[i]);check(fields==1,"valid matrix fixture");end
                    for(i=0;i<26;i=i+1)begin
                        fields=$fscanf(fd,"%h\n",expected_word);check(fields==1,"valid solution fixture");
                        expected[64*i +:64]=expected_word;
                    end
                    command(n);
                    for(i=0;i<total;i=i+1)begin
                        // Burst and gapped streams alternate; payload is valid only on handshakes.
                        if(cases%2)begin
                            if(i%7==0)repeat(2)@(negedge clk);
                            send_element(inputs[i],i==total-1);
                        end else begin
                            @(negedge clk);matrix_valid=1;matrix_fp64=inputs[i];matrix_last=(i==total-1);
                            #1;check(matrix_ready,"continuous stream ready");
                            @(posedge clk);#1;
                        end
                    end
                    @(negedge clk);matrix_valid=0;
                    await_result(expected_status,expected);
                    $fdisplay(report,"CASE %0d n=%0d status=%0d compute_cycles=%0d",cases,n,rsp_status,cycles);
                end else if(fields!=-1)begin check(0,"bad fixture header");$stop;end
            end
            $fclose(fd);
            protocol_cases=protocol_cases+1;command(0);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(27);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(31);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(2);
            send_element(64'h3ff0000000000000,1);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(1);
            send_element(64'h3ff0000000000000,0);
            send_element(64'h4000000000000000,0);await_result(`PAR_BAD_CONFIG,0);

            // Cancel an incomplete load and then reuse the same storage.
            protocol_cases=protocol_cases+1;command(3);
            send_element(64'h3ff0000000000000,0);reset_solver;
            command(1);send_element(64'h4000000000000000,0);send_element(64'h4018000000000000,1);
            expected=0;expected[63:0]=64'h4008000000000000;await_result(`PAR_OK,expected);

            // Cancel during a real floating-point transaction.
            protocol_cases=protocol_cases+1;command(1);
            send_element(64'h4000000000000000,0);send_element(64'h4018000000000000,1);
            cycles=0;
            while(!dut.fp_rsp_valid && cycles<1000)begin @(posedge clk);#1;cycles=cycles+1;end
            check(dut.fp_rsp_valid,"reached arithmetic transaction before reset");
            reset_solver;
            repeat(20)begin @(posedge clk);#1;check(!rsp_valid&&cmd_ready,"no stale result after cancellation");end
            command(1);send_element(64'h4000000000000000,0);send_element(64'h4018000000000000,1);
            await_result(`PAR_OK,expected);

            // Reset while output is held, without accepting it.
            protocol_cases=protocol_cases+1;command(0);
            check(rsp_valid,"invalid command response pending");
            reset_solver;
            command(1);send_element(64'h4000000000000000,0);send_element(64'h4018000000000000,1);
            await_result(`PAR_OK,expected);
            $fdisplay(report,"RESULT matrices=%0d protocol_cases=%0d checks=%0d errors=%0d max_compute_cycles=%0d",
                      cases,protocol_cases,checks,errors,max_cycles);
            $fclose(report);done=1;
        end
    end
endmodule
