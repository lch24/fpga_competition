`timescale 1ns/1ps
`include "calib_defs.vh"
module eigen_limit_checker(output reg done=0,output reg [31:0] errors=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cv=0,mv=0,last=0,rr=0;
    reg [63:0] data=0;
    wire cr,mr,rv;
    wire [7:0] status;
    wire [575:0] vector_out;
    wire [63:0] l0,l1,lmax;
    wire [13:0] rotations;
    integer round_id,r,c,cycles;
    jacobi_eigen #(.MAX_SEARCH_ROUNDS(1)) dut(
      .clk(clk),.rst_n(rst_n),.cmd_valid(cv),.cmd_ready(cr),.cmd_n(4'd6),
      .matrix_valid(mv),.matrix_ready(mr),.matrix_fp64(data),.matrix_last(last),
      .rsp_valid(rv),.rsp_ready(rr),.rsp_status(status),.rsp_min_vector_fp64(vector_out),
      .rsp_min_value_fp64(l0),.rsp_second_value_fp64(l1),.rsp_max_value_fp64(lmax),.rsp_rotations(rotations));
    initial begin
        repeat(2)@(negedge clk);rst_n=1;
        for(round_id=0;round_id<2;round_id=round_id+1)begin
            @(negedge clk);cv=1;
            @(posedge clk);if(!cr)errors=errors+1;
            @(negedge clk);cv=0;
            for(r=0;r<6;r=r+1)for(c=0;c<6;c=c+1)begin
                @(negedge clk);mv=1;last=(r==5&&c==5);
                data=(r==c)?64'h4000000000000000:0;
                if(round_id==1&&((r==0&&c==1)||(r==1&&c==0)))data=64'h3ff0000000000000;
                @(posedge clk);if(!mr)errors=errors+1;
            end
            @(negedge clk);mv=0;cycles=0;
            while(!rv&&cycles<10000)begin @(posedge clk);#1;cycles=cycles+1;end
            if(!rv)errors=errors+1;
            if(round_id==0)begin
                if(status!==0||rotations!==0||l0!==64'h4000000000000000)errors=errors+1;
            end else if(status!==`PAR_CALIB_INVALID||rotations!==1||vector_out!==0||l0!==0||l1!==0||lmax!==0)
                errors=errors+1;
            @(negedge clk);rr=1;@(posedge clk);@(negedge clk);rr=0;
        end
        done=1;
    end
endmodule

module tb_jacobi_eigen;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,matrix_valid=0,matrix_last=0,rsp_ready=0;
    reg [3:0] cmd_n=0;
    reg [63:0] matrix_fp64=0;
    wire cmd_ready,matrix_ready,rsp_valid;
    wire [7:0] rsp_status;
    wire [575:0] vector_out;
    wire [63:0] lambda0,lambda1,lambda_max;
    wire [13:0] rotations;
    jacobi_eigen dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_n(cmd_n),
      .matrix_valid(matrix_valid),.matrix_ready(matrix_ready),.matrix_fp64(matrix_fp64),.matrix_last(matrix_last),
      .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_min_vector_fp64(vector_out),
      .rsp_min_value_fp64(lambda0),.rsp_second_value_fp64(lambda1),.rsp_max_value_fp64(lambda_max),
      .rsp_rotations(rotations));
    wire limit_done;wire [31:0] limit_errors;
    eigen_limit_checker limit_test(limit_done,limit_errors);
    integer failures=0,checks=0,cases=0,protocol_cases=0,max_cycles=0;
    reg main_done=0;
    wire done=main_done&&limit_done;
    wire [32:0] errors=failures+limit_errors;
    integer fd,report,n,expected_status,compare_vector,expected_rotations,fields,i,j,cycles;
    reg [63:0] matrix_bits[0:80],gold[0:2],gold_vector[0:8],word;
    reg [575:0] held_vector;
    reg [63:0] held0,held1,heldmax;
    reg [13:0] held_rotations;
    reg [7:0] held_status;
    real a[0:80],v[0:8],values[0:2];
    real matrix_norm,row_norm,scale,norm2,dot,residual,row_sum,max_residual,max_eval_error,eval_error;
    real worst_residual=0.0,worst_norm_error=0.0;
    function real absolute;
        input real x;begin absolute=(x<0)?-x:x;end
    endfunction
    task check;
        input condition;
        input [1023:0] message;
        begin
            checks=checks+1;
            if(condition!==1'b1)begin
                failures=failures+1;
                if(failures<100)$fdisplay(report,"FAIL case=%0d t=%0t %0s status=%0d",cases,$time,message,rsp_status);
            end
        end
    endtask
    task reset_solver;
        begin
            @(negedge clk);rst_n=0;cmd_valid=0;matrix_valid=0;rsp_ready=0;
            #1;check(!cmd_ready&&!matrix_ready&&!rsp_valid,"reset gates channels");
            @(negedge clk);rst_n=1;
            @(posedge clk);#1;check(cmd_ready&&!rsp_valid,"reset returns to idle");
        end
    endtask
    task command;
        input integer size;
        begin
            @(negedge clk);cmd_valid=1;cmd_n=size;
            #1;check(cmd_ready,"command ready");
            @(posedge clk);#1;check(!cmd_ready,"command latched");
            @(negedge clk);cmd_valid=0;cmd_n=15;
        end
    endtask
    task send;
        input [63:0] value;
        input last;
        begin
            @(negedge clk);matrix_valid=1;matrix_fp64=value;matrix_last=last;
            #1;check(matrix_ready,"matrix input ready");
            @(posedge clk);#1;@(negedge clk);matrix_valid=0;
        end
    endtask
    task verify_numeric;
        begin
            matrix_norm=0;norm2=0;dot=0;max_residual=0;max_eval_error=0;
            for(i=0;i<n;i=i+1)begin
                row_norm=0;
                for(j=0;j<n;j=j+1)begin a[i*n+j]=$bitstoreal(matrix_bits[i*n+j]);row_norm=row_norm+absolute(a[i*n+j]);end
                if(row_norm>matrix_norm)matrix_norm=row_norm;
                word=vector_out[64*i +:64];
                check(word[62:52]!=11'h7ff,"finite eigenvector component");
                v[i]=$bitstoreal(word);norm2=norm2+v[i]*v[i];
                dot=dot+v[i]*$bitstoreal(gold_vector[i]);
            end
            scale=(matrix_norm>1e-30)?matrix_norm:1e-30;
            values[0]=$bitstoreal(lambda0);values[1]=$bitstoreal(lambda1);values[2]=$bitstoreal(lambda_max);
            check(lambda0[62:52]!=11'h7ff&&lambda1[62:52]!=11'h7ff&&lambda_max[62:52]!=11'h7ff,"finite eigenvalues");
            check(values[0]<=values[1]&&values[1]<=values[2],"ascending eigenvalue order");
            for(i=0;i<3;i=i+1)begin
                eval_error=absolute(values[i]-$bitstoreal(gold[i]))/scale;
                if(eval_error>max_eval_error)max_eval_error=eval_error;
                check(eval_error<2e-11,"analytic eigenvalue match");
            end
            check(absolute(norm2-1.0)<1e-10,"unit eigenvector");
            if(absolute(norm2-1.0)>worst_norm_error)worst_norm_error=absolute(norm2-1.0);
            if(compare_vector)check(absolute(absolute(dot)-1.0)<1e-9,"known minimum eigenvector up to sign");
            for(i=0;i<n;i=i+1)begin
                row_sum=0;for(j=0;j<n;j=j+1)row_sum=row_sum+a[i*n+j]*v[j];
                residual=absolute(row_sum-values[0]*v[i])/scale;
                if(residual>max_residual)max_residual=residual;
            end
            if(max_residual>worst_residual)worst_residual=max_residual;
            check(max_residual<2e-11,"independent A*v-lambda*v residual");
            for(i=n;i<9;i=i+1)check(vector_out[64*i +:64]===0,"unused vector slots are zero");
            if(expected_rotations>=0)check(rotations==expected_rotations,"expected rotation count");
            check(rotations<100*n*n,"iteration cap respected");
            $fdisplay(report,"NUMERIC case=%0d rotations=%0d eigen_error=%e residual=%e norm_error=%e",
                       cases,rotations,max_eval_error,max_residual,absolute(norm2-1.0));
        end
    endtask
    task await_result;
        input [7:0] status;
        input numeric_check;
        begin
            cycles=0;
            while(!rsp_valid&&cycles<2000000)begin
                @(posedge clk);#1;cycles=cycles+1;check(!cmd_ready,"busy blocks next task");
            end
            if(cycles>max_cycles)max_cycles=cycles;
            check(rsp_valid,"bounded completion");check(rsp_status===status,"expected status");
            if(status==0&&numeric_check)verify_numeric;
            if(status!=0)check(vector_out===0&&lambda0===0&&lambda1===0&&lambda_max===0,"failed payload cleared");
            held_vector=vector_out;held0=lambda0;held1=lambda1;heldmax=lambda_max;
            held_rotations=rotations;held_status=rsp_status;
            @(negedge clk);cmd_valid=1;cmd_n=9;matrix_valid=1;matrix_fp64=64'h7ff8000000000000;
            repeat(4)begin
                @(posedge clk);#1;
                check(rsp_valid&&!cmd_ready&&!matrix_ready&&vector_out===held_vector&&lambda0===held0&&
                      lambda1===held1&&lambda_max===heldmax&&rotations===held_rotations&&rsp_status===held_status,
                      "all output fields stable under backpressure");
            end
            @(negedge clk);cmd_valid=0;matrix_valid=0;rsp_ready=1;
            @(posedge clk);#1;check(!rsp_valid&&cmd_ready,"response consumed");
            @(negedge clk);rsp_ready=0;
        end
    endtask
    task identity_after_reset;
        begin
            n=6;compare_vector=0;expected_rotations=0;
            for(i=0;i<3;i=i+1)gold[i]=64'h3ff0000000000000;
            for(i=0;i<9;i=i+1)gold_vector[i]=0;
            command(6);
            for(i=0;i<36;i=i+1)begin
                matrix_bits[i]=(i/6==i%6)?64'h3ff0000000000000:0;
                send(matrix_bits[i],i==35);
            end
            await_result(0,1);
        end
    endtask
    initial begin
        fd=$fopen("../../../data/calibration/eigen_vectors.txt","r");report=$fopen("eigen_results.txt","w");
        if(!fd||!report)begin failures=1;main_done=1;end
        else begin
            reset_solver;
            while(!$feof(fd))begin
                fields=$fscanf(fd,"%d %d %d %d\n",n,expected_status,compare_vector,expected_rotations);
                if(fields==4)begin
                    cases=cases+1;
                    for(i=0;i<n*n;i=i+1)begin fields=$fscanf(fd,"%h\n",matrix_bits[i]);check(fields==1,"matrix fixture");end
                    for(i=0;i<3;i=i+1)fields=$fscanf(fd,"%h\n",gold[i]);
                    for(i=0;i<9;i=i+1)fields=$fscanf(fd,"%h\n",gold_vector[i]);
                    command(n);
                    for(i=0;i<n*n;i=i+1)begin
                        if(cases%2)begin if(i%7==0)repeat(2)@(negedge clk);send(matrix_bits[i],i==n*n-1);end
                        else begin
                            @(negedge clk);matrix_valid=1;matrix_fp64=matrix_bits[i];matrix_last=(i==n*n-1);
                            #1;check(matrix_ready,"continuous stream ready");@(posedge clk);#1;
                        end
                    end
                    @(negedge clk);matrix_valid=0;
                    await_result(expected_status,expected_status==0);
                    $fdisplay(report,"CASE %0d n=%0d status=%0d cycles=%0d",cases,n,rsp_status,cycles);
                end else if(fields!=-1)begin check(0,"malformed fixture");$stop;end
            end
            $fclose(fd);
            protocol_cases=protocol_cases+1;command(0);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(5);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(10);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(6);send(0,1);await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(6);
            for(i=0;i<36;i=i+1)send(0,0);
            await_result(`PAR_BAD_CONFIG,0);
            protocol_cases=protocol_cases+1;command(9);send(0,0);reset_solver;identity_after_reset;
            protocol_cases=protocol_cases+1;command(6);
            for(i=0;i<36;i=i+1)begin
                word=(i/6==i%6)?64'h4000000000000000:0;
                if(i==1||i==6)word=64'h3ff0000000000000;
                send(word,i==35);
            end
            cycles=0;
            while(!(dut.fp_op==`PAR_FP_ATAN2&&dut.fp_req_ready&&dut.state==31)&&cycles<10000)begin
                @(posedge clk);#1;cycles=cycles+1;
            end
            check(cycles<10000,"reached rotation arithmetic for cancellation");
            @(posedge clk);#1;reset_solver;
            repeat(150)begin @(posedge clk);#1;check(!rsp_valid&&cmd_ready,"no stale rotation result");end
            identity_after_reset;
            protocol_cases=protocol_cases+1;command(0);check(rsp_valid,"pending failure response");
            reset_solver;identity_after_reset;
            $fdisplay(report,"RESULT matrices=%0d protocol_cases=%0d errors=%0d checks=%0d max_cycles=%0d residual=%e norm_error=%e",
                cases,protocol_cases,failures,checks,max_cycles,worst_residual,worst_norm_error);
            $fclose(report);main_done=1;
        end
    end
endmodule
