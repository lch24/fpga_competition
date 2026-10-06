`timescale 1ns/1ps
// 固定一拍 RAM 模型；每个流元素在握手沿核对，随机/长时间背压不允许丢点。
module tb_residual_engine;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,rsp_ready=0,data_ready=0;
    wire cmd_ready,rsp_valid,data_valid,data_last,point_rd_en;
    wire [1:0] point_rd_view_id;
    wire [5:0] point_rd_index;
    wire [7:0] data_index,rsp_status;
    wire [63:0] data_fp64,rsp_cost_fp64;
    reg [15:0] cmd_width,cmd_height;
    reg [1727:0] cmd_state,saved_state;
    reg [15:0] saved_width,saved_height;
    reg point_rd_valid=0;
    reg [31:0] point_rd_x_fp32=0,point_rd_y_fp32=0;
    reg [7679:0] observations;
    reg [15359:0] expected_residuals;
    reg [63:0] expected_cost,cost_snapshot,data_snapshot;
    reg [7:0] index_snapshot,status_snapshot;
    reg last_snapshot,stalled=0,monitor=0,done=0,abort_test=0;
    integer errors=0,cases=0,protocol_cases=0,checks=0;
    integer fd,report,rc,cycles,received=0,reads=0,expected_count,expected_status,drop_read,addr;
    integer stall_cycles=0,max_cycles=0,phase=0;
    reg [31:0] random_state=32'h79b12951;
    residual_engine dut(
        .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .cmd_width(cmd_width),.cmd_height(cmd_height),.cmd_state(cmd_state),
        .point_rd_en(point_rd_en),.point_rd_view_id(point_rd_view_id),.point_rd_index(point_rd_index),
        .point_rd_valid(point_rd_valid),.point_rd_x_fp32(point_rd_x_fp32),.point_rd_y_fp32(point_rd_y_fp32),
        .data_valid(data_valid),.data_ready(data_ready),.data_index(data_index),
        .data_fp64(data_fp64),.data_last(data_last),
        .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_cost_fp64(rsp_cost_fp64));
    task check;
        input condition;
        input [255:0] label;
        begin checks=checks+1;if(condition!==1'b1) begin errors=errors+1;
            if(errors<40) $fdisplay(report,"FAIL case=%0d phase=%0d %0s received=%0d reads=%0d",cases,phase,label,received,reads);
        end end
    endtask
    task numeric;
        input [63:0] actual_bits,expected_bits;
        input real tolerance;
        real actual_value,expected_value,difference,bound;
        begin
            actual_value=$bitstoreal(actual_bits);expected_value=$bitstoreal(expected_bits);
            difference=actual_value-expected_value;if(difference<0) difference=-difference;
            bound=tolerance*(1+(expected_value<0?-expected_value:expected_value));
            check((^actual_bits)!==1'bx && actual_bits[62:52]!=2047 && difference<=bound,"numeric residual/cost");
        end
    endtask
    // 在请求采样沿更新数据，下一采样沿由 DUT 接收，匹配 corner_store 契约。
    always @(posedge clk) begin
        if(!rst_n) begin point_rd_valid<=0;point_rd_x_fp32<=0;point_rd_y_fp32<=0;end
        else begin
            point_rd_valid<=point_rd_en && (reads!=drop_read);
            if(point_rd_en) begin
                addr=point_rd_view_id*40+point_rd_index;
                check(point_rd_view_id<3 && point_rd_index<40 && addr==reads,"RAM address ordering");
                point_rd_x_fp32<=observations[addr*64 +: 32];
                point_rd_y_fp32<=observations[addr*64+32 +: 32];
                reads=reads+1;
            end
        end
        if(rst_n && monitor) begin
            if(stalled) check(data_valid && data_fp64===data_snapshot && data_index===index_snapshot && data_last===last_snapshot,"stream stable under stall");
            if(data_valid) begin
                check(data_index==received && data_last==(received==239),"stream index/last");
                if(data_ready) begin
                    check(received<240,"no extra residual");
                    if(received<240) numeric(data_fp64,expected_residuals[received*64 +: 64],2e-9);
                    received=received+1;
                end
            end
            stalled=data_valid && !data_ready;
            data_snapshot=data_fp64;index_snapshot=data_index;last_snapshot=data_last;
            if(rsp_valid && !abort_test) check(received==expected_count,"response after expected stream");
        end else stalled=0;
    end
    task reset_dut;
        begin
            @(negedge clk);rst_n=0;cmd_valid=0;rsp_ready=0;data_ready=0;monitor=0;
            repeat(3) @(negedge clk);
            check(!rsp_valid && !data_valid && !point_rd_en,"reset clears all producers");
            rst_n=1;@(negedge clk);check(cmd_ready,"ready after reset");
        end
    endtask
    task load_vector;
        begin
            rc=$fscanf(fd,"%d %d %d %d %d %h %h %h %h\n",expected_status,expected_count,drop_read,saved_width,saved_height,saved_state,observations,expected_residuals,expected_cost);
            if(rc!=9 && rc!=-1) $fatal(1,"malformed residual vector");
        end
    endtask
    task launch;
        begin
            @(negedge clk);check(cmd_ready,"ready before command");
            received=0;reads=0;stall_cycles=0;monitor=1;stalled=0;
            cmd_state=saved_state;cmd_width=saved_width;cmd_height=saved_height;
            cmd_valid=1;data_ready=0;
            @(negedge clk);cmd_valid=0;cmd_state=~saved_state;cmd_width=1;cmd_height=1;
            check(!cmd_ready,"busy after acceptance");
        end
    endtask
    task wait_result;
        begin
            cycles=0;
            while(!rsp_valid && cycles<200000) begin
                // 同时覆盖随机背压，以及最后一个数据与每视图边界的长背压。
                random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
                data_ready=random_state[0] | random_state[2];
                if(data_valid && (data_index==0 || data_index==79 || data_index==159 || data_index==239) && stall_cycles<17) begin data_ready=0;stall_cycles=stall_cycles+1;end
                if(data_valid && data_ready) stall_cycles=0;
                @(negedge clk);cycles=cycles+1;
                check(!cmd_ready,"busy excludes new command");
            end
            if(cycles>max_cycles) max_cycles=cycles;
            check(rsp_valid,"response timeout");check(rsp_status==expected_status,"response status");
            check(received==expected_count,"total stream count");
            if(expected_status==0) begin check(reads==120,"all 120 observations read");numeric(rsp_cost_fp64,expected_cost,2e-9);end
            else check(rsp_cost_fp64===64'h7ff0000000000000,"failed cost is +Inf");
            cost_snapshot=rsp_cost_fp64;status_snapshot=rsp_status;cmd_valid=1;data_ready=1;
            repeat(7) begin @(negedge clk);check(rsp_valid && !cmd_ready && !data_valid && !point_rd_en && rsp_cost_fp64===cost_snapshot && rsp_status===status_snapshot,"response backpressure");end
            cmd_valid=0;rsp_ready=1;@(negedge clk);rsp_ready=0;monitor=0;
            check(cmd_ready && !rsp_valid,"completion consumed");
        end
    endtask
    initial begin
        report=$fopen("residual_engine_results.txt","w");fd=$fopen("../../../data/calibration/residual_engine_vectors.txt","r");
        if(report==0 || fd==0) $fatal(1,"cannot open vectors/results");
        reset_dut();
        while(!$feof(fd)) begin load_vector();if(rc==9) begin launch();wait_result();cases=cases+1;end end
        $fclose(fd);
        fd=$fopen("../../../data/calibration/residual_engine_vectors.txt","r");load_vector();$fclose(fd);
        abort_test=1;phase=1;
        launch();repeat(30) @(negedge clk);reset_dut();
        repeat(2000) begin @(negedge clk);check(!rsp_valid && !data_valid && !point_rd_en,"no stale result after reset");end
        protocol_cases=protocol_cases+1;
        // 复位发生在 RAM 已返回、点投影计算进行中。
        phase=2;launch();cycles=0;
        while(reads==0 && cycles<10000) begin @(negedge clk);cycles=cycles+1;end
        check(reads==1,"reached point calculation");repeat(70) @(negedge clk);reset_dut();
        protocol_cases=protocol_cases+1;
        // 复位发生在残差输出背压中。
        phase=3;launch();cycles=0;
        while(!data_valid && cycles<10000) begin @(negedge clk);cycles=cycles+1;end
        check(data_valid,"reached stalled stream");repeat(6) @(negedge clk);reset_dut();
        protocol_cases=protocol_cases+1;
        phase=4;abort_test=0;launch();wait_result();protocol_cases=protocol_cases+1;
        $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d checks=%0d errors=%0d max_cycles=%0d",cases,protocol_cases,checks,errors,max_cycles);
        $fclose(report);done=1;
    end
endmodule
