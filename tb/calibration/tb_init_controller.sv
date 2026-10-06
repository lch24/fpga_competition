`timescale 1ns/1ps
// 真正的corner_store + 全部init子模块；仅3个专门场景强制子核状态以检查调度容错。
module tb_init_controller;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,cmd_valid=0,rsp_ready=0,seed_ready=0;
    wire cmd_ready,rsp_valid,seed_valid;wire [7:0] rsp_status;
    wire [2:0] seed_id,rsp_seed_count;wire [1727:0] seed_state;
    reg [15:0] width,height,cmd_width,cmd_height;
    wire read_en,read_valid;wire [1:0] read_view;wire [5:0] read_index;wire [31:0] read_x,read_y;
    reg clear=0,corner_valid=0,view_rsp_valid=0,corner_last=0;
    reg [7:0] corner_view,corner_index,view_rsp_view;
    reg [31:0] corner_x,corner_y;
    wire corner_ready,view_rsp_ready;wire [2:0] usable;
    reg [7679:0] points;
    reg [1727:0] expected[0:4];reg [7:0] expected_status;
    integer mode=0,expected_count,expected_mask,seen_mask=0,received=0,reads=0;
    integer errors=0,cases=0,protocol_cases=0,cycles=0,max_cycles=0;
    integer fd,report,rc,i,j,v,stall_cycles=0;
    reg done=0,monitor=0,held=0;
    reg [1727:0] held_state;reg [2:0] held_id;
    reg [7:0] held_status;reg [2:0] held_count;
    reg [31:0] random_state=32'h652fab19;
    real av,ev,difference,tolerance;
    init_controller dut(.clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),
        .cmd_width(cmd_width),.cmd_height(cmd_height),
        .point_rd_en(read_en),.point_rd_view_id(read_view),.point_rd_index(read_index),
        .point_rd_valid(read_valid),.point_rd_x_fp32(read_x),.point_rd_y_fp32(read_y),
        .seed_valid(seed_valid),.seed_ready(seed_ready),.seed_id(seed_id),.seed_state(seed_state),
        .rsp_valid(rsp_valid),.rsp_ready(rsp_ready),.rsp_status(rsp_status),.rsp_seed_count(rsp_seed_count));
    corner_store store(.clk(clk),.rst_n(rst_n),.clear(clear),
        .corner_valid(corner_valid),.corner_ready(corner_ready),.corner_view_id(corner_view),
        .corner_point_index(corner_index),.corner_x_fp32(corner_x),.corner_y_fp32(corner_y),.corner_last(corner_last),
        .view_rsp_valid(view_rsp_valid),.view_rsp_ready(view_rsp_ready),.view_rsp_view_id(view_rsp_view),.view_rsp_status(8'b0),
        .view_done(),.view_status(),.view_point_count(),.view_format_error(),.view_usable(usable),
        .rd_en(read_en),.rd_view_id(read_view),.rd_point_index(read_index),.rd_valid(read_valid),.rd_x_fp32(read_x),.rd_y_fp32(read_y));
    // 定向错误注入保留真实子核计算与握手，只覆盖完成状态，验证父模块的跳过规则。
    always @(negedge clk) begin
        if(mode==1)force dut.z_status=8'd4;else release dut.z_status;
        if(mode==3 || (mode==2 && dut.seed==2))force dut.p_status=8'd4;else release dut.p_status;
    end
    task check;input condition;input [511:0] label;
        begin if(condition!==1'b1)begin errors=errors+1;if(errors<40)$fdisplay(report,"FAIL case=%0d %0s cycle=%0d",cases,label,cycles);end end
    endtask
    always @(posedge clk) begin
        if(rst_n && monitor)begin
            if(held)check(seed_valid && seed_id===held_id && seed_state===held_state,"seed stable under backpressure");
            held=seed_valid && !seed_ready;held_id=seed_id;held_state=seed_state;
            if(read_en)begin
                check(read_view==reads/40 && read_index==reads%40,"RAM address order");reads=reads+1;
            end
            if(seed_valid)check(!rsp_valid && !cmd_ready,"seed precedes response and keeps busy");
            if(seed_valid && seed_ready)begin
                check(seed_id<5,"seed id bounds");
                if(seed_id<5)begin
                    check((expected_mask & (1<<seed_id))!=0 && (seen_mask & (1<<seed_id))==0,"expected unique seed id");
                    check((seen_mask >> seed_id)==0,"seed IDs increasing");
                    for(j=0;j<27;j=j+1)begin
                        av=$bitstoreal(seed_state[64*j +:64]);ev=$bitstoreal(expected[seed_id][64*j +:64]);
                        difference=av-ev;if(difference<0)difference=-difference;
                        tolerance=3e-7*(1+(ev<0?-ev:ev));
                        if(!((^seed_state[64*j +:64])!==1'bx && seed_state[64*j+52 +:11]!=2047 && difference<=tolerance))begin
                            check(0,"seed numeric result");$fdisplay(report,"seed=%0d item=%0d actual=%.17g expected=%.17g diff=%.9g",seed_id,j,av,ev,difference);
                        end
                    end
                    seen_mask=seen_mask | (1<<seed_id);
                end
                received=received+1;
            end
        end else held=0;
    end
    task load_vector;
        begin rc=$fscanf(fd,"%d %d %d %d %d %h %h %h %h %h %h %d\n",expected_status,width,height,mode,expected_count,points,
            expected[0],expected[1],expected[2],expected[3],expected[4],expected_mask);end
    endtask
    task reset_dut;
        begin @(negedge clk);monitor=0;rst_n=0;cmd_valid=0;rsp_ready=0;seed_ready=0;corner_valid=0;view_rsp_valid=0;clear=0;mode=0;
            repeat(3)@(negedge clk);check(!cmd_ready && !rsp_valid && !seed_valid && !read_en,"reset clears producer valid");
            rst_n=1;@(negedge clk);check(cmd_ready,"reset recovery");end
    endtask
    task fill_store;
        begin
            @(negedge clk);clear=1;@(negedge clk);clear=0;
            for(v=0;v<3;v=v+1)begin
                for(i=0;i<40;i=i+1)begin
                    corner_view=v;corner_index=i;corner_last=(i==39);corner_x=points[(v*40+i)*64 +:32];corner_y=points[(v*40+i)*64+32 +:32];corner_valid=1;
                    #1;check(corner_ready,"store accepts ordered corner");@(negedge clk);
                end
                corner_valid=0;view_rsp_view=v;view_rsp_valid=1;#1;check(view_rsp_ready,"store accepts view completion");@(negedge clk);view_rsp_valid=0;
            end
            check(usable===3'b111,"three committed views");
        end
    endtask
    task launch;
        begin @(negedge clk);check(cmd_ready,"command ready");monitor=1;held=0;seen_mask=0;received=0;reads=0;stall_cycles=0;
            cmd_width=width;cmd_height=height;cmd_valid=1;seed_ready=0;
            @(negedge clk);cmd_valid=0;cmd_width=0;cmd_height=0;end
    endtask
    task await_response;
        begin
            cycles=0;
            while(!rsp_valid && cycles<4000000)begin
                // 每个seed先持续背压17拍，然后确定性伪随机ready。
                if(seed_valid)begin stall_cycles=stall_cycles+1;random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};seed_ready=stall_cycles>17 && random_state[0];end
                else begin stall_cycles=0;seed_ready=0;end
                @(negedge clk);cycles=cycles+1;check(!cmd_ready,"busy excludes command");
            end
            check(rsp_valid,"response timeout");if(!rsp_valid)$fatal(1,"timeout");if(cycles>max_cycles)max_cycles=cycles;
            seed_ready=0;check(rsp_status===expected_status,"response status");check(received==expected_count && rsp_seed_count==expected_count,"accepted seed count");check(seen_mask==expected_mask,"seed ID mask");
            if(expected_status==0)check(reads==120,"120 reads through real corner_store");
        end
    endtask
    task consume_response;
        begin
            held_status=rsp_status;held_count=rsp_seed_count;cmd_valid=1;
            repeat(7)begin @(negedge clk);check(rsp_valid && !cmd_ready && !seed_valid && !read_en && rsp_status===held_status && rsp_seed_count===held_count,"completion stable under stall");end
            cmd_valid=0;rsp_ready=1;@(negedge clk);rsp_ready=0;monitor=0;check(!rsp_valid && cmd_ready,"response consumed");
        end
    endtask
    initial begin
        report=$fopen("init_controller_results.txt","w");fd=$fopen("../../../data/calibration/init_controller_vectors.txt","r");if(!fd || !report)$fatal(1,"file open");reset_dut();
        while(!$feof(fd))begin load_vector();if(rc==12)begin fill_store();launch();await_response();consume_response();cases=cases+1;end else if(rc!=-1)$fatal(1,"malformed vector");end
        $fclose(fd);fd=$fopen("../../../data/calibration/init_controller_vectors.txt","r");load_vector();$fclose(fd);
        fill_store();launch();repeat(100)@(negedge clk);reset_dut();repeat(100)begin @(negedge clk);check(cmd_ready && !rsp_valid && !seed_valid,"no stale result after cancellation");end protocol_cases=protocol_cases+1;
        fill_store();launch();cycles=0;while(!seed_valid && cycles<4000000)begin @(negedge clk);cycles=cycles+1;end check(seed_valid,"reached blocked seed");repeat(5)@(negedge clk);reset_dut();protocol_cases=protocol_cases+1;
        fill_store();launch();await_response();consume_response();protocol_cases=protocol_cases+1;
        $fdisplay(report,"RESULT cases=%0d protocol_cases=%0d errors=%0d max_cycles=%0d",cases,protocol_cases,errors,max_cycles);$fclose(report);done=1;
    end
endmodule
