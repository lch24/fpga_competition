`include "calib_defs.vh"

/*
配置：统一见 rtl/include/calib_config.vh；下文具体计数例子以默认3张5×8为例，
实际容量、循环边界和端口位宽由PAR_*派生，修改配置后须重新编译/综合。
作用：按3张图分别保存40个FP32二维角点，并保留各图检测结果用于计算和调试。
实现：120x64位单写单读RAM、每图计数/状态寄存器；无浮点运算器。
只检查FP32指数是否为全1以拒绝NaN/Inf；坐标范围由后续算法检查。

任务边界：clear为同步单拍，清除计数/状态/读有效，不要求清零RAM数据。
clear优先于写入、检测响应与读取；clear期间corner_ready/view_rsp_ready/rd_valid为0。
任务编号由calib_top检查，缓存不保存job_id，也不再提供整次收集的rsp握手。
calib_top只在收集阶段转发写入/检测响应，计算期间禁止修改缓存。

角点：按view=0,1,2顺序，每图point_index=0..39；valid&&ready才接收。
检测响应：每图恰好一次，失败时点数不足也必须接收；成功在最后一点握手后发送。
同图角点与响应同时有效时，角点握手优先，响应背压一拍；上游须保持响应。
只有当前图完成响应被接收后才推进到下一图；提前发送其他图会记录该图格式错误。
格式错误后该图不再接收角点，但仍可接收检测响应；done后角点/重复响应均背压。
40点收齐但尚未done时，多余点仍被接收并记格式错误，计数不超过40。
view_done仅表示已接收检测结束通知，view_status原样保存检测状态，仅done=1时解释。
view_point_count统计通过顺序/last/有限性检查并写入的点数，0..40。
view_format_error记录本图流格式错误或成功响应时点数不完整；错误后保持至clear。
view_usable[v]=done[v] && status[v]==0 && count[v]==40 && !format_error[v]。
非法view_id不能截断为有效编号：由calib_top在转发前拒绝并报BAD_CONFIG。
缓存自身对非法view_id压低ready，不索引RAM/状态；没有归属图可记录该错误。
失败后保留已有计数、状态和数据，其他图的usable不清除；顶层禁止启动整次标定。
读取不清状态，任务结束也不清状态，直到下一次clear/复位，便于调试。

读取：rd_en在上升沿采样，地址=view_id*40+point_index；不设ready，固定1拍返回。
精确定义：E_n采样rd_en/地址，坐标与rd_valid在E_n之后有效，调用方在E_(n+1)采样。
调用方须保证view_id<3、point_index<40且该图usable；非法读取属于设计错误，仿真断言检查。
防御行为：非法/不可用视图读取不产生rd_valid，不暴露旧任务RAM数据。
首版一次一笔读取，收到rd_valid后才发下一笔；调用方发起前预留接收寄存器。
无返回背压，必须在rd_valid有效的周期消费；rd_valid=0时数据不解释。
clear/复位取消在途读；复位后所有状态与valid清零，rst_n同步释放。
*/
module corner_store (
    input wire clk, // core_clk
    input wire rst_n, // 低有效复位，同步释放
    input wire clear, // 同步清空本次收集状态，不清RAM内容
    input wire corner_valid, // 角点输入有效
    output wire corner_ready, // 可接收角点；背压时上游保持载荷
    input wire [7:0] corner_view_id, // 0..PAR_VIEWS-1，保持团队输入位宽
    input wire [7:0] corner_point_index, // 0..PAR_POINTS-1，保持团队输入位宽
    input wire [31:0] corner_x_fp32, // 原图像素x
    input wire [31:0] corner_y_fp32, // 原图像素y
    input wire corner_last, // 仅本图第39号点为1
    input wire view_rsp_valid, // 单张图检测完成通知
    output wire view_rsp_ready, // 失败时不等待收齐40点
    input wire [7:0] view_rsp_view_id, // 0..PAR_VIEWS-1
    input wire [7:0] view_rsp_status, // 原始检测状态，0成功
    output wire [`PAR_VIEWS-1:0] view_done, // bit v：第v图已收到检测完成通知
    output wire [8*`PAR_VIEWS-1:0] view_status, // 第v图占[8*v +: 8]；done=0时不解释
    output wire [`PAR_POINT_BITS*`PAR_VIEWS-1:0] view_point_count, // 第v图占[PAR_POINT_BITS*v +: PAR_POINT_BITS]，范围0..PAR_POINTS
    output wire [`PAR_VIEWS-1:0] view_format_error, // bit v：本图输入格式错误，保持到clear
    output wire [`PAR_VIEWS-1:0] view_usable, // bit v：本图40点完整且检测成功、无格式错误
    input wire rd_en, // 同步读使能，无ready
    input wire [`PAR_VIEW_BITS-1:0] rd_view_id, // 0..PAR_VIEWS-1，图像编号
    input wire [`PAR_POINT_BITS-1:0] rd_point_index, // 0..PAR_POINTS-1，图内角点编号
    output wire rd_valid, // 固定1拍读有效，无返回背压
    output wire [31:0] rd_x_fp32, // x，仅rd_valid时有效
    output wire [31:0] rd_y_fp32 // y，仅rd_valid时有效
);
    reg [63:0] point_mem [0:`PAR_TOTAL_POINTS-1];
    reg [63:0] rd_data_q;
    reg rd_valid_q;
    reg [`PAR_VIEWS-1:0] done_q;
    reg [8*`PAR_VIEWS-1:0] status_q;
    reg [`PAR_VIEWS-1:0] format_error_q;
    reg [`PAR_POINT_BITS-1:0] count_q [0:`PAR_VIEWS-1];
    reg [`PAR_VIEW_BITS-1:0] active_view_q; // PAR_VIEWS表示全部视图已结束
    integer v;

    wire corner_id_ok = (corner_view_id < `PAR_VIEWS);
    wire rsp_id_ok = (view_rsp_view_id < `PAR_VIEWS);
    wire corner_fire = corner_valid && corner_ready;
    wire rsp_fire = view_rsp_valid && view_rsp_ready;
    wire finite_xy = (corner_x_fp32[30:23] != 8'hff) &&
                     (corner_y_fp32[30:23] != 8'hff);
    wire corner_shape_ok = corner_id_ok &&
        (corner_view_id == active_view_q) &&
        (corner_point_index < `PAR_POINTS) &&
        (corner_point_index == count_q[corner_view_id]) &&
        (corner_last == (corner_point_index == (`PAR_POINTS-1))) && finite_xy;
    wire store_write = corner_fire && corner_shape_ok;
    wire [`PAR_ADDR_BITS-1:0] wr_addr = corner_view_id * `PAR_POINTS + corner_point_index;
    wire [`PAR_ADDR_BITS-1:0] rd_addr = rd_view_id * `PAR_POINTS + rd_point_index;
    wire read_ok = rd_en && (rd_view_id < `PAR_VIEWS) &&
                   (rd_point_index < `PAR_POINTS) && view_usable[rd_view_id];

    assign corner_ready = rst_n && !clear && corner_id_ok &&
                          !done_q[corner_view_id] && !format_error_q[corner_view_id];
    assign view_rsp_ready = rst_n && !clear && rsp_id_ok &&
                            !done_q[view_rsp_view_id] &&
                            !(corner_fire && (corner_view_id == view_rsp_view_id));
    assign view_done = done_q;
    assign view_status = status_q;
    assign view_format_error = format_error_q;
    // 计数按view低位优先打包，位宽随每图容量推导。
    assign rd_valid = rst_n && !clear && rd_valid_q;
    assign rd_x_fp32 = rd_data_q[31:0];
    assign rd_y_fp32 = rd_data_q[63:32];

    genvar g;
    generate
        for (g = 0; g < `PAR_VIEWS; g = g + 1) begin : view_flags
            assign view_point_count[`PAR_POINT_BITS*g +: `PAR_POINT_BITS] = count_q[g];
            assign view_usable[g] = done_q[g] && (status_q[8*g +: 8] == `PAR_OK) &&
                                    (count_q[g] == `PAR_POINTS) && !format_error_q[g];
        end
    endgenerate

    // RAM本体不复位/清零，保留同步读模板以便综合推断存储资源。
    // 读数据仅在rd_valid时有意义；控制状态阻止读取旧任务内容。
    always @(posedge clk) begin
        if (store_write)
            point_mem[wr_addr] <= {corner_y_fp32, corner_x_fp32};
        if (rst_n && !clear && read_ok)
            rd_data_q <= point_mem[rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_q <= 3'b000;
            status_q <= 24'd0;
            format_error_q <= 3'b000;
            active_view_q <= 2'd0;
            rd_valid_q <= 1'b0;
            for (v = 0; v < `PAR_VIEWS; v = v + 1)
                count_q[v] <= 6'd0;
        end else if (clear) begin
            done_q <= 3'b000;
            status_q <= 24'd0;
            format_error_q <= 3'b000;
            active_view_q <= 2'd0;
            rd_valid_q <= 1'b0;
            for (v = 0; v < `PAR_VIEWS; v = v + 1)
                count_q[v] <= 6'd0;
        end else begin
            rd_valid_q <= read_ok;
            if (corner_fire) begin
                if (corner_shape_ok)
                    count_q[corner_view_id] <= count_q[corner_view_id] + 6'd1;
                else
                    format_error_q[corner_view_id] <= 1'b1;
            end
            if (rsp_fire) begin
                done_q[view_rsp_view_id] <= 1'b1;
                status_q[8*view_rsp_view_id +: 8] <= view_rsp_status;
                if ((view_rsp_view_id != active_view_q) ||
                    ((view_rsp_status == `PAR_OK) && (count_q[view_rsp_view_id] != `PAR_POINTS)))
                    format_error_q[view_rsp_view_id] <= 1'b1;
                if (view_rsp_view_id == active_view_q)
                    active_view_q <= active_view_q + 2'd1;
            end
        end
    end
endmodule
