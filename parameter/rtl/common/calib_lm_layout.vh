// 在LM各模块内部包含，活动列顺序必须完全相同。
// 先优化4个内参和各视图6个外参；再依次加入k1、k2/p1/p2；k3仍固定0。
function integer active_index;
    input integer k;
    begin
        if(k<4)active_index=k;
        else if(k<`PAR_STAGE0_N)active_index=k+5;
        else active_index=k-6*`PAR_VIEWS;
    end
endfunction
function integer columns;
    input [1:0] stage;
    begin
        case(stage)
            0:columns=`PAR_STAGE0_N;
            1:columns=`PAR_STAGE1_N;
            default:columns=`PAR_ACTIVE_N;
        endcase
    end
endfunction
