// 模块内部包含；无全局include guard，函数在各模块内独立定义。
// 精确表示 value/2，供整数/半整数棋盘坐标使用；不依赖real或浮点IP。
function [63:0] board_half;
    input integer value;
    integer k,top;
    reg [31:0] mag;
    reg [63:0] shifted;
    reg [10:0] exponent;
    begin
        mag=(value<0)?-value:value;top=0;
        for(k=0;k<32;k=k+1)if(mag[k])top=k;
        shifted={32'b0,mag}<<(52-top);exponent=1022+top;
        board_half=(mag==0)?64'b0:{(value<0),exponent,shifted[51:0]};
    end
endfunction
function [63:0] board_x;
    input integer index;
    integer k;
    begin
        // Valid callers use 0..PAR_POINTS-1. Coordinates are compile-time
        // constants: infer a lookup table, not a signed hardware remainder.
        board_x=0;
        for(k=0;k<`PAR_POINTS;k=k+1)
            if(index==k)board_x=board_half(2*(k % `PAR_BOARD_COLS)-(`PAR_BOARD_COLS-1));
    end
endfunction
function [63:0] board_y;
    input integer index;
    integer k;
    begin
        board_y=0;
        for(k=0;k<`PAR_POINTS;k=k+1)
            if(index==k)board_y=board_half(2*(k / `PAR_BOARD_COLS)-(`PAR_BOARD_ROWS-1));
    end
endfunction
