// Include inside a module: exact unsigned 16-bit integer -> FP64 conversion.
// Shared source only; each caller retains its original combinational function.
// No include guard: Verilog functions have module-local scope.
function [63:0] u16;
    input [15:0] x;
    integer k,top;
    reg [63:0] shifted;
    reg [10:0] exponent;
    begin
        top = 0;
        for (k = 0;k<16;k = k+1)if (x[k])top = k;
        shifted = {48'd0,x}<<(52-top);
        exponent = 1023+top;
        u16 = x == 0?64'd0:{1'b0,exponent,shifted[51:0]};
    end
endfunction
