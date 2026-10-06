`timescale 1ns/1ps
`define PAR_POINTS (ROWS*COLS)
`define PAR_BOARD_ROWS ROWS
`define PAR_BOARD_COLS COLS
module geometry_case #(parameter ROWS=5,COLS=8)(output reg done=0);
 `include "calib_geometry.vh"
 integer point;
 real x,y;
 initial begin
  for(point=0;point<ROWS*COLS;point=point+1)begin
   x=(2.0*(point%COLS)-(COLS-1))/2.0;
   y=(2.0*(point/COLS)-(ROWS-1))/2.0;
   if(board_x(point)!==$realtobits(x) || board_y(point)!==$realtobits(y))
    $fatal(1,"board lookup mismatch rows=%0d cols=%0d index=%0d",ROWS,COLS,point);
  end
  done=1;
 end
endmodule
module tb_calib_geometry;
 wire a,b,c;
 reg finished=0;
 integer checked=0;
 geometry_case #(.ROWS(5),.COLS(8)) d0(a);
 geometry_case #(.ROWS(6),.COLS(7)) d1(b);
 geometry_case #(.ROWS(2),.COLS(128)) d2(c);
 initial begin wait(a&&b&&c);checked=338;finished=1;#1;$finish;end
 initial begin #1000;$fatal(1,"geometry timeout");end
endmodule
`undef PAR_POINTS
`undef PAR_BOARD_ROWS
`undef PAR_BOARD_COLS
