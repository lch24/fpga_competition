transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_board_video/finished]!=1} {
  echo "FAIL tb_board_video"
  quit -code 1 -f
 }
 echo "PASS tb_board_video"
 quit -code 0 -f
}
run -all
