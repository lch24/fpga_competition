transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_board_flow/finished]!=1} {quit -code 1 -f}
 echo "PASS board flow: three views, backpressure, frame ownership, success and failure"
 quit -code 0 -f
}
run -all
