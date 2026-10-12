transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_board_display_switch/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_board_display_switch"
 quit -code 0 -f
}
run -all
