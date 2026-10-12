transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_board_flow/finished]!=1} {quit -code 1 -f}
 echo "PASS board flow: manual slots, commands, display drain, failure/retry, reset"
 quit -code 0 -f
}
run -all
