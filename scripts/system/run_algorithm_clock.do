transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_algorithm_clock/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_algorithm_clock"
 quit -code 0 -f
}
run -all
