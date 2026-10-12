transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_debug_snapshot/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_debug_snapshot"
 quit -code 0 -f
}
run -all
