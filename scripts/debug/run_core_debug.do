transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_core_debug/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_core_debug"
 quit -code 0 -f
}
run -all
