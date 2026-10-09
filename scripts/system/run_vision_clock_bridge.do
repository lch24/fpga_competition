transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_vision_clock_bridge/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_vision_clock_bridge"
 quit -code 0 -f
}
run -all
