transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_vision_numeric/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_vision_numeric"
 echo "NUMERIC_CYCLES [examine -radix unsigned /tb_vision_numeric/cycles] RMS [examine -radix hex /tb_vision_numeric/result_rms]"
 quit -code 0 -f
}
run -all
