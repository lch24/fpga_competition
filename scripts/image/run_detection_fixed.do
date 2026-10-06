transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set count [examine -radix unsigned /tb_detection_fixed/test/observed]
 set phase [examine -radix unsigned /tb_detection_fixed/test/debug_phase]
 set errors [examine -radix unsigned /tb_detection_fixed/test/violations]
 set deviation [examine /tb_detection_fixed/test/max_corner_error]
 set cycles [examine -radix unsigned /tb_detection_fixed/test/cycles]
 echo "FIXED_DETECTION corners=$count phase=$phase errors=$errors max_error=$deviation cycles=$cycles"
 if {$count!=120 || $phase!=10 || $errors!=0 || $deviation>0.02} {quit -code 1 -f}
 echo "PASS fixed detection"
 quit -code 0 -f
}
run -all
