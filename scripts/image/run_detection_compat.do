transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set count [examine -radix unsigned /tb_detection_compat/test/observed]
 set phase [examine -radix unsigned /tb_detection_compat/test/debug_phase]
 set errors [examine -radix unsigned /tb_detection_compat/test/violations]
 echo "DETECTION_RESULT corners=$count phase=$phase errors=$errors"
 set cycles [examine -radix unsigned /tb_detection_compat/test/cycles]
 echo "DETECTION_CYCLES $cycles"
 if {$count!=120 || $phase!=10 || $errors!=0} {quit -code 1 -f}
 echo "PASS detection compatibility"
 quit -code 0 -f
}
run -all
