transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 16000} {incr tick} {
 run 1 ms
 if {$tick % 250 == 0} {echo "TOP_PROGRESS cycles=[examine -radix unsigned /tb_calib_top/cycles] errors=[examine -radix unsigned /tb_calib_top/errors]"}
 if {[examine -radix unsigned /tb_calib_top/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_calib_top/done]
set failures [examine -radix unsigned /tb_calib_top/errors]
set cases [examine -radix unsigned /tb_calib_top/cases]
set control [examine -radix unsigned /tb_calib_top/CONTROL_ONLY]
set real_input [examine -radix unsigned /tb_calib_top/REAL_INPUT]
set expected [expr {$real_input ? 1 : ($control ? 40 : 2)}]
if {$completed != 1 || $failures != 0 || $cases != $expected} {echo "FAIL done=$completed errors=$failures cases=$cases expected=$expected";quit -code 1 -f}
echo "CALIB_TOP_PASS control=$control cases=$cases errors=$failures"
quit -code 0 -f
