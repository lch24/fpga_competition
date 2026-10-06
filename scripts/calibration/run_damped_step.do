transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 200000} {incr tick} {
 run 1 ms
 if {[examine -radix unsigned /tb_damped_step/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_damped_step/done]
set failures [examine -radix unsigned /tb_damped_step/errors]
set cases [examine -radix unsigned /tb_damped_step/cases]
if {$completed != 1 || $failures != 0 || $cases != 35} {echo "FAIL done=$completed errors=$failures cases=$cases";quit -code 1 -f}
echo "DAMPED_STEP_PASS cases=$cases errors=$failures"
quit -code 0 -f
