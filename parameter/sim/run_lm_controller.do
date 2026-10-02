transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 200000} {incr tick} {
 run 1 ms
 if {[examine -radix unsigned /tb_lm_controller/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_lm_controller/done]
set failures [examine -radix unsigned /tb_lm_controller/errors]
set cases [examine -radix unsigned /tb_lm_controller/cases]
if {$completed != 1 || $failures != 0 || $cases != 21} {echo "FAIL done=$completed errors=$failures cases=$cases";quit -code 1 -f}
echo "LM_CONTROLLER_PASS cases=$cases errors=$failures"
quit -code 0 -f
