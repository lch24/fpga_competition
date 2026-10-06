transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 30000} {incr tick} {
 run 100 us
 if {[examine -radix unsigned /tb_validate_result/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_validate_result/done]
set failures [examine -radix unsigned /tb_validate_result/errors]
set cases [examine -radix unsigned /tb_validate_result/cases]
set protocols [examine -radix unsigned /tb_validate_result/protocol_cases]
if {$completed != 1 || $failures != 0 || $cases != 66 || $protocols != 6} {
 echo "VALIDATE_RESULT_FAIL done=$completed cases=$cases protocols=$protocols errors=$failures"
 quit -code 1 -f
}
echo "VALIDATE_RESULT_PASS cases=$cases protocols=$protocols errors=$failures"
quit -code 0 -f
