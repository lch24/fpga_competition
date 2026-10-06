transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
# Poll in simulation time; stop immediately when both checkers finish.
for {set tick 0} {$tick < 10000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_fp_operator/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_fp_operator/done]
set failures [examine -radix unsigned /tb_fp_operator/errors]
set count [examine -radix unsigned /tb_fp_operator/checked]
if {$completed != 1 || $failures != 0 || $count == 0} {
    echo "FP_OPERATOR_FAIL done=$completed errors=$failures vectors=$count"
    quit -code 1 -f
}
echo "FP_OPERATOR_PASS vectors=$count errors=$failures"
quit -code 0 -f
