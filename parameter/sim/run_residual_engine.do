transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 3000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_residual_engine/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_residual_engine/done]
set failures [examine -radix unsigned /tb_residual_engine/errors]
set count [examine -radix unsigned /tb_residual_engine/cases]
set protocols [examine -radix unsigned /tb_residual_engine/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 49 || $protocols != 4} {
    echo "RESIDUAL_ENGINE_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "RESIDUAL_ENGINE_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
