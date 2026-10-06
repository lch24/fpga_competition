transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 3000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_rotation/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_rotation/done]
set failures [examine -radix unsigned /tb_rotation/errors]
set count [examine -radix unsigned /tb_rotation/cases]
set protocols [examine -radix unsigned /tb_rotation/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 155 || $protocols != 3} {
    echo "ROTATION_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "ROTATION_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
