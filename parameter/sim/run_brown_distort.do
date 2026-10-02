transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 3000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_brown_distort/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_brown_distort/done]
set failures [examine -radix unsigned /tb_brown_distort/errors]
set count [examine -radix unsigned /tb_brown_distort/cases]
set protocols [examine -radix unsigned /tb_brown_distort/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 260 || $protocols != 6} {
    echo "BROWN_DISTORT_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "BROWN_DISTORT_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
