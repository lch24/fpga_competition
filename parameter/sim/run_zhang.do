transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 20000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_zhang/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_zhang/done]
set failures [examine -radix unsigned /tb_zhang/errors]
set count [examine -radix unsigned /tb_zhang/cases]
set protocols [examine -radix unsigned /tb_zhang/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 16 || $protocols != 4} {
    echo "ZHANG_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "ZHANG_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
