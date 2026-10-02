transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 3000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_project_point/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_project_point/done]
set failures [examine -radix unsigned /tb_project_point/errors]
set count [examine -radix unsigned /tb_project_point/cases]
set protocols [examine -radix unsigned /tb_project_point/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 129 || $protocols != 3} {
    echo "PROJECT_POINT_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "PROJECT_POINT_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
