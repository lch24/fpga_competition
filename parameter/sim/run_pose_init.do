transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 20000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_pose_init/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_pose_init/done]
set failures [examine -radix unsigned /tb_pose_init/errors]
set count [examine -radix unsigned /tb_pose_init/cases]
set protocols [examine -radix unsigned /tb_pose_init/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 54 || $protocols != 3} {
    echo "POSE_INIT_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "POSE_INIT_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
