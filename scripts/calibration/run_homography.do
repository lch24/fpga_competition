transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 20000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_homography/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_homography/done]
set failures [examine -radix unsigned /tb_homography/errors]
set count [examine -radix unsigned /tb_homography/cases]
set protocols [examine -radix unsigned /tb_homography/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 22 || $protocols != 4} {
    echo "HOMOGRAPHY_FAIL done=$completed cases=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "HOMOGRAPHY_PASS cases=$count protocols=$protocols errors=$failures"
quit -code 0 -f
