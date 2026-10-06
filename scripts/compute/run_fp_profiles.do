transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 10000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_fp_profiles/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_fp_profiles/done]
set failures [examine -radix unsigned /tb_fp_profiles/errors]
set count [examine -radix unsigned /tb_fp_profiles/checked]
if {$completed != 1 || $failures != 0 || $count != 63561} {
    echo "FP_PROFILES_FAIL done=$completed errors=$failures vectors=$count"
    quit -code 1 -f
}
echo "FP_PROFILES_PASS vectors=$count errors=$failures"
quit -code 0 -f
