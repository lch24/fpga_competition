transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 1000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_gauss_solver/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_gauss_solver/done]
set failures [examine -radix unsigned /tb_gauss_solver/errors]
set count [examine -radix unsigned /tb_gauss_solver/cases]
set protocols [examine -radix unsigned /tb_gauss_solver/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 45 || $protocols != 8} {
    echo "GAUSS_SOLVER_FAIL done=$completed matrices=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "GAUSS_SOLVER_PASS matrices=$count protocols=$protocols errors=$failures"
quit -code 0 -f
