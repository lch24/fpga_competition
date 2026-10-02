transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 3000} {incr tick} {
    run 100 us
    if {[examine -radix unsigned /tb_jacobi_eigen/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_jacobi_eigen/done]
set failures [examine -radix unsigned /tb_jacobi_eigen/errors]
set count [examine -radix unsigned /tb_jacobi_eigen/cases]
set protocols [examine -radix unsigned /tb_jacobi_eigen/protocol_cases]
if {$completed != 1 || $failures != 0 || $count != 30 || $protocols != 8} {
    echo "JACOBI_EIGEN_FAIL done=$completed matrices=$count protocols=$protocols errors=$failures"
    quit -code 1 -f
}
echo "JACOBI_EIGEN_PASS matrices=$count protocols=$protocols limit_cases=2 errors=$failures"
quit -code 0 -f
