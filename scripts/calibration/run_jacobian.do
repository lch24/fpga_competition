transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 200000} {incr tick} {
 run 1 ms
 if {[examine -radix unsigned /tb_jacobian/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_jacobian/done]
set failures [examine -radix unsigned /tb_jacobian/errors]
set cases [examine -radix unsigned /tb_jacobian/cases]
if {$completed != 1 || $failures != 0 || $cases != 12} {echo "FAIL done=$completed errors=$failures cases=$cases";quit -code 1 -f}
echo "JACOBIAN_PASS cases=$cases errors=$failures"
quit -code 0 -f
