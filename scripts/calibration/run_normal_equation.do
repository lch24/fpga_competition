transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 200000} {incr tick} {
 run 1 ms
 if {[examine -radix unsigned /tb_normal_equation/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_normal_equation/done]
set failures [examine -radix unsigned /tb_normal_equation/errors]
set cases [examine -radix unsigned /tb_normal_equation/cases]
if {$completed != 1 || $failures != 0 || $cases != 15} {echo "FAIL done=$completed errors=$failures cases=$cases";quit -code 1 -f}
echo "NORMAL_EQUATION_PASS cases=$cases errors=$failures"
quit -code 0 -f
