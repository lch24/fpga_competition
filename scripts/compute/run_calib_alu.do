transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 1000} {incr tick} {
 run 100 us
 if {[examine -radix unsigned /tb_calib_alu/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_calib_alu/done]
set failures [examine -radix unsigned /tb_calib_alu/errors]
set count [examine -radix unsigned /tb_calib_alu/checked]
if {$completed != 1 || $failures != 0 || $count < 1000} {
 echo "CALIB_ALU_FAIL done=$completed errors=$failures vectors=$count"
 quit -code 1 -f
}
echo "CALIB_ALU_PASS vectors=$count errors=$failures"
quit -code 0 -f
