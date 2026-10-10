transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 1000} {incr tick} {
 run 100 us
 if {[examine -radix unsigned /tb_calib_engine/done] == 1} {break}
}
set completed [examine -radix unsigned /tb_calib_engine/done]
set failures [examine -radix unsigned /tb_calib_engine/errors]
set count [examine -radix unsigned /tb_calib_engine/checked]
if {$completed != 1 || $failures != 0 || $count < 100} {
 echo "CALIB_ENGINE_FAIL done=$completed errors=$failures actions=$count"
 quit -code 1 -f
}
echo "CALIB_ENGINE_PASS actions=$count errors=$failures"
quit -code 0 -f
