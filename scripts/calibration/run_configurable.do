onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 1600} {incr tick} {
 run 1 ms
 if {$tick % 100 == 0} {echo "CONFIG_PROGRESS cycles=[examine -radix unsigned /tb_configurable/cycles] errors=[examine -radix unsigned /tb_configurable/errors]"}
 if {[examine /tb_configurable/done] == 1} {break}
}
if {[examine /tb_configurable/done] != 1 || [examine -radix unsigned /tb_configurable/errors] != 0} {quit -code 1 -f}
echo "CONFIG_DONE cycles=[examine -radix unsigned /tb_configurable/cycles] seeds=[examine -radix unsigned /tb_configurable/seeds] residuals=[examine -radix unsigned /tb_configurable/seen]"
echo CONFIGURABLE_PASS
quit -code 0 -f
