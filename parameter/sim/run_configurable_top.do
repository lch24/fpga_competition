onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
for {set tick 0} {$tick < 10000} {incr tick} {
 run 1 ms
 if {$tick % 250 == 0} {echo "CONFIG_TOP_PROGRESS cycles=[examine -radix unsigned /tb_configurable_top/cycles] errors=[examine -radix unsigned /tb_configurable_top/errors]"}
 if {[examine /tb_configurable_top/done] == 1} {break}
}
if {[examine /tb_configurable_top/done] != 1 || [examine -radix unsigned /tb_configurable_top/errors] != 0 || [examine -radix unsigned /tb_configurable_top/cases] != 3} {quit -code 1 -f}
echo CONFIGURABLE_TOP_PASS
quit -code 0 -f
