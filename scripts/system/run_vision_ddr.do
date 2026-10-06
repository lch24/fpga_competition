transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_vision_ddr/finished]
 set checks [examine -radix unsigned /tb_vision_ddr/checks]
 set errors [examine -radix unsigned /tb_vision_ddr/violations]
 if {$done!=1 || $checks!=12 || $errors!=0} {quit -code 1 -f}
 echo "PASS vision DDR: $checks jobs, DDR workspace overlap rejection, pixels, errors and retry"
 quit -code 0 -f
}
run -all
