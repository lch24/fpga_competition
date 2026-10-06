onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_grid_validate_serial/finished]
 set count [examine -radix unsigned /tb_grid_validate_serial/checked]
 if {$done!=1 || $count!=6} {quit -code 1 -f}
 echo "PASS grid validate serial cases=$count"
 quit -code 0 -f
}
run -all
