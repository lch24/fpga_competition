onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_grid_validate_serial/finished]
 set count [examine -radix unsigned /tb_grid_validate_serial/checked]
 if {$done!=1 || $count!=38} {quit -code 1 -f}
 set maxcycles [examine -radix unsigned /tb_grid_validate_serial/max_job_cycles]
 set totalcycles [examine -radix unsigned /tb_grid_validate_serial/total_job_cycles]
 echo "PASS grid validate serial cases=$count max_job_cycles=$maxcycles total_job_cycles=$totalcycles"
 quit -code 0 -f
}
run -all
