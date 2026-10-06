transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_fp64_sqrt_serial/finished]
 set count [examine -radix unsigned /tb_fp64_sqrt_serial/checked]
 set resets [examine -radix unsigned /tb_fp64_sqrt_serial/resets]
 if {$done!=1 || $count!=7498 || $resets!=5} {quit -code 1 -f}
 echo "PASS sqrt serial vectors=$count resets=$resets"
 quit -code 0 -f
}
run -all
