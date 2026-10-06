transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine /tb_grid_shared/finished]!=1 || [examine -radix unsigned /tb_grid_shared/checked]!=48} {quit -code 1 -f}
 echo "PASS grid shared"
 quit -code 0 -f
}
run -all
