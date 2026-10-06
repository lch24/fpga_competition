transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 if {[examine -radix unsigned /tb_undistort/finished]!=1} {quit -code 1 -f}
 echo "PASS tb_undistort"
 quit -code 0 -f
}
run -all
