transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set count [examine -radix unsigned /tb_fp64_add_bounds/checks]
 set errors [examine -radix unsigned /tb_fp64_add_bounds/errors]
 set done [examine -radix unsigned /tb_fp64_add_bounds/done]
 echo "BOUNDS_RESULT checks=$count errors=$errors done=$done"
 if {$count!=12 || $errors!=0 || $done!=1} {quit -code 1 -f}
 echo "PASS tb_fp64_add_bounds"
 quit -code 0 -f
}
run -all
