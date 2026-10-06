transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_candidate_cache/finished]
 set checks [examine -radix unsigned /tb_candidate_cache/checked]
 set faults [examine -radix unsigned /tb_candidate_cache/faults]
 set errors [examine -radix unsigned /tb_candidate_cache/violations]
 if {$done!=1 || $checks!=324 || $faults!=3 || $errors!=0} {quit -code 1 -f}
 echo "PASS candidate DDR cache: $checks accesses, $faults recovery cases, zero protocol errors"
 quit -code 0 -f
}
run -all
