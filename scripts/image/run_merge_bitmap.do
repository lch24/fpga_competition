transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_merge_bitmap/finished]
 set cases [examine -radix unsigned /tb_merge_bitmap/cases]
 set checks [examine -radix unsigned /tb_merge_bitmap/checks]
 if {$done!=1 || $cases!=6 || $checks!=29} {quit -code 1 -f}
 echo "PASS merge bitmap: $cases jobs, $checks exact centroids, stalls, depth boundary and reset retry"
 quit -code 0 -f
}
run -all
