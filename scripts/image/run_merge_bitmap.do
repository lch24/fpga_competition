transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_merge_bitmap/finished]
 set cases [examine -radix unsigned /tb_merge_bitmap/cases]
 set checks [examine -radix unsigned /tb_merge_bitmap/checks]
 if {$done!=1 || $cases!=7 || $checks!=33} {quit -code 1 -f}
 echo "PASS merge bitmap: $cases jobs, $checks exact centroids, stalls, depth boundary, raster early-exit and reset retry"
 quit -code 0 -f
}
run -all
