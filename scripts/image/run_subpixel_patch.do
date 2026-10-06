transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_subpixel_patch/finished]
 set cases [examine -radix unsigned /tb_subpixel_patch/cases]
 set checked [examine -radix unsigned /tb_subpixel_patch/checked]
 if {$done!=1 || $cases!=4 || $checked!=1644} {quit -code 1 -f}
 echo "PASS subpixel patch: $cases radii, $checked bytes, stalls and exact raster addresses"
 quit -code 0 -f
}
run -all
