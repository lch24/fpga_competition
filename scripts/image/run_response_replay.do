transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set a [examine -radix unsigned /tb_response_replay/small_a/finished]
 set b [examine -radix unsigned /tb_response_replay/small_b/finished]
 if {$a!=1 || $b!=1} {quit -code 1 -f}
 echo "PASS response replay: 6 frames, two sizes, stalls and reset"
 quit -code 0 -f
}
run -all
