transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_replay_pyramid/finished]
 set checks [examine -radix unsigned /tb_replay_pyramid/checks]
 if {$done!=1 || $checks!=2} {quit -code 1 -f}
 echo "PASS replay pyramid: two native layers and explicit dump rejection"
 quit -code 0 -f
}
run -all
