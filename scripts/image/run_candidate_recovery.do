transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_candidate_recovery/finished]
 set checks [examine -radix unsigned /tb_candidate_recovery/checked]
 if {$done!=1 || $checks!=2} {quit -code 1 -f}
 echo "PASS candidate/gray concurrent error drain and retry without reset"
 quit -code 0 -f
}
run -all
