onfinish stop
onerror {quit -code 1 -f}
onbreak {resume}
run -all
if {[examine /tb_detection_capture/finished] != 1 || [examine -radix unsigned /tb_detection_capture/checked] != 5} {quit -code 1 -f}
echo "PASS detection capture: empty/short/exact/overflow/drain/restart"
quit -code 0 -f
