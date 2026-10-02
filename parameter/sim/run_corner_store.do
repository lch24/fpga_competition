transcript on
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
log -r /*
run 110 us
set completed [examine -radix unsigned /tb_corner_store/tb_done]
set failures [examine -radix decimal /tb_corner_store/errors]
set checks [examine -radix decimal /tb_corner_store/checks]
set scenarios [examine -radix decimal /tb_corner_store/scenarios]
if {$completed != 1 || $failures != 0 || $checks == 0} {
    echo "CORNER_STORE_FAIL completed=$completed errors=$failures checks=$checks"
    quit -code 1 -f
}
echo "CORNER_STORE_PASS scenarios=$scenarios checks=$checks errors=$failures"
quit -code 0 -f
