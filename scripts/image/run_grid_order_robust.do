transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {resume}
run -all
echo "Cases: [examine -radix unsigned /tb_grid_order_robust/cases], errors: [examine -radix unsigned /tb_grid_order_robust/errors]"
echo "Case errors: [examine -radix unsigned /tb_grid_order_robust/case_errors]"
echo "Case cycles: [examine -radix unsigned /tb_grid_order_robust/case_cycles]"
if {[examine /tb_grid_order_robust/finished]!=1 || [examine -radix unsigned /tb_grid_order_robust/errors]!=0 || [examine -radix unsigned /tb_grid_order_robust/cases]!=8} {
    echo "FAIL grid order robust"
    quit -code 1 -f
}
echo "PASS grid order robust: 8 cases, exact point membership and degenerate rejection"
quit -code 0 -f
