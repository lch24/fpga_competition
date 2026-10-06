transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
    set done [examine -radix unsigned /tb_reg_config_clock/finished]
    set edges [examine -radix unsigned /tb_reg_config_clock/edges]
    if {$done!=1 || $edges!=6} {quit -code 1 -f}
    echo "PASS tb_reg_config_clock edges=$edges half_period=1200 full_period=2400"
    quit -code 0 -f
}
run -all
