transcript on
onfinish stop
onerror {quit -code 1 -f}
onbreak {
    set done [examine -radix unsigned /tb_gray_dma/finished]
    set violations [examine -radix unsigned /tb_gray_dma/violations]
    if {$done!=1 || $violations!=0} {quit -code 1 -f}
    echo "PASS gray DMA numeric/error/backpressure checks"
    quit -code 0 -f
}
run -all
