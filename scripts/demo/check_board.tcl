onerror {quit -code 1 -f}
set enabled [examine -radix unsigned /calibrated_view_top/ENABLE_DISTORT_DEMO]
set found [find instances -recursive /calibrated_view_top/*]
set text [join $found "\n"]
if {$enabled} {
 if {![string match {*g_demo/flow*} $text]} {error "missing optional wrapper"}
 if {[string match {*g_normal/flow*} $text]} {error "normal flow duplicated"}
} else {
 if {[string match {*g_demo/flow*} $text]} {error "demo exists in normal build"}
 if {![string match {*g_normal/flow*} $text]} {error "missing normal flow"}
}
echo "PASS board elaboration ENABLE_DISTORT_DEMO=$enabled"
quit -code 0 -f
