# Run on an elaborated calibrated_view_top (vendor port stubs are sufficient).
# These are actual downstream parameters/ports, so an old hardcoded DMA stride
# is caught even when the top-level WIDTH/HEIGHT themselves are correct.
onerror {quit -code 1 -f}
run 0 ns
foreach {signal expected} {
    unpack/WIDTH 1280 unpack/HEIGHT 720
    unpack/CROP_X 160 unpack/CROP_Y 90
    unpack/CROP_WIDTH 960 unpack/CROP_HEIGHT 540
    algorithm/WIDTH 960 algorithm/HEIGHT 540
    algorithm/frame_stride 1920 algorithm/frame_capacity 1036800
    algorithm/calibration/cmd_width 960 algorithm/calibration/cmd_height 540
    display/WIDTH 960 display/HEIGHT 540
    display_scale/WIDTH 960 display_scale/HEIGHT 540
} {
    set value [examine -radix unsigned /calibrated_view_top/$signal]
    if {$value ne $expected} {error "$signal=$value expected $expected"}
}
set realtime [examine -radix unsigned /calibrated_view_top/REALTIME_VIDEO]
if {$realtime} {
    foreach {signal expected} {
        g_video_capture/capture/WIDTH 960 g_video_capture/capture/HEIGHT 540
        g_realtime/remap/WIDTH 960 g_realtime/remap/HEIGHT 540
        algorithm/g_packed/correction/cmd_width 960 algorithm/g_packed/correction/cmd_height 540
    } {
        set value [examine -radix unsigned /calibrated_view_top/$signal]
        if {$value ne $expected} {error "$signal=$value expected $expected"}
    }
}
set signature [examine -radix hex {/calibrated_view_top/geometry_diagnostic[31:0]}]
if {![string equal -nocase $signature 43525031]} {error "geometry signature $signature"}
set enabled [examine -radix unsigned /calibrated_view_top/ENABLE_DISTORT_DEMO]
echo "PASS board geometry crop/stride/calibration/map/display REALTIME_VIDEO=$realtime ENABLE_DISTORT_DEMO=$enabled"
quit -code 0 -f
