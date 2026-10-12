# Compile/elaborate ONLY. Intentionally no synthesize/implement/program command.
set root [file normalize [file join [file dirname [info script]] ../..]]
cd [file join $root OV5640_DualView_100H]
open_project [file join $root OV5640_DualView_100H DualView_OV5640.pds]
if {[catch {compile -force_to_run} detail]} {
 puts "REALTIME_COMPILE_FAILED: $detail"
 exit 1
}
puts "REALTIME_COMPILE_PASS"
exit
