# Run from OV5640_DualView_100H with PDS shell -file ../scripts/build/check_pds.tcl.
set root [file normalize [file join [file dirname [info script]] ../..]]
open_project [file join $root OV5640_DualView_100H DualView_OV5640.pds]
# Preserve explicit RTL state encoding; automatic FSM optimization stalled on
# this large arithmetic/control design after successful elaboration.
compile -force_to_run -fsm_compiler FALSE
exit
