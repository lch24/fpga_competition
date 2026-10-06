# Isolated operator only; never opens or changes the board project.
set root [file normalize [file join [file dirname [info script]] ../..]]
set build [file join $root build system sqrt_pds_[clock seconds]]
create_project [file join $build sqrt.pds] -synthesize_tool 2 -family Logos2 -device PG2L100H -package FBG676 -speedgrade -6 -in_process
add_design [file join $root rtl compute float stream fp64_sqrt.v]
compile -top_module fp64_sqrt -fsm_compiler FALSE
synthesize -ads
exit
