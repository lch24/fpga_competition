param([string]$VsRoot=$env:VS_ROOT)
throw 'Retired: this runner required the removed multi-seed/staged C++ reference. RTL still uses that algorithm. Existing RTL fixtures are preserved. For current C++ use algorithom/closer2fpga/tests/run_recommended_tests.cmd; migrate RTL before regenerating its reference data.'
