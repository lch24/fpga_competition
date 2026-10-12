onfinish stop
onerror {quit -code 1 -f}
onbreak {
 set done [examine -radix unsigned /tb_fp32_basic_pipeline/done]
 set count [examine -radix unsigned /tb_fp32_basic_pipeline/checks]
 set errors [examine -radix unsigned /tb_fp32_basic_pipeline/errors]
 set latency [examine -radix unsigned /tb_fp32_basic_pipeline/max_cycles]
 set total [examine -radix unsigned /tb_fp32_basic_pipeline/total_cycles]
 if {$done!=1 || $count!=18050 || $errors!=0} {
  echo "FAIL FP32 checks=$count errors=$errors done=$done"
  quit -code 1 -f
 }
 echo "PASS FP32 checks=$count errors=$errors max_cycles=$latency total_cycles=$total"
 quit -code 0 -f
}
run -all
