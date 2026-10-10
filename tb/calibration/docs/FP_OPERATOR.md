# 浮点后端与验证

`fp_operator` 提供请求/响应接口。当前 FP64 路径进入 `fp_math_program`：基础算术调用 `calib_alu` 等数据通路，复杂函数按 `scripts/compute/microcode/build_math.py` 生成的程序执行，内部工作 RAM 保存高精度定点中间值。

标定指令核心、残差服务和检测精修通过 `fp_calibration_pool` 仲裁使用后端。请求握手后保持响应归属，结果背压时保持有效。指令宽度 32 位与数值精度无关，标定主数据仍是 FP64，复杂数学内部还使用 Q128。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/compute/run_fp_operator.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/compute/run_calib_alu.ps1
python scripts/compute/check_pair_add_pool.py
```

前两个入口检查浮点后端及基础算术，最后一个检查检测共享 FP32 加减池的数值、四客户端并发、背压、CE 和复位。检测流式算术与去畸变算术的独立测试放在 `tb/compute/stream` 和 `tb/compute/remap`。
