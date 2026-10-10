# 结果检查阶段与验证

`validate_result` 是检查阶段适配器：装入最终状态，启动共享检查程序，处理残差服务请求，再将工作 RAM 中的参数、误差和姿态诊断发布出去。

程序源是 `scripts/calibration/engine/build_validate.py`，包括有限性与参数范围、重投影误差统计、各图姿态恢复和采样映射检查。成功时输出九个 FP32 相机参数，R/t 作为诊断信息；失败时返回状态而不发布有效相机参数。

```powershell
python scripts/calibration/engine/check_validate.py
```

测试使用 `tb/calibration/tb_validate_result.sv`，覆盖数值与协议场景。快速单例使用 `--quick`。输入在 `data/calibration/validate_result_vectors.txt`，实际输出在 `build/calibration_engine/validate`。
