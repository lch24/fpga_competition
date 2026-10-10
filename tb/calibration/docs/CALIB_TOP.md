# 标定顶层任务控制

`calib_top` 将角点存储、初值、LM、检查、残差和共享浮点服务连接起来。当前任务流程是：

```text
收集各视图角点与检测状态
→ 初值程序输出一个 width-based seed（ID=2）
→ 单阶段 LM（stage=2，k3 固定）
→ 检查程序
→ 相机参数、诊断和完成响应
```

`stage=2` 和 seed ID 是保留的接口编号，不表示还运行前两个 LM 阶段或枚举多个初值。三个阶段共享 `calib_execution_service`；LM 和检查还复用残差服务。FP64 运算池同时提供检测精修的外部客户端。

`tb/calibration/tb_calib_top.sv` 和 `scripts/calibration/run_calib_top.ps1` 是任务验证入口；`scripts/calibration/test_calib_real_data.js` 使用归档实拍输入运行数值对照。控制桩测试只验证调度，阶段数值验证见本目录 README。

最终完成响应与参数包独立握手。下游在成功状态和完整参数包都到齐后启动建表。失败或复位会结束本次计算，不沿用旧任务的有效结果。
