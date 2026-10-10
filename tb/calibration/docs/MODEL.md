# 投影与残差模型

模型把棋盘平面点变成预测像素，再与检测坐标相减，为 LM 和结果检查提供残差。

```text
棋盘点 → R/t 变换 → 透视除法 → Brown 畸变 → 内参投影 → 预测像素
                                                          ↓ 与观测相减
                                                        残差/代价
```

`rtl/compute/geometry` 中，旋转模块处理姿态表示转换；`geometry_engine` 按固定运算表和工作 RAM 完成投影；`project_point`、`brown_distort` 提供投影和单独畸变的接口包装；`residual_engine` 遍历视图与角点，产生残差和代价。

板级 LM/检查共用残差服务，浮点请求进入共享池。模型的数值步骤仍由专用序列执行，标定程序用 HOST 调用整批残差，避免把所有像素/角点接口搬进通用指令核心。

运行入口位于 `scripts/compute`：`run_rotation.ps1`、`run_project_point.ps1`、`run_brown_distort.ps1`、`run_residual_engine.ps1`。参考数据在 `data/calibration`，仿真输出在 `build` 对应子目录。
