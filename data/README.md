# 程序源、测试输入与参考数据

## 指令程序

| 目录 | 内容 | 维护方式 |
|---|---|---|
| `programs/calibration` | 指令定义 `isa.json`，姿态、旋转、求解等 `.asm` | 输入源，修改后运行 `build_rom.py`；部分汇编供独立指令测试 |
| `programs/detection` | 检测任务可读清单 `flow.lst` | 由 `scripts/image/build_detection_program.py` 生成 |
| `programs/features` | 精修程序清单 `refine.lst` | 由 `scripts/compute/microcode/build_feature.py` 生成 |
| `programs/math` | 数学程序清单 `program.lst` | 由 `scripts/compute/microcode/build_math.py` 生成 |

实际综合使用 `rtl/include/*program_init.vh`，这些生成物随仓库提供。初值/LM/检查的展开汇编写入 `build/calibration_engine` 供调试，不是另一份维护源。程序 ROM 随 FPGA 配置初始化，工作数据运行时再装入 RAM。

## 数值输入

| 目录 | 内容 |
|---|---|
| `calibration` | 浮点、几何、初值、LM、检查等参考向量 |
| `real` | 已归档的真实图像角点和对应 C++ 参数 |
| `fixtures` | C++ 推荐算法的固定回归基准 |
| `system` | 独立渲染图像、排序和系统参考 |
| `rom` | 图像算法的权重/三角常量表 |
| `reports` | 有日期和版本边界的历史验证证据 |
| `generators` | 按 calibration/image/remap/system/rom 分类的数据生成程序 |

真实角点以 FP32 位模式输入 RTL，期望结果来自独立参考模型或保存的 C++ 导出；仿真实际输出在 `build`。独立算术向量生成器仍可写入 `data/image`；依赖已删除 Shi–Tomasi 软件接口的旧图像阶段导出器和孤立 TB 已清理。当前可运行入口见[脚本目录](../scripts/README.md)，历史结果见 [reports](reports/README.md)。
