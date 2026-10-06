# 测试输入与参考数据

| 子目录 | 内容与来源 |
| --- | --- |
| calibration | 独立标定/几何/浮点参考向量、用例名称和基准结果 |
| real | 已保存的真实图像 C++ 导出：角点、标定参数、完成标志；保持每次导出文件成套 |
| reports | 从旧工作目录保留的精简数值对比与局部综合报告，见 [说明](reports/README.md) |
| system | 独立渲染图像、平方根向量、棋盘排序基准及系统对比基准 |
| rom | RTL 使用的权重和三角表唯一维护源；运行目录中的副本均为工具生成 |
| generators/calibration | JS/C++ 标定与数值参考、真实导出格式转换 |
| generators/image | 检测阶段 C++ 向量导出与 JPG 转换 |
| generators/remap | NumPy FP32 算术和坐标映射参考 |
| generators/system | 独立测试棋盘图像、平方根向量生成 |
| generators/rom | ROM 表生成器 |

以下命令从仓库根运行。已保存的快速回归向量可直接使用；重新生成数据不能覆盖 RTL 的输出以充当期望值。

```powershell
node data/generators/calibration/generate_model_vectors.js
node data/generators/calibration/generate_init_vectors.js
node data/generators/system/generate_board_images.js
python data/generators/system/generate_sqrt_vectors.py
python data/generators/remap/generate_vectors.py --out data/remap
```

历史检测阶段的大型二进制向量按需写入 `data/image`；`generators/image/run_export.cmd` / `run_export_gcc.cmd` 可生成基础图像参考，其他阶段生成器位于同目录。实际源图由 C++ 工程提供，尚未生成的历史向量不属于快速测试必备数据。

`real` 保存已有对拍输入，C++ 应用以后生成的新一轮导出仍写入其原 `exports` 目录；需要固定为回归输入时再成套归档到这里。仿真实际结果写入根 `build`，不混入参考目录。
