# 角点与相机参数导出

桌面程序在检测和标定后、显示窗口前，将本次结果写入 `main.cpp` 的 `export_root` 下新建的 `run_*` 目录，并在控制台打印路径。

| 文件 | 内容 |
|---|---|
| `corners.csv` | view、点号、十进制坐标及 FP32 原始位模式 |
| `calibration.json` | 图像/棋盘尺寸、各图状态、相机参数、R/t 和误差 |
| `COMPLETE.txt` | 成套导出完成标志 |

角点列为 `view_id,point_index,x,y,x_fp32_hex,y_fp32_hex`，点号按行优先。十六进制字段保留实际输入的 FP32 值，RTL 对照从这里读取，不重新做十进制舍入。

JSON 格式为 `closer2fpga.calibration.v1`，当前算法标识 `single_seed_schur_hybrid_lm_v2`。相机参数顺序为 `fx,fy,cx,cy,k1,k2,k3,p1,p2`，另有相应位模式。各图 R 按行优先保存，t 以棋盘首角点为原点并乘格长；默认格长 1 时平移单位为棋盘格。

`camera_valid` 表示 C++ 结果可用于校正，`rtl_input_ready` 是导出器的输入格式检查，`rtl_algorithm_matches=false` 表示 C++ 与 RTL 求解组织尚不相同。`max_iterations_per_stage` 是保留的字段名，当前 C++ 只有一阶段、上限 60。检测失败时仍导出状态，未执行标定则 `result=null`。

对拍使用同一目录中的角点和参数：角点送入 RTL，参数只作为结果参考，不注入初值。固定回归输入归档到根 `data/real`，实际仿真输出写入 `build`。运行入口为 `scripts/calibration/test_calib_real_data.js`，格式读取器为 `data/generators/calibration/calib_real_data.js`。
