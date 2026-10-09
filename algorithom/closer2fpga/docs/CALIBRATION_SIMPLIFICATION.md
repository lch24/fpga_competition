# 当前标定实现

C++ 已固定为经过验证的推荐方案，没有算法模式选项。图像尺寸、棋盘行列数、视图数和方格尺寸仍由输入决定，至少需要三张有效视图。

## 流程和接口

1. 归一化角点，固定 H 最后一项为 1，用 8 元线性最小二乘求各视图单应矩阵。
2. 焦距初值等于图宽，主点为图像中心；由 H 和 K 恢复各图 R、t，畸变初值为零。
3. 单阶段 LM，联合优化内参、k1/k2/p1/p2 和全部 R、t，固定 k3=0。
4. 保留收敛、内参范围、重投影误差、姿态及 33×25 点映射折叠检查；汇总复用最终残差。

LM 使用单边差分，最多 60 次外迭代、每次 8 次阻尼试探。代价改善停止阈值为 1e-7×(1+E)，相对步长阈值为 1e-9，也保留梯度停止条件。只接受降低代价的更新。

删除了 Zhang、特征值初始化、多初值、分阶段、中心差分及模型切换，以及配置类和无调用的特征值求解器。状态仍保留 k3 的零值位置，不进入优化方程。

~~~cpp
auto result = calibrate_camera(points, width, height, rows, cols, square_size);
if (result.camera.valid) {
    // 才能应用校正。
}
~~~

[入口](../closer2fpga/algo/calibrate.cpp) → [单应求解](../closer2fpga/algo/calibration/initialization.cpp) → [姿态](../closer2fpga/algo/calibration/pose_init.cpp) → [LM](../closer2fpga/algo/calibration/lm.cpp) → [检查](../closer2fpga/algo/calibration/report.cpp)。

## 验证

从仓库根目录运行，MSVC 根目录可通过 VS_ROOT 指定：

~~~powershell
& .\algorithom\closer2fpga\tests\run_recommended_tests.cmd
& .\algorithom\closer2fpga\tests\run_calibration_tests.cmd
& .\algorithom\closer2fpga\tests\run_export_tests.cmd
~~~

第一项对比上次推荐方案保存的 [8 组数值基准](../../../data/fixtures/calibration/recommended.csv)，覆盖真实角点、不同棋盘/视图数、纯径向、切向畸变、噪声、长短焦距和弱姿态，以及重复视图/缺点/退化输入拒绝。测试不再携带其他算法实现。其余两项验证 OpenCV 对照、纯 C++ 去畸变、缓冲区和导出。

真实角点 RMS 约 0.542585388 像素、残差计算 299 遍，与精简前推荐配置一致。原方案曾需要 9145 遍；推荐方案与原方案的采样映射最大变化约 0.0012 像素。删掉切向项曾导致约 17 像素变化，因此保留 p1/p2。

低 RMS 不保证边缘精度：弱姿态和较短焦距合成数据边缘真值误差仍约 8～10 像素，原方案也存在此问题。当前求解为 double、角点和输出参数为 FP32；尚未验证 FP32 求解或迁移 RTL，计算遍数下降不等于 FPGA LUT 或时间同比下降。

导出算法标识为 single_seed_forward_lm_v1，rtl_algorithm_matches=false，保留旧字段 max_iterations_per_stage=60（当前只有一阶段）。rtl_input_ready 仅表示角点格式可供旧适配器使用，不表示算法逐位一致。旧 RTL 设计规划属于旧硬件流程，不能作为当前 C++ 流程说明。

旧 RTL 的 init/LM/top/check 四个 C++ 参考生成入口已明确停用，防止新算法覆盖旧硬件期望值；已有向量与 RTL 仿真入口保留。待 RTL 迁移后再恢复相应对照。
