# 初值阶段与验证

`init_controller` 把图像尺寸、棋盘几何和已提交的 FP32 角点装入共享工作 RAM，然后启动初值入口。程序逐图求归一化 DLT/Jacobi 单应矩阵，以图宽作为焦距初值，由 H 和 K 恢复 R/t，畸变置零。完成后读回一组 FP64 状态，交给 LM。

算法源在 `scripts/calibration/engine/build_init.py`，姿态子程序在 `data/programs/calibration/pose.asm`。板级没有独立 `homography/zhang/jacobi_eigen` 实例；`pose_init` 保留作姿态子程序的独立验证包装。

`tb/calibration/tb_init_controller.sv` 连接当前共享服务，检查种子、错误输入、任务重启和响应。运行：

```powershell
python scripts/calibration/engine/check_init.py
```

快速单例使用 `--quick`。原始角点和期望输入在 `data/calibration/init_controller_vectors.txt`，本次实际输出在 `build/calibration_engine/init`。不同视图数的姿态独立测试入口为 `scripts/calibration/check_pose_views.py`。
