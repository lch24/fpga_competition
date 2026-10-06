# 测试与工程脚本

从仓库根目录运行下面的命令。脚本根据自身位置找到仓库，工作目录固定在根 `build` 下，也可用脚本的完整路径从其他目录启动。仓库可以移动，路径可以包含空格。

| 子目录              | 用途                                             |
| ------------------- | ------------------------------------------------ |
| calibration         | 初始化、LM、校验、标定顶层、多配置及 C++ 对拍    |
| compute             | 浮点/矩阵/几何测试、共享计算与定点回归           |
| memory              | 角点存储独立测试与缓存 Tcl 检查                  |
| image、remap、video | 相应模块的 ModelSim Tcl 完成检查                 |
| system              | 联合编译与仿真入口、系统测试清单、数值结果检查   |
| build               | PDS 配置、隔离综合、目录检查、RTL 连接生成、清理 |
| common              | 外部工具定位和模拟器退出处理                     |

需安装 ModelSim、Node.js；Python 数值参考用到 NumPy，安装命令为 `python -m pip install -r data/generators/requirements.txt`。C++ 参考生成需要 MSVC 或相应生成器支持的 g++。

ModelSim 从 `MODELSIM_BIN` 环境变量或 PATH 查找，也可传 `-ModelSimBin`。PDS 使用 `PDS_SHELL` 或 PATH，也可传 `-PdsBin` / `--pds`。MSVC 使用 `VS_ROOT` 或 `-VsRoot`。这些值指向各人自己的工具安装位置，仓库不保存开发者机器上的绝对路径。

```powershell
# 不启动仿真的目录/路径检查
python scripts/build/check_test_layout.py
python scripts/build/check_rtl_layout.py

# 板级连接与控制；IP 端口桩只验证连接
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -Board -OnlyTest tb_board_flow
# DDR 集成与校正单项检查
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -OnlyTest tb_vision_ddr
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/system/run_checks.ps1 -OnlyTest tb_undistort
# 局部数值/协议回归
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/compute/run_project_point.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/memory/run_corner_store.ps1
python scripts/calibration/check_shared_init.py
python scripts/calibration/check_shared_residual.py
node scripts/calibration/test_calib_real_data.js
```

`run_checks.ps1` 维护常用联合测试集，`-OnlyTest` 选择其中一个；不指定时运行默认集合，完整数值 LM 仍需显式选择。某些历史检测 TB 使用按需生成的 `data/image` 向量，并不属于默认快速集合。独立数值脚本使用独立工作库；不要同时运行两个写入同一 `build/system` 的联合脚本。

生成输入/参考数据的程序放在 [data/generators](../data/README.md)。日志、波形、临时编译清单和生成的测试包装写入 `build`，不提交 Git。PDS 所需 ROM 镜像由 `configure_pds.ps1` 从 `data/rom` 复制，clone 后首次使用 PDS 应运行该脚本；只配置并保存/重开工程，不启动综合。
