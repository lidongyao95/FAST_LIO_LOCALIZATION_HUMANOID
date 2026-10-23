# G1 MuJoCo 定位与导航验证记录

验证日期：2026-10-23

环境：Ubuntu 22.04、ROS 2 Humble、官方 Unitree G1 29-DoF MuJoCo 模型

验证范围：本机仿真；不包含真实 G1、真机传感器标定或 ARM64 部署。

## 1. 地图与定位

- 在停止原有域 72 进程后重新执行了一遍完整链路。
- `/map_save` 返回 `success=True`，生成的 `mapping/map.pcd` 共 31,714 个有限点，
  约 992 KiB；同时导出了 `map.ply`、`map.pgm`、`map.yaml` 和 `metadata.json`。
- 重启仿真到 `(1.0, -1.0, 0.3)` 并加载本次生成的 `map.ply`，初始配准
  fitness 为 `0.87902`，通过 `threshold_fitness_init=0.5`。
- 操作员门禁启用时，控制器处于速度模式、支架已释放、`/odom` 新鲜，定位置信度
  约 `0.94–0.97`。
- 15 秒稳态采集中，IMU 约 `500 Hz`、LiDAR 与定位均约 `9.8–9.9 Hz`；地图水平
  位置 RMSE 为 `0.125 m`，最大水平误差为 `0.129 m`。

## 2. Nav2 目标导航

命令：

```bash
./scripts/validate_nav_goal.sh \
  --x 3.0 --y -1.0 --timeout 90 \
  --output sim/generated/acceptance/nav_goal_final.json
```

结果：

- 目标状态：`SUCCEEDED`。
- 共收到 193 条 `/cmd_vel_nav`，其中 192 条非零；规划路径更新 10 次。
- 最终真值约 `(2.830, -0.816)`，`map -> base_footprint` 约
  `(2.816, -0.929)`，水平误差 `0.114 m`。
- 导航过程最大全局水平误差 `0.141 m`，最终定位置信度约 `0.893`。

### 动态障碍与取消

合并 lifecycle manager 前，同一无干扰域中还完成了动态障碍和取消测试：

- 动态障碍目标成功，规划更新 62 次，最小障碍中心距离约 `1.986 m`，最终全局
  水平误差 `0.025 m`。
- 取消测试返回 `CANCELED`；取消后最后 2 秒真值路径长约 `0.051 m`。

## 3. 构建、启动与关闭

- `setup_g1_local_deps.sh`、`build_g1_controller.py` 和 `build_g1_ros2.sh` 按文档
  顺序执行成功；三个 ROS 包全部构建完成。
- 另外在独立临时 clone 中完成从零验证；测试工作区的最终文件对应 tree
  `8bb1424f2db61cf4e6b5249578adafd394142ff3`。五个 submodule 从官方仓库重新
  拉取，`.deps` 从空目录准备，Python 依赖和 Open3D 软件包重新下载，控制器及三个
  ROS 包在空 `build/install` 中成功构建。
- 全新 clone 在域 74 完成建图、重定位和目标导航；目标成功，最终 `map ->
  base_footprint` 水平误差 `0.138 m`，最大误差 `0.159 m`。
- 首次 `g1_operator.sh prepare` 已通过持续重发模式请求完成 `FixStand`、`Velocity`
  和支架释放，不再依赖命中的单次消息。
- Nav2 改由项目 lifecycle helper 顺序激活后：越界警告为 0，激活超时为 0。
- 停止导航时五个 lifecycle 节点正常退出，日志中 `SIGKILL` 和 `ERROR` 计数均为 0。
- 验收结束后域 72 的仿真、控制器、定位和导航锁均已释放，没有残留进程。
- 定位节点空输入保护已加入：目标或源点云少于 10 点时跳过 ICP，避免首帧崩溃。
- 定位线程在节点析构前 join；单独向节点发送 `Ctrl+C` 后退出码为 0。

## 4. 键盘控制

- 交互式 PTY 启动时，控制器输出 `Keyboard control enabled`。
- 本轮键盘 `W` 输入使真值从约 `(0.375, 0.245)` 移动到 `(0.419, 0.250)`。
- 空格和 `g1_operator.sh stop` 后命令文件为零速度，导航门禁关闭。
- 控制器会把键盘速度限幅，并在 Nav2 导航门禁开启时禁止键盘覆盖导航速度。

## 5. 可视化与 GPU

- MuJoCo 房间几何改为默认显示的 geom group 0，并同步雷达组掩码；实际窗口中
  可见地面、四侧墙体和障碍物。
- 行走仿真进程在 `nvidia-smi` 中显示为 RTX 4090 的图形进程，验证时占用约
  `157 MiB`。
- 启用 PRIME 的终端中，`glxinfo -B` 报告
  `NVIDIA GeForce RTX 4090/PCIe/SSE2`，未使用 `llvmpipe`。
- `/livox/lidar` 与 `/cloud_registered_1` 均实测约 `10 Hz`。
- 建图和定位入口默认启动 RViz；RViz 中可见实时雷达点云、FAST-LIO 配准点云和
  三维定位地图，`Global Status` 为 `Ok`。
- 控制器命令与状态文件已按 ROS domain 隔离，避免并行 domain 互相覆盖状态。

## 6. 证据文件

运行产物位于 `sim/generated/`：

- `mapping/map.pcd`
- `mapping/map.ply`
- `mapping/map.pgm`
- `mapping/map.yaml`
- `mapping/metadata.json`
- `acceptance/nav_goal_final.json`
- `acceptance/nav_dynamic_obstacle.json`
- `acceptance/nav_cancel.json`
- `acceptance/validation-final.json`
- `acceptance/mapping.log`
- `acceptance/localization-final.log`
- `acceptance/navigation-final.log`
- `ros_logs/`

这些目录被 `.gitignore` 排除，验证记录以本文源码和实际运行产物共同为准。
