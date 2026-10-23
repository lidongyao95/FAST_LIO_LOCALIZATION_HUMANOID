# G1 MuJoCo 定位与导航验证记录

验证日期：2026-10-23

环境：Ubuntu 22.04、ROS 2 Humble、官方 Unitree G1 29-DoF MuJoCo 模型

验证范围：本机仿真；不包含真实 G1、真机传感器标定或 ARM64 部署。

## 1. 地图与定位

- 早期完整闭环使用 `sim/generated/mapping/map.pcd`，共 39,947 个有限点。
- 最新一次实际建图保存成功，`/map_save` 返回 `success=True`，当前 PCD 为
  296,091 点、约 9.48 MB。
- 早期闭环还导出了同坐标系下的 `map.ply`、`map.pgm`、`map.yaml` 和
  `metadata.json`。
- 重启仿真到 `(1.0, -1.0, 0.3)` 后加载 `mapping/map.ply`，初始配准 fitness
  连续两次为 `0.93456` 和 `0.93484`，通过 `threshold_fitness_init=0.5`。
- 操作员门禁启用时，定位数据新鲜，置信度约 `0.956`，控制器处于速度模式，
  支架已释放。
- Nav2 实际加载 `mapping/map.yaml`，不是场景预生成地图。

## 2. 动态障碍目标导航

命令：

```bash
./scripts/validate_nav_goal.sh \
  --x 4.0 --y 1.5 --timeout 90 \
  --obstacle-after 3.0 \
  --output sim/generated/nav_dynamic_obstacle_validation.json
```

结果：

- 目标状态：`SUCCEEDED`。
- 起点真值约 `(0.987, -1.018)`，终点真值约 `(4.182, 1.586)`。
- 运动开始 3 秒后，在 `(2.20, 0.40)` 放入静态地图中不存在的动态障碍。
- 障碍经 `/livox/lidar` 进入 Nav2 代价地图；规划路径更新 32 次。
- 机器人中心与障碍中心的最小平面距离约 `1.082 m`，未进入碰撞轮廓。
- 最终定位置信度约 `0.921`，机身高度约 `0.784 m`。

## 3. 取消与停止

命令：

```bash
./scripts/validate_nav_goal.sh \
  --x 0.0 --y 0.0 --timeout 15 --cancel-after 3.0 \
  --output sim/generated/nav_generated_cancel_validation.json
```

结果：

- Nav2 目标状态：`CANCELED`。
- 取消后最后 2 秒真值路径长度约 `0.049 m`，机器人保持在目标点附近并减速停止。
- 随后执行 `./scripts/g1_operator.sh stop`，导航门禁关闭、目标取消、零速度发布。

## 4. 构建与关闭

- 全部 `scripts/*.sh` 通过 `bash -n`。
- 全部 `scripts/*.py` 通过 Python AST 解析。
- `global_localization_node` 编译成功。
- 定位节点空输入保护已加入：目标或源点云少于 10 点时跳过 ICP，避免首帧崩溃。
- 定位线程在节点析构前 join；单独向节点发送 `Ctrl+C` 后退出码为 0。
- 验证结束后没有残留 G1、FAST-LIO、Open3D 或 Nav2 进程。

## 5. 键盘控制

- 交互式 PTY 启动时，控制器输出 `Keyboard control enabled`。
- 连续发送 `W` 后，MuJoCo 真值从原点附近移动到约 `(0.296, 0.065)`。
- 连续发送 `Q` 后，偏航四元数约从 `w=0.99995, z=0.0023` 变为
  `w=0.98395, z=0.17836`，对应约 `0.36 rad` 左转。
- 按空格后，机器人从约 `(0.296, 0.065)` 回到约 `(0.293, 0.063)` 并稳定。
- 控制器会把键盘速度限幅，并在 Nav2 导航门禁开启时禁止键盘覆盖导航速度。

## 6. 可视化与 GPU

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

## 7. 证据文件

运行产物位于 `sim/generated/`：

- `mapping/map.pcd`
- `mapping/map.ply`
- `mapping/map.pgm`
- `mapping/map.yaml`
- `mapping/metadata.json`
- `nav_dynamic_obstacle_validation.json`
- `nav_generated_cancel_validation.json`
- `ros_logs/`

这些目录被 `.gitignore` 排除，验证记录以本文源码和实际运行产物共同为准。
