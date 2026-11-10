# G1 MuJoCo 建图、定位与 Nav2 导航

本文对应 ROS 2 Humble、官方 Unitree MuJoCo G1 模型和本仓库内的 FAST-LIO/Open3D
定位实现。当前验证范围是室内平地仿真，不代表真机导航已经验收。

默认使用 `ROS_DOMAIN_ID=72` 和 Fast DDS。所有终端都应从本仓库根目录启动下方
脚本，脚本会统一设置 ROS、依赖库和工作目录。

脚本会自动加载环境，但新终端中直接执行 `ros2 ...` 前必须先加载工作区，
否则可能连接到错误的 ROS domain，看起来就像节点或服务不存在：

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
source scripts/walking_env.sh
```

## 1. 入口说明

| 入口 | 用途 |
|---|---|
| `scripts/run_g1_mujoco.sh` | 支架约束的传感器/定位最小演示，不启动行走策略 |
| `scripts/start_walking_sim.sh` | 自由站立 G1、MuJoCo 传感器和动态障碍服务 |
| `scripts/start_g1_controller.sh` | 官方 G1 控制策略、DDS 低层桥和 `/cmd_vel` 桥 |
| `scripts/start_g1_mapping.sh` | FAST-LIO 建图和 RViz；运行期间提供 `/map_save` |
| `scripts/start_g1_localization.sh` | FAST-LIO 连续里程计和 Open3D 三维地图重定位 |
| `scripts/start_g1_navigation.sh` | Nav2、静态/实时障碍代价地图和 RViz |
| `scripts/g1_operator.sh` | 状态检查、站稳、解除支架、启用导航和停止 |

启动脚本会检查前序阶段是否仍在运行。定位和导航阶段还会由
`g1_operator.sh enable-nav` 检查控制器模式、支架释放、里程计/定位数据新鲜度
以及定位置信度，不以固定等待时间作为就绪依据。

MuJoCo 窗口显示场景网格和碰撞几何，包括地面、墙体和障碍物；RViz 显示雷达
实时点云、FAST-LIO 配准点云和三维定位地图。两者用途不同。

有 X11 显示时，G1 入口默认启用 NVIDIA PRIME：

```text
MUJOCO_GL=glfw
__NV_PRIME_RENDER_OFFLOAD=1
__GLX_VENDOR_LIBRARY_NAME=nvidia
```

可用 `glxinfo -B` 确认为 RTX 显卡。需要强制软件渲染时设置
`G1_MUJOCO_GPU=0`。建图和定位入口默认打开 RViz，可分别用
`rviz:=false` 和 `show_rviz:=false` 关闭。

GPU 检查：

```bash
source scripts/walking_env.sh
glxinfo -B | grep 'OpenGL renderer'
nvidia-smi
```

MuJoCo 应显示为 RTX 显卡的图形进程。环境变量只在启动时读取，修改脚本后必须
重启仿真进程。

RViz 中的主要点云显示：

| 显示名 | 话题 | 含义 |
|---|---|---|
| `Live LiDAR` | `/livox/lidar` | 雷达原始实时点云 |
| `FAST-LIO registered cloud` | `/cloud_registered_1` | 配准到局部坐标系的点云 |
| `3D localization map` | `/localization/map_cloud` | 加载的三维定位地图 |

## 2. 终端键盘控制

在交互式终端运行 `start_g1_controller.sh` 后，控制器会启用键盘输入：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左移 / 右移 |
| `Q` / `E` | 左转 / 右转 |
| 空格 | 立即将速度置零 |
| `1` / `2` / `3` | Passive / FixStand / Velocity 状态；必须先 `2` 站稳，再按 `3` |

持续产生方向键输入即可运动，停止后约 0.25 秒自动回到零速度。键盘速度已经限幅为
`0.15 m/s` 前进、`0.12 m/s` 后退、`0.15 m/s` 横移和 `0.2 rad/s` 转向。

横移速度低于 `0.15 m/s` 时，当前行走策略可能不启动横向步态；请在速度模式并解除
支架后持续按键，`A/D` 才会产生可见横移。

Nav2 导航启用后，键盘运动会被锁定，导航速度具有独占控制权。停止导航后，
键盘控制自动恢复。如果标准输入不是 TTY，键盘功能自动关闭，但 ROS `/cmd_vel`
入口仍然可用。

## 3. 行走建图

终端 1，启动 MuJoCo：

```bash
./scripts/start_walking_sim.sh
```

终端 2，启动控制器：

```bash
./scripts/start_g1_controller.sh
```

终端 3，等待控制器就绪后站稳、切换速度模式并解除支架：

```bash
./scripts/g1_operator.sh prepare
./scripts/g1_operator.sh status
```

终端 4，开始 FAST-LIO 建图：

```bash
./scripts/start_g1_mapping.sh
```

终端 5，用键盘移动机器人：先按 `W` 直行，再按 `Q` 或 `E` 转向，完成后按空格
停止。也可以用 ROS 速度指令作为后备：

```bash
source scripts/walking_env.sh
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.12}, angular: {z: 0.0}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.15}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

每条命令用 `Ctrl+C` 结束。地图覆盖足够后保存 PCD：

```bash
source scripts/walking_env.sh
ros2 service call /map_save std_srvs/srv/Trigger "{}"
```

预期响应：

```text
success=True
message='Map saved.'
```

保存路径为 `sim/generated/mapping/map.pcd`。可以检查点数和修改时间：

```bash
grep -E 'WIDTH|POINTS|DATA' sim/generated/mapping/map.pcd
stat sim/generated/mapping/map.pcd
```

确认保存成功后，在终端 4 用 `Ctrl+C` 结束建图，然后导出 PLY 和二维栅格：

```bash
source scripts/walking_env.sh
/usr/bin/python3 scripts/export_g1_map.py sim/generated/mapping/map.pcd
```

输出为：

```text
sim/generated/mapping/map.pcd
sim/generated/mapping/map.ply
sim/generated/mapping/map.pgm
sim/generated/mapping/map.yaml
sim/generated/mapping/metadata.json
```

## 4. 重启并加载生成地图

先停止终端 4、3、2、1 中的进程。之后在终端 1 以新起点启动仿真；新起点必须
位于已建图区域，不是任意位置都能可靠重定位：

```bash
./scripts/start_walking_sim.sh --start-x 1.0 --start-y -1.0 --start-yaw 0.3
```

终端 2：

```bash
./scripts/start_g1_controller.sh
```

终端 3，加载行走生成的三维地图并给出初始位置：

```bash
./scripts/start_g1_localization.sh \
  map_ply:=$PWD/sim/generated/mapping/map.ply \
  initial_x:=1.0 initial_y:=-1.0 initial_z:=0.0 initial_yaw:=0.3
```

终端 4，确认定位收敛并释放支架：

```bash
./scripts/g1_operator.sh prepare
./scripts/g1_operator.sh status
```

终端 5，加载同一地图导出的二维栅格：

```bash
./scripts/start_g1_navigation.sh \
  map_yaml:=$PWD/sim/generated/mapping/map.yaml
```

终端 6，开启运动门禁：

```bash
./scripts/g1_operator.sh enable-nav
```

在 RViz 中使用 `Nav2 Goal` 或 `2D Goal Pose` 下发目标。应同时确认地图、机器人、
点云和规划路径重合。

## 5. 实时障碍与停止

Nav2 的全局和局部代价地图都订阅 FAST-LIO 输出的 `/cloud_registered_body_1`。仿真中预置了一个不在静态地图里的红色障碍，可通过服务启停。

放入和移除障碍：

```bash
ros2 service call /g1/obstacle std_srvs/srv/SetBool "{data: true}"
ros2 service call /g1/obstacle std_srvs/srv/SetBool "{data: false}"
```

保留原启动顺序，只给第一个 MuJoCo 脚本增加参数。房间 PLY 和二维栅格仍由原有墙体和底图障碍生成。通过预设开关启动时直接放入障碍，行走中可调用服务移除或重新放入。

```bash
./scripts/start_walking_sim.sh --moving-obstacle --fixed-obstacles 3
```

传入 `--moving-obstacle` 后，红色移动薄板（沿 X 轴厚 0.02 m，宽 0.5 m，高 1.5 m）按仿真时间在起点和终点之间往返。`--fixed-obstacles N` 取 0 至 3，按预设顺序放入前 N 个，依次对应 `/g1/fixed_obstacle_1` 到 `/g1/fixed_obstacle_3`。固定障碍是红橙色，服务同样使用 `std_srvs/srv/SetBool`。这些扩展参数用于默认房间场景。

正常停止：

```bash
./scripts/g1_operator.sh stop
```

该命令关闭导航门禁、发布零速度并取消当前 Nav2 目标。随后按导航、定位、控制、
仿真终端的逆序用 `Ctrl+C` 停止。

## 6. 自动验证

仅行走控制：

```bash
./scripts/validate_walking.sh
```

导航目标、取消和动态障碍：

```bash
./scripts/validate_nav_goal.sh \
  --x 4.0 --y 1.5 --timeout 60 \
  --obstacle-after 3.0 --output sim/generated/nav_dynamic_obstacle_validation.json
```

验证结果保存在 `sim/generated/`。`trace` 记录 MuJoCo 真值、定位里程计、置信度
和导航门禁；真值只用于评价，不参与 FAST-LIO 或 Nav2。

本次完成闭环的具体数据和环境边界见 [`VALIDATION.md`](VALIDATION.md)。

## 7. 故障检查

如果 `/map_save` 一直显示 `waiting for service to become available`：

```bash
source scripts/walking_env.sh
ros2 node list | grep g1_mapping
ros2 service list | grep map_save
```

只有建图阶段提供 `/map_save`，定位和导航阶段不提供。确认建图进程存在后再次调用。
如果 `ros2 node list` 中没有 `/g1_mapping`，通常是没有加载本仓库环境，或建图
进程已经在另一个 ROS domain。

如果 MuJoCo 没有房间，确认使用的是当前生成的 `sim/generated/scene.xml` 并重启
`scripts/start_walking_sim.sh`。如果 RViz 没有点云，检查 `Live LiDAR` 和
`FAST-LIO registered cloud` 是否勾选，并检查话题频率：

```bash
source scripts/walking_env.sh
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered_1
```

```bash
ros2 topic echo /Odometry --field pose.pose.position
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /localization_3d_confidence
ros2 run tf2_ros tf2_echo map base_footprint
./scripts/g1_operator.sh status
```

如果机器人不运动，先检查 `controller_mode=3`、`released=true`、定位置信度不低于
`0.6`、`odom_fresh=true` 和 `nav_ready=true`。定位漂移或代价地图持续报告越界时，
应停止导航并重新启动定位，而不是只重启 Nav2。

## 8. 使用真机扫描生成的仿真场景

[扫描场景与 RViz 导航指南](g1_real_robot_docs/scan_mujoco_preview.md)复用本手册第 4 节启动顺序。
先生成场景和配对地图，再用 `start_walking_sim.sh --model-path ...` 加载场景，
定位与导航分别显式传入对应 `map_ply`、`map_yaml`。地面扩展和碰撞体素属于仿真近似，
不能直接作为真机导航验收结果。

使用实测点云生成的石膏网格时，按[4322 石膏版手动导航](g1_real_robot_docs/scan_mujoco_preview.md#5-4322-石膏版手动导航)
加载配套场景、定位点云和二维图。该节给出了初始位姿、RViz 配置和交接判据。
