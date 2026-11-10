# G1 PC2 实机逐步运行 · 静止自动对齐与仅规划

适用 PC2/Foxy，机器人在录制区域内静止。每个 PC2 终端先加载下列环境。域 223 是本次实机诊断域，启动前确认没有旧回放、定位或 TF 发布器占用它。控制和速度桥保持隔离。

```bash
export REPO="$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export MAP="$HOME/ldy/g1_real_data/maps/hang_A_02_plaster_preview"
source /opt/ros/foxy/setup.bash
source "$REPO/install/local_setup.bash"
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
```

## 1. 核对输入

PC2 上应有配套的 `$MAP/{clean.pcd,map.yaml,map.pgm}`，来源为 4322 的
`~/ldy/hang-a02-filter-comparison/plaster-map/`。核对 SHA-256，保持同一组地图。
运行 `ping -c 2 192.168.123.120` 和 `chronyc tracking`。现场保持静止至步骤 6 结束。

## 2. 启动雷达和 FAST-LIO

分别在两个 PC2 终端执行：

```bash
ros2 launch "$REPO/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
```

```bash
logs=$(mktemp -d /tmp/g1-static-lio.XXXXXX)
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$REPO/FAST_LIO/config/mid360.yaml" \
  -p use_sim_time:=false -p common.lid_topic:=/livox/lidar \
  -p common.input_roll_180:=true -p pcd_save.pcd_save_en:=false \
  -p "debug_log_dir:=$logs"
```

第三个 PC2 终端可用 `python3 "$REPO/scripts/probe_topics.py" lidar --seconds 8`
核对点云，对 IMU 把 `lidar` 换为 `imu`。FAST-LIO 日志应进入 IMU 初始化完成和地图树初始化。
从这里到复验结束保持 FAST-LIO 进程持续运行。

## 3. 在 Mac 采集和求解

Mac Python 环境需兼容版本的 NumPy/SciPy。下面的 `local_repo` 指向 Mac 上的仓库。

```bash
local_repo=/Users/lidongyao/Desktop/code/SLAM/FAST_LIO_LOCALIZATION_HUMANOID
tools="$local_repo/scripts/g1_nav_web/static_alignment"
work=$(mktemp -d)
cd "$work"
scp unitree@192.168.123.164:ldy/g1_real_data/maps/hang_A_02_plaster_preview/clean.pcd .
remote_env='source /opt/ros/foxy/setup.bash; source ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID/install/local_setup.bash; export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1'
ssh unitree@192.168.123.164 "$remote_env; python3 - camera_init" <"$tools/capture.py" >raw.npy
python3 "$tools/level.py" >floor.txt
python3 "$tools/match.py"
cat alignment.json
```

脚本要求候选评分差至少 0.15、20 cm 内重叠率至少 85%、内点 RMS 至多 0.10 m，
并检查三个子批次的位置和航向。失败时暂停，不发布 TF。
这里的目标地面法向来自石膏版元数据，换图需要更新这项输入。

## 4. 发布静止预览 TF

在独立 PC2 终端运行，替换尖括号为本次 JSON 输出，角度均为弧度。
只发布地图到局部里程计的初始对齐。FAST-LIO 持续发布 `camera_init → body`，
网页和仅规划使用合成后的 `map → body`。

```bash
ros2 run tf2_ros static_transform_publisher <x> <y> <z> <yaw> <pitch> <roll> map camera_init
```

在步骤 3 的 Mac 终端用新帧复验：

```bash
ssh unitree@192.168.123.164 "$remote_env; python3 - map" <"$tools/capture.py" >verified.npy
python3 "$tools/verify.py"
```

复验失败则停止本轮静态 TF 发布器。同一 FAST-LIO 会话内，网页随里程计更新，无需刷新。
FAST-LIO 重启或跟踪丢失后，重新采集和求解。此预览未接入持续地图配准，累计误差需要另行验证。

## 5. 启动仅规划和网页

在 PC2 启动只含地图和全局规划的 Nav2：

```bash
ros2 launch "$REPO/open3d_loc/launch/g1_navigation.launch.py" \
  "repo:=$REPO" "map_yaml:=$MAP/map.yaml" mode:=plan robot_frame:=body use_sim_time:=false rviz:=false
```

另开 Mac 终端建立网页隧道：

```bash
ssh -t -o ExitOnForwardFailure=yes -L 127.0.0.1:18085:127.0.0.1:18085 unitree@192.168.123.164
```

在该 SSH 会话加载开头的 PC2 环境后运行：

```bash
python3 "$REPO/scripts/g1_nav_web/server.py" --port 18085 --robot-frame body --demo '实机雷达 · 连续里程计预览'
```

## 6. 验收与结束

打开 <http://127.0.0.1:18085/>，开启全局代价地图，确认地图、定位 TF、规划器就绪。
同一地图下放入、再移走一个独立障碍，记录叠层的标记和清除，以及同一目标的仅规划路径变化。
Mac 可查询 `/api/state` 的 `costmap` 版本是否变化。结束时在各启动终端 Ctrl+C，
先停网页和 Nav2，再停本轮静态 TF、FAST-LIO 和雷达。仿真使用独立的 ROS 域和既有启动脚本。

正式外参、连续定位、实机走行、停止和接管的验收见[实机部署指南](g1_real_robot_slam_pc2_b300_guide.md)。
本手册截至静止对齐和仅规划。当天数据见[调试记录](debug_log/2026-11-10_pc2_live_nav_web.md)。
