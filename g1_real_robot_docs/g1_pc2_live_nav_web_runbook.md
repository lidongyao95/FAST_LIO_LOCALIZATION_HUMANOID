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

Mac 到 PC2 有两条路径，任选其一：

- 同一 Wi-Fi：PC2 的 `wlan0` 与 Mac 同在 `172.18.0.0/16`，可直接 `ssh unitree@172.18.17.116`，
  也可用 `ubuntu.local`（mDNS，实测可解析）。该地址是 DHCP 租约，重连后可能变化，先用
  `ping -c 2 ubuntu.local` 或请人在 PC2 上 `hostname -I` 确认。ROS 侧不受影响：
  域 223 已设 `ROS_LOCALHOST_ONLY=1`，DDS 不出 PC2 回环，跨机流量只有网页的 SSH 隧道。
- 有线直连：Mac 的 `ifconfig` 里出现 `192.168.123.x`（本轮为 `.100`），PC2 为 `192.168.123.164`。
  没接网线时 PC2 的 22 端口即使能建立连接，也会在 `kex_exchange_identification` 阶段被对端断开，
  同时 `ping` 全丢包；这种情况先查网线，不要去查 sshd。

两个地址在 `known_hosts` 中是独立记录，第一次用新地址会报 `Host key verification failed`。
登记前先 `ssh-keyscan -t ed25519 <新地址>`，确认 ed25519 公钥与已有那条（同一台机器）一致。

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
从这里到复验结束保持 FAST-LIO 进程持续运行。本轮判据：点云 10 Hz、每帧约 2 万点，IMU 200 Hz。

`ros2 topic list` 和 `ros2 topic hz` 在 Foxy 下会因 CLI 守护进程缓存或 QoS 不匹配而显示为空或漏掉
`/livox/*`，不能据此判断雷达没数据。先 `ros2 daemon stop` 刷新图再重试，核对一律以
`probe_topics.py` 的输出为准。

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
SciPy 会提示所需的 NumPy 版本区间（例如 NumPy 2.5.1），该告警不影响本次求解；
只有评分、重叠率或复验结果异常时才降级 NumPy。

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

顺序约束：`map → camera_init` 必须先于步骤 5 的 Nav2 生效。缺 `map → body` 时
`global_costmap` 无法完成激活，`scripts/ensure_g1_nav_lifecycle.py` 超时后整个 launch 以
`RuntimeError: G1 Nav2 lifecycle startup failed (exit 1)` 退出，日志末尾是向 `map_server` 和
`planner_server` 发 `SIGINT`。已经退出的 launch 不会自愈，TF 生效后必须重新执行步骤 5。

## 5. 启动仅规划和网页

在 PC2 启动只含地图和全局规划的 Nav2。先在同一环境确认
`ros2 run tf2_ros tf2_echo map body` 已能输出变换，否则按上一条会直接退出：

```bash
ros2 launch "$REPO/open3d_loc/launch/g1_navigation.launch.py" \
  "repo:=$REPO" "map_yaml:=$MAP/map.yaml" mode:=plan robot_frame:=body use_sim_time:=false rviz:=false
```

另开 Mac 终端建立网页隧道（走 Wi-Fi 时把地址换成 `172.18.17.116` 或 `ubuntu.local`）：

```bash
ssh -t -o ExitOnForwardFailure=yes -L 127.0.0.1:18085:127.0.0.1:18085 unitree@192.168.123.164
```

本地端口必须与网页服务的 `--port` 一致。换成 18086 之类时服务端 `Host` 白名单只认
`127.0.0.1:18085` 和 `localhost:18085`，会直接返回 403 `Invalid host`。

在该 SSH 会话加载开头的 PC2 环境后运行：

```bash
python3 "$REPO/scripts/g1_nav_web/server.py" --port 18085 --robot-frame body --demo '实机雷达 · 连续里程计预览'
```

Nav2 侧判据是日志出现 `All G1 Nav2 lifecycle nodes active`，网页侧判据是
`curl -s http://127.0.0.1:18085/api/state` 中 `map`、`planner` 为 `true`，`pose` 非空，
`costmap` 为非 0 的版本号。网页只监听 `127.0.0.1:18085`，隧道映射的也是 18085；
访问 `18081`（`--port` 的默认值）不会有任何服务。

重启 Nav2 前先等旧进程完全退出：launch 一开始会执行 `ensure_g1_nav_lifecycle.py --wait-for-empty-graph`，
判据是图上连续 2 秒无残留，整体 12 秒超时。残留时以
`Previous navigation session still visible: nodes=..., global costmap publishers=..., local PIDs=...` 失败。

## 6. 验收与结束

打开 <http://127.0.0.1:18085/>，开启全局代价地图，确认地图、定位 TF、规划器就绪。
同一地图下放入、再移走一个独立障碍，记录叠层的标记和清除，以及同一目标的仅规划路径变化。

「实时雷达障碍点」为空时先硬刷新页面（`Cmd + Shift + R`），页面在数据就绪前打开会沿用旧缓存，
表现为没有雷达点但其他图层正常。仍为空时用
`curl -s http://127.0.0.1:18085/api/obstacles` 区分前后端：返回数百个随场景变化的 `points`
说明后端在出点、问题在浏览器侧；返回 `null` 则回步骤 2 查 `/cloud_registered_body_1`。
Mac 可查询 `/api/state` 的 `costmap` 版本是否变化。结束时在各启动终端 Ctrl+C，
先停网页和 Nav2，再停本轮静态 TF、FAST-LIO 和雷达。仿真使用独立的 ROS 域和既有启动脚本。

正式外参、连续定位、实机走行、停止和接管的验收见[实机部署指南](g1_real_robot_slam_pc2_b300_guide.md)。
本手册截至静止对齐和仅规划。当天数据见[调试记录](debug_log/2026-11-10_pc2_live_nav_web.md)。

## 已知地图下的连续定位

已有三维地图时，真机定位、实时网页和导航联调默认使用 Open3D 持续配准。前述静止对齐用于给当前 FAST-LIO 会话提供初值。Open3D 初始化并进入稳态后，由它发布 `map → odom`，通过 `odom → camera_init → body` 给网页和仅规划提供地图位姿。切换时停掉旧的 `map → camera_init` 静态发布器。正式验收采用实测安装外参与独立参考位姿。纯 FAST-LIO 用于建图、隔离诊断和对照实验。见[本轮分析](debug_log/2026-11-10_pc2_live_nav_web.md#重启复测与-open3d-使用约定)。
