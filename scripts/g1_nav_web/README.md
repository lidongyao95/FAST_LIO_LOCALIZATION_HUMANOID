# PC2 网页仅规划

Mac 通过 SSH 转发打开浏览器。网页显示 `/map`、地图坐标系下的 `base_footprint`、可开关的全局代价地图叠层，以及本次 `ComputePathToPose` 返回的路径。在地图上点选目标，按住拖动设置朝向，然后点击「仅规划」。

## 连接现场 ROS 2

先在 PC2 按实机调试文档准备已验收的地图、定位和 `mode:=plan`。网页进程要与规划器使用同一套 ROS 2 环境，包括 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 和 RMW。演示用的 217 域和静态 TF 只用于合成图。

在 Mac 终端建立 SSH 会话并转发端口：

```bash
ssh -t -o ExitOnForwardFailure=yes -L 127.0.0.1:18081:127.0.0.1:18081 unitree@192.168.123.164
```

在该会话中加载现场 ROS 环境并启动网页：

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
# 在此设置与现场规划器相同的 ROS 环境变量。
python3 scripts/g1_nav_web/server.py
```

在 Mac 浏览器打开 <http://127.0.0.1:18081/>。页面上地图、TF 和规划器均就绪后，点选目标。点击「仅规划」会提交一个规划 Action。现场使用实时时钟，回放如使用仿真时钟，则给网页进程追加 `--ros-args -p use_sim_time:=true`。Ctrl+C 停止网页进程。

全局代价地图来自 `/global_costmap/costmap_raw`。红色表示障碍或不可通行的内切膨胀区，橙色表示膨胀代价。走行目标和取消走行属于后续功能。

## Foxglove 浏览器可视化

PC2 上的 `foxglove_bridge.py` 使用 Foxglove 的 Python WebSocket 库。
依赖装在 `~/ldy/nav-web-bridge/foxglove/lib`，版本为
`foxglove-websocket==0.1.2` 和 `websockets==11.0.3`。

从 Mac 用 SSH 将本地 `127.0.0.1:8765` 转发到 PC2 同端口。PC2 终端加载
Foxy 和项目 overlay，设置与地图和定位节点一致的 ROS 环境，再执行：

```bash
export PYTHONPATH="$HOME/ldy/nav-web-bridge/foxglove/lib:${PYTHONPATH:-}"
python3 scripts/g1_nav_web/foxglove_bridge.py \
  --cloud "$HOME/ldy/g1_real_data/maps/hang_A_02_plaster_preview/clean.pcd"
```

在 Foxglove 浏览器客户端选择 Foxglove WebSocket，填入
`ws://127.0.0.1:8765`。添加 3D 面板，固定坐标系设为 `map`，
启用 `/plaster/clean_cloud`、`/map`、`/global_costmap/costmap` 和 TF。
`/localization/scan_cloud` 由现场定位节点提供。点选仅规划仍通过原网页提交。
