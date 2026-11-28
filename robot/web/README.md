# PC2 网页：规划与实机导航

Mac 通过 SSH 转发打开浏览器。网页显示 `/map`、地图坐标系下的机器人位姿（实机使用 `body`）、可开关的全局代价地图叠层，以及本次 `ComputePathToPose` 返回的路径。在地图上点选目标，按住拖动设置朝向，然后点击「仅规划」；实机模式下再点击「开始实机导航」，由导航 SDK 驱动行走，随时可「取消并停车」。

日常操作优先使用[实机部署路线](../../../docs/WORKFLOWS.md#实机部署路线)中的
`05_web_navigation.sh`，它会检查持续定位、启动 Nav2 和 Web，并传入 `--robot-frame body`。
以下步骤用于需要单独启动网页的调试场景。05 默认以实机导航模式启动；`--plan-only` 只启动 `map_server` 与 `planner_server`，网页仅提供路径，不启动控制器、局部代价地图、行为树、速度桥和 SDK。

## 连接现场 ROS 2

先在 PC2 按实机调试文档准备已验收的地图、定位和 Nav2（`mode:=plan` 或 `mode:=real`）。网页进程要与 Nav2 使用同一套 ROS 2 环境，包括 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 和 RMW。演示用的 217 域和静态 TF 只用于合成图。实机导航还需要已构建的 `$G1_DATA/bin/g1_navigation_sdk`（`./scripts/real_robot/build_navigation_sdk.sh`），并要求机器人由现场操作员进入 FSM 501。

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
python3 scripts/real_robot/web/server.py --robot-frame body
# 实机导航调试需追加：--real-navigation --sdk-executable "$G1_DATA/bin/g1_navigation_sdk" --robot-interface eth0
```

在 Mac 浏览器打开 <http://127.0.0.1:18081/>。页面上地图、TF 和规划器均就绪后，点选目标。点击「仅规划」会提交一个规划 Action；实机模式下点击「开始实机导航」会先规划预览、再提交 `NavigateToPose` 目标并启动 SDK，见[实机导航](#实机导航)。现场使用实时时钟，回放如使用仿真时钟，则给网页进程追加 `--ros-args -p use_sim_time:=true`。Ctrl+C 停止网页进程。

全局代价地图来自 `/global_costmap/costmap_raw`。红色表示障碍或不可通行的内切膨胀区，橙色表示膨胀代价。实机导航的目标提交、取消与停车见[实机导航](#实机导航)。

## 实机导航

`05_web_navigation.sh` 默认（不带 `--plan-only`）启用实机导航。网页只在定位健康、Nav2 导航服务就绪且 SDK 已构建时才允许开始；开始前机器人必须由现场操作员进入 **FSM 501（Regular mode）**，否则 SDK 返回 `REJECTED FSM_NOT_501`，页面显示启动失败原因，开关命令见 [WORKFLOWS B5.0](../../../docs/WORKFLOWS.md#b50-实机导航前置让机器人进入-fsm-501)。

- 点击「开始实机导航」：网页先按当前目标调用 `ComputePathToPose` 预览路径，成功后再提交 `NavigateToPose` Action，并启动 `g1_navigation_sdk` 子进程（自动移除 `LD_LIBRARY_PATH`，SDK 自带 DDS）。SDK 完成 `READY`/`ARM` 握手后，网页把控制器输出的 `/g1/navigation/cmd_vel` 以 20 Hz 转发为 `V x y z`。
- 开始条件：`/localization_3d_registration_ok` 为真且 5 秒内有更新、`map → body` TF 年龄小于 1 秒、`/g1/navigation/cmd_vel` 恰好一个发布者、Nav2 在 4 秒内接受目标。
- 停止条件：点击「取消并停车」、到达目标或导航结束、页面心跳超过 2 秒未更新、定位或 TF 失效、控制器速度超过 0.35 秒未更新、开始后 2 秒仍无速度输出。停止时网页取消 Nav2 目标并向 SDK 发 `STOP`；零速度确认失败会提示人工接管。
- 同一时刻只允许一个任务，由发起页面持有 token 并约每 2 秒续期心跳；刷新或关闭页面会因心跳丢失而停车。实机导航进行中不能修改“规划考虑实时障碍”和三个滑块，需先取消任务。
- 现场必须有人值守并保证急停可用；网页只负责按控制器输出转发速度，不替代运控与现场安全判断。

`GET /api/state` 的 `navigation` 字段给出 `mode`、`available`、`reason`、`phase`、`active` 和 `message`；`POST /api/navigate` 提交 `{"x","y","yaw"}` 先规划再开始；`POST /api/navigation/cancel` 取消；`POST /api/navigation/heartbeat` 携带 `{"token"}` 续期。`phase` 取 `idle`/`starting`/`running`/`succeeded`/`stopped`/`failed`。无 ROS 的回归检查：`python3 scripts/checks/check_web_navigation.py`（伪 ROS + 伪 SDK，不接触机器人）。

## 实时障碍参与规划

页面的“规划考虑实时障碍”默认开启，决定当前雷达观测是否进入代价地图。规划模式下它只影响全局规划；实机模式下局部代价地图读取同一组滤波点云，因此同时影响全局与局部障碍层，但仍需现场确认实际避障效果。“显示全局代价地图”和“显示实时雷达障碍点”只控制画面。

| 状态 | 规划行为 |
| --- | --- |
| 开启 | 同时考虑静态地图与实时障碍，等待新观测及代价地图更新后确认 |
| 关闭 | 用新空观测替换实时输入，清除实时障碍层并等待地图更新；静态地图障碍继续参与规划 |
| 切换中或状态未知 | 暂停提交规划；配置不匹配、雷达中断、服务超时或地图未更新会报告原因 |

切换会使旧路径失效，需要重新规划；实机导航进行中需先取消任务。多个浏览器读取同一运行状态。刷新页面或重启 Web 保留开关值；重新启动 Nav2 会话默认开启，不改写地图和长期配置。

### 用滑块调整障碍判定

| 滑块 | 范围 | 含义 |
| --- | --- | --- |
| 点群判定半径 | 0.03–0.50 m | 单帧三维球形邻域；半径内的点互相提供支持 |
| 最少支持点数 | 1–50 点 | 包含该点本身；达到门限才标记障碍，1 表示不做密度筛选 |
| 障碍膨胀距离 | 0–1.50 m | Nav2 全局膨胀层的半径，同时作用于静态与实时障碍 |

先按实际代价地图配置的高度与感知距离选取候选点，再做邻域支持判定。每帧独立计算，不把多帧累积或整串连通点当作局部共识。半径越大或最少点数越低，越容易保留障碍点；反向调整会筛掉更多稀疏点。默认半径 0.15 m、最少 1 点，膨胀距离读取实际全局代价地图参数（仓库默认 0.60 m）。这些是初始值，不代表现场调参已经验收。

拖动时预览数值，松手后提交三项设置。页面在更新期间暂停规划，等新点云、参数读回和代价地图更新确认后显示“已生效”。旧路径失效，需重新规划。页面显示最近一帧输入、候选和障碍点数，紫色图层仍显示原始实时点，不会随密度筛选隐藏。膨胀距离是代价影响范围，实际绕行距离还取决于机器人轮廓和规划器。

参数保存在 Nav2 会话，刷新浏览器或重启 Web 后读取原值；新 Nav2 会话重新使用默认点群设置及启动参数中的膨胀距离。失败会恢复原设置和观测；恢复未确认时保持状态未知，检查日志后重启 Nav2 会话。

Foxy 的[膨胀层源码](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_costmap_2d/plugins/inflation_layer.cpp)在初始化时读取半径，Humble 的[实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/inflation_layer.cpp)支持运行时参数回调。因此 Foxy 修改膨胀距离会暂时停用并重新配置规划器，重新加载全局代价地图；Humble 直接使用参数回调。改点群半径和点数不重载规划器。

PC2 使用系统 Python，需要 NumPy、SciPy（至少 1.3，使用 KD-tree 邻域查询）。`01_self_check.sh robot` 与 Web 入口会检查依赖；缺少时在 PC2 安装 `python3-numpy python3-scipy` 后重试。更新本功能需同步 `scripts/navigation/`、Web 和导航 launch，并重新启动 Nav2 会话。

### 谁管理输入与状态

`g1_navigation.launch.py` 的 `mode:=plan` 与 `mode:=real` 都会启动 `scripts/navigation/planning_obstacles.py`，由该 Nav2 会话管理节点生命周期和开关状态：

```text
FAST-LIO 机体云 → 规划障碍节点 → 筛选后的标记点 + 原始清除射线 → 全局代价地图 → 全局规划器
                         ↑ ROS 状态/设置服务
                   Web 后端 ← 浏览器
```

标记点使用 `/g1/planning_obstacles/cloud`；清除射线使用 `/g1/planning_obstacles/clearing_cloud`，后者不做密度筛选，防止孤立点过滤减少旧障碍的射线清除。原始机体云继续供定位和网页显示。Web 后端的 `obstacle_control.py` 只调用状态/设置服务，不承担点云转发。退出 Web 不会中断 Nav2 的障碍输入；规划障碍节点意外退出会结束该 Nav2 会话。仿真 `mode:=full` 保持原来的全局/局部障碍链路。

无需额外传入障碍话题参数，统一入口与手动 `mode:=plan` 使用同一配置。仅更新 Web 而仍运行旧 Nav2 配置时，页面会报告状态不可用；首次升级须停止旧 Nav2 和 Web，再启动本版。

### 切换如何确认

节点检查实际障碍源、插件类型、定位 TF 和清除范围。关闭时先发送新空观测，再调用全局 `ClearCostmapExceptRegion(reset_distance=0)`，最后等待全局代价地图更新；开启也等待新实测观测和地图更新。静态图层与膨胀图层不作为清除对象。失败恢复原输入并显示未知状态，后续轮询重新核对。

兼容性依据：[Foxy 障碍层](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_costmap_2d/plugins/obstacle_layer.cpp)没有 Humble 的动态 `enabled` 回调；[Foxy 清除服务](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_costmap_2d/src/clear_costmap_service.cpp)使用 `clearable_layers`，[Humble 清除服务](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/clear_costmap_service.cpp)改用插件的 `isClearable()`。因此本开关控制输入链。

Web 使用 `GET /api/state` 中的 `planning_obstacles` 读取状态，`POST /api/planning-obstacles` 接收 `{"enabled": true|false}` 或 `{"thresholds": {"neighbor_radius": 0.15, "min_points": 3, "inflation_radius": 0.6}}`，两种请求分别提交。阈值通过 ROS `SetParametersAtomically` 类型的独立设置服务提交，返回读回状态。规划前后重新核对节点会话和状态版本；切换期间计算出的旧路径不返回成功。

无需 ROS 的回归检查：`python3 scripts/checks/check_planning_obstacles.py`；点云筛选和阈值事务用 `python3 scripts/checks/check_obstacle_thresholds.py`（需要 NumPy/SciPy）。另用 `check_navigation_architecture.py` 核对两种 Nav2 模式及参数边界，`check_process_preflight.py` 核对进程清理范围。离线检查不代替 Foxy/Humble 部署后的服务、真实代价地图与现场规划验收。

## Foxglove 浏览器可视化

PC2 上的 `foxglove_bridge.py` 使用 Foxglove 的 Python WebSocket 库。
依赖装在 `~/ldy/nav-web-bridge/foxglove/lib`，版本为
`foxglove-websocket==0.1.2` 和 `websockets==11.0.3`。

从 Mac 用 SSH 将本地 `127.0.0.1:8765` 转发到 PC2 同端口。PC2 终端加载
Foxy 和项目 overlay，设置与地图和定位节点一致的 ROS 环境，再执行：

```bash
export PYTHONPATH="$HOME/ldy/nav-web-bridge/foxglove/lib:${PYTHONPATH:-}"
python3 scripts/real_robot/web/foxglove_bridge.py \
  --cloud "$HOME/ldy/g1_real_data/maps/hang_A_02_plaster_preview/clean.pcd"
```

在 Foxglove 浏览器客户端选择 Foxglove WebSocket，填入
`ws://127.0.0.1:8765`。添加 3D 面板，固定坐标系设为 `map`，
启用 `/plaster/clean_cloud`、`/map`、`/global_costmap/costmap` 和 TF。
`/localization/scan_cloud` 由现场定位节点提供。点选目标后的规划与实机导航仍通过原网页提交。

## 定位前检查导出地图

在 PC2 执行 `./scripts/real_robot/05_web_navigation.sh "$G1_DATA/maps/B1_run03_map" --review --port 18082`（替换为实际地图目录）。Mac 建立 `ssh -N -L 18082:127.0.0.1:18082 unitree@<当次无线IP>`，打开 `http://127.0.0.1:18082/`。

此模式只读加载 trinary 地图 YAML 和图像，不启动 ROS/定位/Nav2；支持缩放、拖动平移、点击查询栅格状态及地图坐标。地图原点显示红色十字。结果用于人工地图核对，不自动证明地图质量或机器人净空。地图在启动时加载，重新导出后需重启检查服务。仅回环监听并限制 Host；检查结束 Ctrl+C 停止。

二维地图下方显示同目录的 `plaster.png`。如文件未生成，页面会提示先安装依赖并执行 `02 export --plaster`；已有地图可按 WORKFLOWS 的说明单独补充石膏预览。生成后重启检查服务。石膏预览仅用于结构检查，不支持导航坐标选取。

石膏预览优先读取 `surface.ply` 显示交互三维网格：左键拖动旋转、右键拖动平移、滚轮缩放，并提供俯视和重置按钮。Three.js 0.160.1、OrbitControls 和 PLYLoader 随页面本地部署，不依赖浏览器访问外部 CDN；许可保存在 `assets/three/LICENSE`。缺少网格或 WebGL 不可用时回退到静态 `plaster.png`，页面会标明原因。
