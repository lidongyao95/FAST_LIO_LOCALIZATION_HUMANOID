# PC2 网页仅规划

Mac 通过 SSH 转发打开浏览器。网页显示 `/map`、地图坐标系下的机器人位姿（实机使用 `body`）、可开关的全局代价地图叠层，以及本次 `ComputePathToPose` 返回的路径。在地图上点选目标，按住拖动设置朝向，然后点击「仅规划」。

日常操作优先使用[实机部署路线](../../../docs/WORKFLOWS.md#实机部署路线)中的
`05_web_navigation.sh`，它会检查持续定位、启动 Nav2 和 Web，并传入 `--robot-frame body`。
以下步骤用于需要单独启动网页的调试场景。网页只请求全局路径；实机入口不启动控制器、局部代价地图、导航行为树或速度桥。

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
python3 scripts/real_robot/web/server.py --robot-frame body
```

在 Mac 浏览器打开 <http://127.0.0.1:18081/>。页面上地图、TF 和规划器均就绪后，点选目标。点击「仅规划」会提交一个规划 Action。现场使用实时时钟，回放如使用仿真时钟，则给网页进程追加 `--ros-args -p use_sim_time:=true`。Ctrl+C 停止网页进程。

全局代价地图来自 `/global_costmap/costmap_raw`。红色表示障碍或不可通行的内切膨胀区，橙色表示膨胀代价。走行目标和取消走行属于后续功能。

## 实时障碍参与规划

页面的“规划考虑实时障碍”默认开启，决定当前雷达观测是否进入全局代价地图。它用于比较全局规划结果，不代表机器人会执行路径或进行行走中的局部避障。“显示全局代价地图”和“显示实时雷达障碍点”只控制画面。

| 状态 | 全局规划行为 |
| --- | --- |
| 开启 | 同时考虑静态地图与实时障碍，等待新观测及代价地图更新后确认 |
| 关闭 | 用新空观测替换实时输入，清除实时障碍层并等待地图更新；静态地图障碍继续参与规划 |
| 切换中或状态未知 | 暂停提交规划；配置不匹配、雷达中断、服务超时或地图未更新会报告原因 |

切换会使旧路径失效，需要重新点击“仅规划”。多个浏览器读取同一运行状态。刷新页面或重启 Web 保留开关值；重新启动 Nav2 会话默认开启，不改写地图和长期配置。

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

`g1_navigation.launch.py mode:=plan` 自动启动 `scripts/navigation/planning_obstacles.py`，由该 Nav2 会话管理节点生命周期和开关状态：

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
`/localization/scan_cloud` 由现场定位节点提供。点选仅规划仍通过原网页提交。
