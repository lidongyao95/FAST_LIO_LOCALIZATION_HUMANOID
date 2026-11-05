# 2026-11-05 悬吊条件下的办公室推进方案

本文是待执行方案。现场放行后，可在办公室内人工搬运吊架，为 G1 提供传感器观测轨迹。A 段验收办公室三维建图，B 段验收给定近似初值的重新定位和持续定位，C 段验收 TF、二维地图、规划和代价地图。各段按实际设备与证据分别判定。

步态、平衡、行走振动下的建图与定位、实际避障、到点精度、停止、接管和断链行为留待真实行走验证。人工搬运的轨迹只用于采集和观察定位连续性。

## 1. 现场放行

开始吊装、摆位或搬运前，由现场负责人逐项确认并签记：

- [ ] 厂家或现场负责人批准的整机、吊点、吊具和工况。记录吊具型号、额定载荷和批准依据。
- [ ] 与整机及运控固件对应的允许状态、腰部状态和现场停止手段。记录固件号、序列号及实际状态。全程保持速度桥与机器人运控链隔离，禁止发出运动指令。
- [ ] 人员和路线。至少两人，一人负责搬运和停止，一人负责观测和记录；现场负责人按设备与路线确认是否需增加人员。采集前确认作业区和搬运路线。
- [ ] 确认采集项目、允许姿态和运动范围。记录雷达安装位置、朝向、机体姿态与离地高度，作为本轮地图与定位条件。

任一放行项缺失，转用已有录包离线工作。人员离岗、吊具或姿态异常、路线受阻、出现非预期运控状态或现场失去有效观察时，按批准的停止程序停工，复核后再继续。已有证据见[真机调试记录](2026-11-04_g1_pc2_slam.md)。

## 2. 环境、采集和时间检查

在 PC2 的每个采集终端使用同一工作区与 ROS 环境。以下路径按实际部署核对。每轮分配新的运行编号，避免覆盖原始数据或地图。

```bash
export G1_REPO="$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export G1_DATA="$HOME/ldy/g1_real_data"
export ROS_DISTRO=foxy ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=1
source "$G1_REPO/scripts/walking_env.sh"
mkdir -p "$G1_DATA"/{bags,maps,records}
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

1. 检查 `ip -br addr`、`ping -c 20 192.168.123.120` 和本地存储余量。连通性通过后，按[部署指南 §5](../g1_real_robot_slam_pc2_b300_guide.md#5-接入雷达确认输入可以用于算法)核对 MID-360 配置、地址和实际消息。
2. 在正式采集前及换电后执行 `timedatectl`、`chronyc tracking`、`chronyc sources`，记录同步状态和偏差。曾观测到约 680 s 偏差；主机未同步时暂停实时采集并复核。
3. 启动驱动。本仓库的 launch 安装在 `share/livox_ros_driver2/launch_ROS2/` 子目录，`ros2 launch livox_ros_driver2 msg_MID360_launch.py` 会报找不到文件，需给绝对路径：`ros2 launch "$G1_REPO/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"`。在放行条件内采集静止基线，例如 `ros2 bag record -o "$G1_DATA/bags/static_02" /livox/lidar /livox/imu`。录制 10–15 s 后停止，用 `ros2 bag info "$G1_DATA/bags/static_02"` 复核两路计数与时长，抽查静止加速度模长及方向，记录统计窗口。
4. 分别检查实际采样率、各路消息头连续性与单调性、点云帧内逐点时间、点云与 IMU 的动态对齐。历史约 10 Hz 点云与 200 Hz IMU 只作为量级参照。静止数据支持静止检查，动态对齐需另用受控运动样本验证。

**话题名。** 2026-11-05 实测该驱动配置只发布 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`，`xfer_format=1`）和 `/livox/imu`，没有 `/livox/custom_msg`；本仓库的 launch 文件未做话题重映射。若沿用 `/livox/custom_msg` 这一名称，须先自行在 launch 中加 remapping，否则录包和 FAST-LIO 都订不上。细节见[部署与构建排障 §12](2026-11-05_pc2_build_pitfalls.md)。

跨机观察按部署指南附录 A 配置 DDS。离线运行使用独立 ROS 域，各回放终端配置一致，并检查同域无实时驱动和重复播放器。

### 2.1 实时可视化：PC2 运行 RViz，Mac 通过 NoMachine 查看

现场采集时建议由观测人员打开 RViz，及时发现遮挡、重影或定位跳变。PC2 运行 NoMachine ARM64 服务端及 RViz，Mac 运行 NoMachine 客户端。RViz 与 ROS 节点同在 PC2，可沿用本轮的 ROS 域、RMW 和 `ROS_LOCALHOST_ONLY=1`。PC2 无显示器时，先确认已有可用桌面环境，并在远程会话中用 `static_01` 录包验证 RViz 渲染与点云显示。[PC2 部署指南附录 A](../g1_real_robot_slam_pc2_b300_guide.md) · [NoMachine 无显示器配置说明](https://kb.nomachine.com/AR03P00973)

| 阶段 | RViz 设置与观察内容 |
|---|---|
| A 段建图 | **Fixed Frame** 设为 `camera_init`。添加 **PointCloud2**，订阅 `/cloud_registered_1`；添加 **Odometry**，订阅 `/Odometry_loc`。观察墙面重影、吊具回波和覆盖范围。 |
| B 段定位 | **Fixed Frame** 设为 `map`。用 **PointCloud2** 叠加本轮定位地图与扫描，例如 `/localization/map_cloud` 和 `/localization/scan_cloud`，观察门框、墙和拐角是否叠合。 |
| C 段仅规划 | **Fixed Frame** 设为 `map`。由 `map_server` 加载 A 段导出的二维栅格 YAML，在 RViz 添加 **Map** 并订阅 `/map`，查看占用、空闲和未知区域。按本机实际话题叠加 **Map** 类型的代价地图和 **Path** 类型的规划路径；以 `ComputePathToPose` 的返回结果为验收依据。 |

本方案的二维底图来自已保存的 PCD，导出并加载后即可在 NoMachine 中查看。搬运期间持续更新的是三维扫描和已配置的代价地图；若要在 A 段同时看到二维底图逐步铺开，需要额外接入实时二维投影。远程桌面帧率仅用于观察，性能以 PC2 本机的时间戳、日志和录包为准；资源验收时可分别记录开关 RViz 与远程桌面的负载。

## 3. A 段：安装方向核查与三维建图

### 3.1 核查坐标与吊具影响

| 观察项 | 通过证据 |
|---|---|
| 原始传感器系与 IMU | 对照实际安装和坐标约定，记录可观测墙面或重力方向。仅测试现场批准的姿态轴。 |
| 受控转动 | 用 `/Odometry_loc` 或 `camera_init → body` 核对姿态变化的符号和量级。RViz 固定在 `camera_init` 时，配准扫描中重叠的静态墙面应保持叠合。 |
| 受控平移 | 对照实测起终点，检查已测方向的位移符号与尺度。分别记录各实际测试方向。 |
| 吊具影响 | 检查原始扫描遮挡、随体近场回波和地图残留；有影响时记录范围并在现场批准的工况内解决后复测。 |

安装方向与雷达 IMU 到机体的安装外参分别记录。确认当前驱动路径与既有补偿后再决定 `common.input_roll_180`，每次只改一项并用同一录包复核。[倒装输入补偿记录](../inverted_mid360.md)

### 3.2 采集、实时建图或离线建图

**实时分支。** 驱动发布正常后启动下方 FAST-LIO，再用独立终端录制 A 段。确认节点已完成 IMU 初始化，沿批准路线采集目标区域，保留共同可见结构和可辨认的起终点。保持缓慢连续，出现明显摆动或持续积压时暂停采集。记录现场门状态、人员与可见障碍。

```bash
ros2 bag record -o "$G1_DATA/bags/hang_A_01" /livox/lidar /livox/imu
```

**离线分支。** 在隔离 ROS 域检查自定义消息兼容性和 `ros2 bag play -h`，先启动 FAST-LIO，再播放本轮 A 包。PC2 Foxy 已有不带 `--clock` 的静止回放记录；此处仅做使用原始消息头的 FAST-LIO 建图。跨节点按时间查 TF 的回放在 B/C 段另行复核，降低回放速率的结果单独标注。

两个分支均可用以下建图命令。示例输出名按本轮替换，先核对目标不存在、配置中的 `pcd_save.pcd_save_en` 已打开、目标目录可写。参数文件用仓库内置的 [mid360.yaml](../../FAST_LIO/config/mid360.yaml)，其中 `lid_topic` 仍是 `/livox/custom_msg`，与实际话题不一致时在命令行上重映射：`-r /livox/custom_msg:=/livox/lidar`。保留 `lidar_type: 1`，该参数使节点订阅 `CustomMsg` 并进入 `avia_handler`。

```bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$G1_REPO/FAST_LIO/config/mid360.yaml" \
  -r /livox/custom_msg:=/livox/lidar \
  -p use_sim_time:=false \
  -p map_file_path:="$G1_DATA/maps/office_01.pcd" \
  -p debug_log_dir:="$G1_DATA/records"
```

回放分支在另一终端执行 `ros2 bag play "$G1_DATA/bags/hang_A_01"`。两个分支均可在 RViz 用 `camera_init` 查看 `/cloud_registered_1`。待地图积累完成并保持 FAST-LIO 运行时保存，再独立读回：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
ls -lh "$G1_DATA/maps/office_01.pcd"
/usr/bin/python3 "$G1_REPO/scripts/export_g1_map.py" \
  "$G1_DATA/maps/office_01.pcd" --out "$G1_DATA/maps/office_01" \
  --known-free-radius 0
```

**三维地图验收。** 确认文件来自本轮，能独立读取，点数与实际文件一致且 XYZ 有限。按采集前约定的容差记录地面高度散布、典型墙面和门框的重影、实测距离的尺度误差、回起点接缝、区域覆盖及吊具残留。失败时保留录包与证据，先定位问题再复采或回放。

**二维地图另验。** 记录裁剪高度、分辨率、原点和占用规则，检查门口、墙角及未知区。`--known-free-radius 0` 只关闭原点附近的额外空闲区填补，导出仍可能扩展部分空闲区域。二维栅格通过后再进入 C 段。

## 4. B 段：给定近似初值的重新定位与持续定位

### 4.1 部署选择

第二台 G1 的 PC2 overlay 已构建 `open3d_loc` 并通过静止回放配准。执行前用 `ros2 pkg prefix open3d_loc` 核对当前 overlay。正式 B 段可在 PC2 或 x86 上使用独立录包验证，分别记录结果。[第二台 G1 验证状态](../README.md)

| 路线 | 进入定位验收前的检查 |
|---|---|
| x86 离线 | 核对工作区、消息类型与回放时钟，保存定位参数及地图、录包标识。 |
| PC2 ARM 预编译 | 作者提供的 `open3d141_arm.zip` 标为未充分测试。核对来源与 SHA-256，在隔离前缀解包，确认 ELF 架构、glibc/libstdc++ ABI、CMake 导入目标、链接结果及离线冒烟测试。 |
| PC2 ARM 源码构建 | 在需要机载且预编译路线受阻时评估。0.14.1 是当前 C++ API 的低变更候选。上游要求 CMake ≥ 3.19.2，可尝试 `BUILD_GUI=OFF`、`BUILD_PYTHON_MODULE=OFF`、`BUILD_EXAMPLES=OFF` 与 `BUILD_CUDA_MODULE=OFF`。根据 configure 日志、缓存和链接结果核对实际依赖及工具链。 |

仓库依赖脚本跳过 ARM 包属于默认部署策略。输入地图、初值、TF 或时钟不通过时，先处理对应阻塞。版本和构建选项以[Open3D v0.14.1 构建定义](https://github.com/isl-org/Open3D/blob/v0.14.1/CMakeLists.txt)为据。

### 4.2 摆位、初值与启动

1. 选定与 A 段数据独立的 B 段。用实测地图控制点或机体参考位姿确定 3 个位置和朝向，每处做 3 次独立启动试验，覆盖不同于 A 段起点的摆位。先记录初值偏差、允许位置与航向误差、静止漂移和动态跳变限值、初始化超时。增加一组偏初值或相似结构用例，检查误配。现场摆位仍执行第 1 节放行条件。
2. 实时试验在摆位稳定后启动本轮 FAST-LIO，检查实际局部原点。离线试验在隔离域重启 FAST-LIO 和定位，从独立 B 包开头播放。需要统一回放时钟时，先确认播放器可提供 `/clock`，参与的节点统一启用仿真时间并验证消息头和 TF 查询。PC2 Foxy 已知播放器无 `--clock`；该路线缺少已验证时钟方案时，将 B/C 时间验收标为阻塞。
3. 按[部署指南 §7](../g1_real_robot_slam_pc2_b300_guide.md#7-加载地图验证重定位与实时定位)配置安装 TF 和发布权。逐项确认 `map → odom`、局部里程计以及传感器到机体的实测链。若保留旧链的 `base_link → motion_link`，启动定位时设 `publish_motion_tf:=false`，并复核唯一父节点。定位节点启动时若报告安装 TF 查询失败，补齐后重启再验收。定位点云和 Nav2 栅格分别使用独立话题，例如将定位节点的 `/map` 和 `/scan` 重映射到 `/localization/map_cloud` 和 `/localization/scan_cloud`。
4. 在 `loc_real.yaml` 写入初始 `T_map_odom = T_map_base × inverse(T_odom_base)`，其中 `T_odom_base` 包含实测安装外参。配置 `initialpose` 依次为 `[x,y,z,roll,pitch,yaw]`，位置用米，角度用度，源码按 `Rx * Ry * Rz`。ROS `/initialpose` 消息使用四元数，回调存在高 fitness 时忽略请求等限制，重复验收使用独立启动和显式初值。
5. 按指南逐节点启动定位，使用本轮 PCD 与 `loc_real.yaml`。保存 `ros2 param dump /global_localization_node`。分别记录 `threshold_fitness_init` 和 `threshold_fitness`，源码默认均为 0.9，指南中的 0.5 是诊断示例。阈值调整以场地误差和误配用例为依据。

### 4.3 验收信号

| 要验证的内容 | 记录方式与判定 |
|---|---|
| 初始化完成 | 记录启动耗时与初始化日志，结合独立场地参照核对位置、航向与结构叠合。当前实现需连续两次初始化 fitness 超阈值。 |
| 位姿发布与头年龄 | 用 `/localization_3d`、`/localization_3d_delay_ms` 和里程计观察频率、头年龄、停顿与跳变。定位话题在初始化后随里程计回调发布；delay 是节点时间减里程计消息头时间。 |
| 配准尝试与耗时 | 区分初始化和稳态日志，统计尝试间隔、P95 耗时及跳过原因。默认 `loc_frequence=2.0` 对应稳态尝试等待参数，实际间隔还受计算和输入影响。 |
| 配准接受与修正 | 核对稳态 fitness、有效阈值及修正证据，记录接受率、连续拒绝、修正幅度与最后接受距今。节点在通过稳态阈值时才写入修正；`/localization_3d_confidence` 可能重发缓存值，`map → odom` 可随里程计继续广播。无法追溯接受事件或时间时，该项标为未测。 |
| 精度与连续性 | 每处记录实测参考位姿、初始化误差、重复性及相似区域叠合，按预先约定的容差判定。动态连续性按消息时间检查位姿和修正幅度。 |

源码入口见[位姿与 TF 发布](../../open3d_loc/src/global_localization.cpp#L485)及[稳态配准接受门槛](../../open3d_loc/src/global_localization.cpp#L977)。

### 4.4 资源与平台对照

在 PC2 分开测量驱动加 FAST-LIO 基线、定位冷启动、稳态静止和已批准的动态样本。静止可先安排 10 分钟窗口。启动前确定采样间隔、时长、内存余量、允许超时次数和输入积压上限。用实际 PID 分别采样（如 `pidstat -u -r -p PID 1`），记录 RSS、CPU、系统 `MemAvailable`、swap、温度与 CPU 频率（若可读）、输入与输出时间差和丢帧。采样覆盖加载地图与估计法向阶段；内存容量以系统证据为主，避免共享页重复计入。

跨平台对照使用同一地图、B 包、初值、有效参数和重复次数。分别保存位姿误差、fitness、尝试与接受次数及耗时。x86 离线结果与 PC2 机载实时结果分别判定。若需研究速度影响，使用已有动态录包或在批准范围内记录观测，不安排加速直至定位失效。

## 5. C 段：TF、二维地图与 Nav2 仅规划

先确认 A 段二维栅格、B 段定位证据、实际安装 TF 与本机规划组件可用。真机现场仅启动地图服务器、planner server、所需代价地图和生命周期管理。保持速度桥、机器人运控、导航行为树、控制器与恢复行为入口隔离。现有 `scripts/validate_nav_goal.py` 调用 `NavigateToPose`，用于导航链验证时应放在独立离线或仿真任务中。

1. 检查地图与规划节点 active、TF 发布者唯一、代价地图可更新。核对 `ros2 action list -t` 及 `ros2 interface show nav2_msgs/action/ComputePathToPose`。使用实际 action 名称和实装字段发送 `map` 下的规划目标；Foxy 上游定义使用 `pose` 与 `planner_id`。[Foxy 动作定义](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_msgs/action/ComputePathToPose.action)
2. 验证三个用例。可通行目标返回有效路径；阻断用例按预期绕行或返回无路径；在已确认配置了对应障碍层的代价地图上，分别观察障碍标记与射线清除。记录 action 状态、路径、耗时、传感器输入、frame、时间戳和截图或录包。
3. 规划超时按异常记录，检查 planner 生命周期、TF 时钟与连通性、代价地图更新和目标占用状态，修复后复测。人工搬运时按已批准路线单独观察定位连续性。

## 6. 记录和回写

每次运行登记场景、机器与架构、实测或回放、ROS 版本与域、RMW、代码提交和工作区差异、录包与地图标识、节点有效参数、TF 发布权、时钟、门槛、判定及证据路径。现场轮次另记批准依据、人员、吊具、停止手段、机体状态及雷达安装。保留原始包；地图、配置和记录使用新编号。

执行后按[真机验证状态](../README.md)实际行名回写，将方向和外参证据归入“硬件与运控”，A 段归入“B300 建图”并注明实际办公室范围，B 段的 x86 离线和 PC2 机载及 C 段 TF/仅规划证据归入“重定位与导航”。分别标明已验证、证据不足、阻塞或超出本轮范围。静止包、PCD 保存、离线定位和现场规划各自只支持对应结论。
