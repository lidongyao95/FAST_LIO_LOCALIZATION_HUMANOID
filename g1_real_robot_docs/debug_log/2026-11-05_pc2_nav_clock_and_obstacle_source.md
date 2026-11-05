# 2026-11-05 C 段规划链、时钟与障碍源的离线验证

本文记录在第二台 G1 的 PC2 上完成的 C 段前置工作。PC2 只做隔离域的合成测试与已有录包回放。运动目标仅在 4322 的 MuJoCo 仿真中执行。现场放行与 A/B 段实测仍按[办公室推进方案](2026-11-05_suspended_office_plan.md)执行。

## 1. 结论

| 项目 | 结果 | 依据 |
|---|---|---|
| C 段规划链 | 通过。map_server + planner_server + `ComputePathToPose` 可在 PC2 运行，不需要 `nav2_bringup` | 三个用例返回路径或空路径 |
| 障碍层输入 | 已修复。根因是导航参数沿用了仿真的 `/livox/lidar` + `PointCloud2`，真机该话题是 `CustomMsg` | 驱动源码与节点端点类型 |
| `map → body` 链路 | 回放期间可解析 | `static_01` 的 6 次查询均成功 |
| 离线时钟 | 单次静止回放验证通过。新增 `/clock` 发布器，中位领先 94.8 ms | 有/无对照实验 |
| 障碍层标记与清除 | 双端受控栅格对照通过 | 近处障碍格被标记，后续回波穿过该格时可清除 |
| 距离兼容 | 双端最大距离配置对齐 | 4.025 m、7.875 m 标记，8.125 m 无致命标记 |
| 硬编码路径 | 已清理四处 | 参数与 launch 实测 |
| C 段真机入口 | 通过。`mode:=plan` 只起 map_server、planner_server 与其内部 global costmap；参数整份重写后 `use_sim_time` 与地图均生效 | 双端节点集、参数读回与规划用例 |

## 2. C 段规划链预验证

### 2.1 静止 PCD 不能用于二维底图

用 `scripts/export_g1_map.py` 从 `static_check_01.pcd` 导出（`--known-free-radius 0`）：810×435 栅格、分辨率 0.1 m，free 3120、occupied 7080、unknown 342150。free 质心在 (4.87, 0.05)，而**原点 2 m 内 free 栅格为 0、原点自身是 unknown**。该图不可用于规划，二维地图验收只能等 A 段正式录包。

### 2.2 合成图与三个用例

改用合成 10×10 m / 0.05 m 图：隔断墙在 x=0、y=-5..3，y=3..5 留通道；封闭盒 x=1.5..2.5、y=-2..-1。机器人位姿由 `map → base_footprint` 静态 TF 给在 (-2.0, 0.0)。

| 用例 | 目标 | 结果 |
|---|---|---|
| 需绕行 | (3.0, 0.0) | `SUCCEEDED`，376 点，长 9.42 m，最大 y=3.64（经通道），墙带内 0 点 |
| 同侧短距 | (-0.5, 1.0) | `SUCCEEDED`，72 点，长 1.83 m（直线距离 1.80 m） |
| 封闭盒内 | (2.0, -1.5) | `ABORTED`，空路径 |

首次执行时三个用例全部 `ABORTED`，原因是把机器人起点放在了 (0,0)，正好落在隔断墙格子里。移到 (-2.0, 0.0) 后复现表中结果。规划失败时的日志只有 `failed to create plan with tolerance`，不会指出起点在障碍内，现场排查时先查起点与 `allow_unknown`。

**Foxy 结构。** `planner_server` 进程内部创建 `/global_costmap/global_costmap` 与 `/global_costmap/global_costmap_rclcpp_node`，参数取自同一 YAML 的 `global_costmap` 段。因此只起 map_server、planner_server、生命周期管理与一个 `map → base_footprint` 的 TF 即可，无需 `nav2_bringup`，也无需单独启动代价地图节点。

## 3. 障碍层的消息类型问题

### 3.1 根因

[generate_nav_params.py](../../scripts/generate_nav_params.py#L23) 原先把障碍源固定成 `/livox/lidar` 加 `data_type: PointCloud2`。这是仿真形状：[g1_mujoco_sensors.py](../../scripts/g1_mujoco_sensors.py#L106) 在仿真里用 `PointCloud2` 发同一个话题名。真机驱动在 `/livox/lidar` 发的是 `livox_ros_driver2/msg/CustomMsg`，两者类型不同，障碍层收不到任何数据；这也是 4322 仿真里代价地图正常、PC2 上异常的原因。生成器该行本次已同步改为 `/cloud_registered_body_1`，避免重新生成时回退。

代价地图侧的订阅端点在 `/global_costmap/global_costmap_rclcpp_node`，类型 `sensor_msgs/msg/PointCloud2`，best effort。

### 3.2 为什么不切驱动的 `xfer_format`

驱动只有一个 `transfer_format_`，两种格式共用话题名 `livox/lidar`，互斥：

| `xfer_format` | 类型 | 逐点时间 |
|---|---|---|
| 1（当前） | `CustomMsg` | 真实 `offset_time` |
| 0 | `PointCloud2`（x,y,z,intensity,tag,line,timestamp） | 字段存在 |

切到 0 需要 FAST-LIO 侧改 `lidar_type: 4`（`MID360`），而 [mid360_handler](../../FAST_LIO/src/preprocess.cpp#L496-L549) 把 `given_offset_time` 强制为 false，改用方位角推算逐点时间。MID-360 是非重复扫描，该推算不成立，会退化去畸变，直接损伤 A 段建图质量。驱动源码见 [lddc.cpp](../../.deps/src/livox_ros_driver2/src/lddc.cpp#L262-L296)。

### 3.3 改为使用 FAST-LIO 的机体点云

FAST-LIO 自身发布 `PointCloud2`：`/cloud_registered_1`（`camera_init`）与 `/cloud_registered_body_1`（`body`），见 [laserMapping.cpp](../../FAST_LIO/src/laserMapping.cpp#L985-L986)。改动为 [g1_nav2_params.yaml](../../navigation/g1_nav2_params.yaml#L162-L165) 中 global/local 两处 `obstacle_layer.lidar.topic` → `/cloud_registered_body_1`。

机体云发布函数遍历 `feats_undistort`，转换到 IMU 的 `body` 坐标系，使用扫描结束戳。`dense_publish_en` 只影响世界系云的稠密分支。早期约 900 点/帧的统计来自 `/cloud_registered_1`。PC2 复测实际收到 131 帧机体云，每帧最小／中位／最大为 4,551／4,608／4,665 点。输出依赖 FAST-LIO 完成初始化，并开启 `scan_publish_en` 和 `scan_bodyframe_pub_en`。 [发布实现](../../FAST_LIO/src/laserMapping.cpp#L582)

Foxy 障碍层读取 `obstacle_range`、`raytrace_range`。Humble 使用 `obstacle_min_range`、`obstacle_max_range`、`raytrace_min_range`、`raytrace_max_range`。原 YAML 只有 Humble 键，双端对照中 4 m 处只有 Humble 标记。现已在 global/local 两处及生成器补齐 Foxy 键，双端复测见第 10 节。Foxy 的 0.4 m 下限仍需单独评估。 [Foxy 障碍层实现](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_costmap_2d/plugins/obstacle_layer.cpp#L105-L165)

## 4. `map → body` 链路

链路为 `map → odom`（定位节点）→ `odom → camera_init`（[静态恒等](../../open3d_loc/launch/g1_walking_localization.launch.py#L50-L51)）→ `camera_init → body`（FAST-LIO）。另有 `odom → base_footprint`、`base_footprint → base_link` 由 [g1_nav_tf.py](../../scripts/g1_nav_tf.py) 发布，安装 TF 由现场按实测外参提供。

回放 `static_01` 并启动定位节点（零初值、`publish_motion_tf:=false`）后连续采样 6 次：

| 目标 | 结果 |
|---|---|
| `map → body` | 全部解析成功，(-0.017…-0.004, ±0.005, -0.016…-0.010) |
| `map → base_footprint` | 成功，z = -0.792（`foot_height` 0.793） |
| 逐段链路 | `map→odom`、`odom→camera_init`、`camera_init→body`、`odom→base_footprint`、`base_footprint→base_link`、`base_link→imu_link`、`motion_link→base_link` 均可解析 |

数值贴近恒等，与"地图本体就在 `camera_init` 系、`initialpose` 为零"一致。

**采样方法。** TF 发布者使用 volatile 的 DDS 语义，探针必须在发布者在线期间启动，否则只能收到 `/tf_static`。第一次采样把探针放在回放结束之后，`map`、`body` 全部报 `target_frame does not exist`，属方法错误而非链路缺陷。

采样时变换戳龄约 5,758 s，因为回放的是 11:34 的原始时间戳而当前时钟已是 13:07。这条直接引出下一节。

## 5. `replay_clock.py` 与离线时钟

PC2 的 `ros2 bag play` 没有 `--clock`，仓库此前也没有 `/clock` 发布器。`use_sim_time:=true` 的节点在回放中时间停在 0，代价地图的更新定时器不走，生命周期也停在 `Checking transform`。

新增 [replay_clock.py](../../scripts/replay_clock.py)：订阅 `/livox/imu`（200 Hz），把 `header.stamp + lead_ms`（默认 100 ms）发布为 `/clock`。时钟随 IMU 回放推进，数据停顿时时钟也停止。脚本丢弃非递增戳，循环播放或向后跳转时应重启相关节点，并保证隔离域内只有一个 `/clock` 发布者。

FAST-LIO 无需改动：[common_lib.h](../../FAST_LIO/include/common_lib.h#L262-L268) 的 `get_ros_time` 只做数值转换、不读系统时钟，其消息头与 TF 始终是 bag 时间，因此保持 `use_sim_time:=false`。

| 检查 | 结果 |
|---|---|
| 时钟跟踪 | `now − imu_stamp` 中位 94.8 ms、最大 100.0 ms |
| 定时器 | 1 Hz 定时器在 13 s 回放内触发 12 次 |
| 有 `/clock` | 代价地图正常更新，障碍层收到 129 帧并产生标记 |
| 无 `/clock`（对照） | 代价地图从不发布，生命周期停在 `Checking transform` |

单次单调回放可先启动 `replay_clock.py`，再让依赖回放时间的消费者使用 `use_sim_time:=true`。94.8 ms 是时钟相对 IMU 戳的观测偏移，端到端处理延迟和 TF 可用性需另测。

## 6. 障碍层标记与清除的离线复现

用改后的仓库参数（仅替换 `map_server.yaml_filename`）加 `replay_clock.py` 回放 `static_01`：

| 检查项 | 结果 |
|---|---|
| 生命周期 | `map_server`、`planner_server` 均 active |
| 静态层 | `StaticLayer: Resizing costmap to 200 X 200 at 0.050000 m/pix` |
| 障碍层输入 | 131 帧 `/cloud_registered_body_1` |
| 占用栅格 | 14892 → 14771 → 14934 → 14768，仅表明合成代价地图发生变化，标记与清除分别由第 10 节的指定栅格对照验证 |
| 告警 | 无 |

## 7. 硬编码路径与参数覆盖优先级

清理的四处：`g1_nav2_params.yaml` 的 `map_server.yaml_filename`（原 `/home/sjtu/...`）、`generate_nav_params.py` 里写回该键的那行、`open3d_loc_g1.launch.py` 里作为 `path_map` 实参的 `/home/sax/GO2_Localization_ROS2/.../1.test.ply`、`g1_real_robot_slam_notes.md` 的 `G1_REPO` 示例。`localization_3d_g1.launch.py` 相应声明并转发 `path_map`，否则包含链会因为缺必填参数直接报错。`generate_nav_params.py` 的障碍源同步改为 `/cloud_registered_body_1`，避免重新生成时把第 3 节的改动覆盖回去。

**参数覆盖踩坑。** 在本轮 PC2 的启动方式下，参数文件中重复的地图键实际生效：`map_server` 传了 `-p yaml_filename:=...` 仍去读文件里的开发机路径。launch 的参数字典也被写成临时参数文件。本轮遇到的优先级仅适用于已记录的启动方式，复现时应核对实际 argv、参数文件顺序和节点全名。把该键从文件里去掉后，`-p` 与 launch 字典都能覆盖，`configure` 成功加载指定地图。

由此得到两条可操作规则：

1. 需要现场覆盖的参数不要在参数文件里写死；`loc_param_g1.yaml` 未声明 `path_map`，所以 `-p path_map:=` 一直有效。
2. 本轮地图加载失败后继续调用 `activate`，观察到节点退出。排查时保存 `configure` 的返回结果并查询实际生命周期状态。

验证方式：参数文件去掉该键后 `ros2 param get /map_server yaml_filename` 返回给定路径且 `configure` 成功；重编 `open3d_loc`（0.92 s）后，`ros2 launch open3d_loc open3d_loc_g1.launch.py path_map:=<pcd>` 的节点参数等于给定路径，不传则明确报 `missing required argument 'path_map'`。

## 8. 未决与现场待验

- **实时验收。** 用正式地图、实测外参和独立动态数据核对定位、global/local costmap、低矮障碍、近距遮挡及移除后的清除。导航 launch 的默认值仍是仿真地图与模拟时间，现场按第 11 节与[推进方案 §5](2026-11-05_suspended_office_plan.md#5-c-段tf二维地图与-nav2-仅规划)的 `mode:=plan` 入口显式给出模式、地图与时间源。
- **高度与射线原点。** 高度筛选分别在 global 的 `map` 和 local 的 `odom` 中完成。应实测两个坐标系中的地板高度再定阈值。未指定 `sensor_frame` 时，机体云的射线起点是 `body` 的 IMU 原点，需与标定的 LiDAR 原点核对。 [Foxy ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_costmap_2d/src/observation_buffer.cpp)
- **时间诊断。** 动态 TF 缓存限制的是样本历史窗口。回放时间与墙钟相差一小时，本身不足以解释按消息戳的 TF 查询失败。应记录点云戳、对应 TF 样本及失败类型。增大 `--lead-ms` 不会补出缺失 TF，也可能增加按当前时间计算的戳龄。
- **兼容范围。** Foxy 无对应的 0.4 m 最小距离参数。8 m 和 9 m 已在全局代价地图读回，局部滚动窗口内的近距行为仍需验收。

## 9. 证据

PC2 `~/ldy/g1_real_data/records/nav_check_01/`：

| 文件 | 内容 |
|---|---|
| `summary.txt` | 三轮操作的结论与踩坑汇总 |
| `planner.log`、`map.log` | 规划链预验证 |
| `nav_plan_only.yaml`、`map.yaml`、`map.pgm` | 首次验证用的参数与合成图 |
| `probe.log`、`loc.log`、`lio.log`、`tf_*.log` | `map → body` 采样与定位初始化 |
| `clock.log`、`control_no_clock_probe.log` | 时钟跟踪与无时钟对照 |
| `obstacle_marking_probe.log`、`nav.yaml` | 改后仓库参数的标记验证 |
| `launch_path_map_ok.log`、`launch_path_map_missing.log` | `path_map` 必填参数行为 |

回放与验证使用的 ROS 域为 93–111，均设置 `ROS_LOCALHOST_ONLY=1`、`rmw_fastrtps_cpp`。所有节点在每次验证后停止，PC2 无残留进程与节点。

## 10. 双端复测

两端用同一份候选参数、时钟脚本和合成地图，在隔离 ROS 域运行。PC2 为 Foxy / Nav2 costmap 0.4.7，4322 为 Humble / Nav2 costmap 1.1.20。两次迭代分别记录了补距离键前后的行为。

| 检查 | PC2 / Foxy | 4322 / Humble |
|---|---|---|
| 合成图全局规划 | 绕行成功（366 点，最大 y=3.632 m），同侧成功（72 点），封闭盒目标中止且返回空路径 | 同左 |
| 指定自由栅格，开启清除 | 近处单元由 0 变 254，加入穿过该格的更远回波后回到 0 | 同左 |
| 同一单元，关闭清除 | 由 0 变 254，更远回波到达后仍为 254 | 同左 |
| 补 Foxy 距离键前，约 4 m 的自由格 | 保持 0 | 标记为 254 |
| 实装障碍层库内的距离键 | `obstacle_range`、`raytrace_range` | `obstacle_min_range`、`obstacle_max_range`、`raytrace_min_range`、`raytrace_max_range` |
| `path_map` | 父子入口均声明该参数。子入口缺参时报错，传入 PCD 后通过参数服务读回指定路径 | 父子入口均声明该参数。子入口缺参时报错，用仓库 `walking_env.sh` 配置运行环境后，传入 PLY 并通过参数服务读回指定路径 |
| 既有数据链回归 | `static_01` 完成定位初始化。收到 131 帧 `body` 点云，每帧最小／中位／最大为 4,551／4,608／4,665 点。收到 2,683 条严格递增的 `/clock`，跨度 13.410 s | 使用候选 YAML 运行仿真导航，local/global costmap 均加载障碍层，生命周期激活。从 (0.4, −0.2) 重定位后，目标 (1.2, −0.2) 返回 `SUCCEEDED` |

受控栅格实验先用近处回波标记目标格，再发送同射线上更远的回波，直接读 `costmap_raw` 中同一格的值。该结果验证了离线全局障碍层的标记与清除，并证实当前同一 YAML 在 4 m 处产生不同标记行为。PC2 的确切距离边界及较远处的清除距离仍需按实装参数和已知距离验证。静止录包统计确认约 900 点/帧应归于另一点云话题。

### 组合定位入口

PC2 与 4322 均使用隔离 ROS 域和临时 launch 覆盖目录，运行候选 `localization_3d_g1.launch.py path_map:=<地图>`，并从子定位节点的参数服务读回父入口传入的 PCD 或 PLY 路径。使用各主机已安装二进制，探针只验证参数转发。日志在两端 `g1-nav-range.*` 目录的 `parent-launch.log`、`parent-param.log`。

### 补齐 Foxy 距离键后

共享 YAML 的 local/global 障碍层与生成器均补上 Foxy 的 `obstacle_range: 8.0`、`raytrace_range: 9.0`，同时保留 Humble 的 min/max 键。生成器在 Humble 上重新运行后，两处 lidar 参数与仓库 YAML 一致。两端在隔离域用同一候选配置和 5 cm 合成地图测试全局障碍层，分别读回各自生效的最大距离参数 8.0 m 和 9.0 m。

**该轮"两端生效"只对临时目录里的候选副本成立**：部署目录的 YAML 与生成器从未同步这两个键，按仓库文件启动时 Foxy 实际取默认 2.5 m / 3.0 m。见第 11 节。

| 合成点云用例 | PC2/Foxy 与 4322/Humble |
|---|---|
| 水平距离 4.025 m、7.875 m | 目标格均被标记为 254 |
| 水平距离 8.125 m | 目标格无致命标记。开启清除时为 0，关闭清除时邻近障碍的膨胀代价为 253 |
| 8.125 m 回波穿过 7.875 m 的已标记格 | 开启清除时由 254 降为 0，关闭清除时保持 254 |

9 m 为运行时参数读回值，行为探针验证到 8.125 m 的清除回波。证据在 PC2 的 `~/ldy/g1-nav-retest.jjFRIy/` 和 `~/ldy/g1-nav-range.sgyh86/`，以及 4322 的 `~/ldy/g1-nav-retest.xoKcMw/` 和 `~/ldy/g1-nav-range.pTAr9U/`。距离用例见 `result.log`、`control-result.log`，仿真结果见 `nav-goal.json`。4322 仿真达成目标后，`g1_nav_tf.py` 在关停阶段报告 `ExternalShutdownException`。

## 11. C 段真机规划入口与双端复测

本节把第 10 节的离线结论收敛成可现场执行的入口，并修正其中一条只对候选副本成立的判断。两端均用隔离 ROS 域、`ROS_LOCALHOST_ONLY=1`、`rmw_fastrtps_cpp`、`use_sim_time:=false`，无 `/clock`；地图为 200×200 / 0.05 m 合成图，机器人位姿由 `map → base_footprint` 与 `map → body` 静态 TF 给在 (-2.0, 0.0)。

### 11.1 规划入口参数化

[g1_navigation.launch.py](../../open3d_loc/launch/g1_navigation.launch.py) 新增 `mode`（`full` / `plan`）与 `use_sim_time`。仅规划模式要求显式给出地图和时钟参数，非法模式或时钟值在创建节点前报错。启动前把 `use_sim_time`、`map_yaml` 写入一份临时参数文件后整份下发。这样绕开第 7 节的优先级问题：临时文件内不再存在与覆盖值同名的冲突项。实测嵌套的 `global_costmap` 段也随 launch 参数变化，这是按节点字典下发时无法保证的。

`mode:=plan` 只启动 `map_server`、`planner_server` 及其内部 global costmap，不启动控制器、恢复行为和 BT navigator，也没有速度输出。[ensure_g1_nav_lifecycle.py](../../scripts/ensure_g1_nav_lifecycle.py) 改为接受节点名参数，缺省仍是原五个节点。

### 11.2 部署漂移

| 项目 | 现象 | 处理 |
|---|---|---|
| PC2 首次复测 | 4.025 m 处不标记；`obstacle_layer.lidar.obstacle_range` 读回 2.5、`raytrace_range` 读回 3.0，即 Foxy 默认值 | 判定为参数未生效，不是障碍层缺陷 |
| 根因 | 两端部署目录的 `navigation/g1_nav2_params.yaml` 与 `scripts/generate_nav_params.py` 都缺这两个 Foxy 键；第 10 节的 8.0/9.0 来自 `~/ldy/g1-nav-range.*` 里的候选副本 | 两端同步该 YAML 与生成器，SHA-256 `125f1214…`，与上次候选文件一致 |
| 同步后 | 两键读回 8.0 / 9.0，4.025 m 恢复标记 | 见 11.3 |

部署机与开发机的配置漂移只能靠反向比对发现，本轮的 `rsync -ain` 只查出这两处，其余文件无差异。

### 11.3 复测结果

| 检查 | PC2 / Foxy | 4322 / Humble |
|---|---|---|
| 节点集 | `map_server`、`planner_server`、`/global_costmap/global_costmap`；无 controller、behavior、bt_navigator | 同左 |
| `use_sim_time` 读回 | planner_server、map_server、嵌套 global costmap 均为 `False` | 同左 |
| `map_server.yaml_filename` | 传入路径 | 传入路径 |
| 障碍源 | `obstacle_layer.lidar.topic` = `/cloud_registered_body_1` | 同左 |
| 生效最大距离 | `obstacle_range` 8.0、`raytrace_range` 9.0 | `obstacle_max_range` 8.0、`raytrace_max_range` 9.0 |
| 规划用例 | 366 点 9.18 m（最大 y 3.63）、72 点 1.83 m、封闭盒 `ABORTED` 空路径 | 同左 |
| 标记与清除 | 近处 0→254，穿透回波后→0；4.025 m →254 | 同左 |
| 重编 `open3d_loc` | 3.94 s，退出 0 | 36.0 s，退出 0 |

`mode:=full` 在 4322 冒烟通过：五个节点全部起齐，`map_server`、`planner_server`、`controller_server` 均为 active；无仿真源与 `odom` 的首轮冒烟中，局部代价地图等待 TF，生命周期助手最终超时。之后在具备仿真 TF 与时钟的完整回归中，生命周期助手成功，目标 (1.2, −0.2) 返回 `SUCCEEDED`。仿真默认值未变，`start_g1_navigation.sh` 现在把它校验过的 `map_yaml` 显式传入。

### 11.4 两个操作注意点

- 临时参数文件在关停时删除，路径同时打进启动日志（`params=/tmp/g1_nav_params_*.yaml`）便于事后核对。实测 `Ctrl-C`（SIGINT）后 `/tmp` 无残留；直接 `kill -TERM` 或 `SIGKILL` 不触发 launch 的 shutdown 事件，会留一个文件，需手工清理。
- Foxy 上 `ros2 lifecycle get /global_costmap/global_costmap` 返回 `unconfigured`，但日志有 `Activating` / `start`，且 `costmap_raw` 正常发布。判定代价地图状态应看 `costmap_raw` 是否有数据或参数读回，不要用这条命令。该行为来自上游：Foxy 的 `planner_server` 直接调用代价地图自身的 `configure()` / `activate()`，不驱动子节点的生命周期状态机，本仓库侧改不了。

证据：PC2 的 `~/ldy/g1-naventry.JC6sgZ/`（`inspect.log`、`result.log`、`range.log`、`launch.log`、`run.sh`、`probe_entry.py`、`probe_range.py`），4322 的 `~/ldy/g1-naventry.UaO2de/`（`inspect.log`、`result.log`、`inspect-full.log`、`full-launch.log`）。两端验证后进程已停止。

### 11.5 启动失败保护

PC2/Foxy 与 4322/Humble 均验证了四种启动拒绝条件：拼错模式、非法时钟值、仅规划缺地图、仅规划缺时钟。四种情况均在启动节点前退出，返回码为 1。将生命周期助手替换为立即退出 7 的探针后，两个平台都发送关停信号、清理临时参数文件，launch 返回 1。Foxy 的退出回调须显式发送关停事件，单独抛异常只会记录错误。实际生命周期超时日志会附上最后一次查询中尚未激活的节点和状态。

证据目录为 PC2 的 `~/ldy/g1-nav-guard.eSEzhs/` 和 4322 的 `~/ldy/g1-nav-guard.qP2H1v/`，包括 `bad-*.log`、`missing-*.log`、`failure.log`；完整仿真结果见 4322 的 `navigation.log` 和 `nav-goal.json`。仿真中仍可见点云早于 TF 缓存的告警，上一次仿真也出现过该告警。
