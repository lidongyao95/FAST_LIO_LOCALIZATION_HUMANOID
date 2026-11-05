# 2026-11-05 第二台 G1 的 PC2 部署、排障与验证

## 1. 验证结论

| 项目 | 结果 | 证据范围 |
|---|---|---|
| PC2 构建 | `livox_ros_driver2`、`fast_lio`、`open3d_loc` 均通过 | aarch64 / Ubuntu 20.04.6 / ROS 2 Foxy |
| PC2 采集与建图 | MID-360 静止录包通过；FAST-LIO 回放保存 13,827 点 PCD | `static_01`，13.408 秒静止数据 |
| 定位启动修复 | 两端均通过 TF 已就绪、延迟到达和缺失用例 | PC2/Foxy 与 4322/Humble；启动用例均无 `/clock` |
| PC2 定位回归 | 延迟提供安装 TF 后，静止回放完成初始化和稳态配准 | 使用同一静止包、同源 PCD 和诊断用单位 TF |
| 4322 仿真回归 | 换起点重定位成功；Nav2 目标返回 `SUCCEEDED` | x86_64 / Ubuntu 22.04 / ROS 2 Humble，纯仿真 |

**现场仍待验收。** A 段需要实测安装方向和外参、验证动态对时并采集正式地图。B 段需要独立录包和参考位姿，验证不同起点的重定位与持续定位。C 段需要验证 PC2 上的规划和代价地图。PC2 已有 planner、map server、lifecycle manager 和 NavFn，缺 `nav2_bringup`。本轮运动指令仅用于 4322 仿真。 [办公室推进方案](2026-11-05_suspended_office_plan.md)

## 2. 定位空裁剪与 TF 修复

### 复现、根因和当前行为

首轮 PC2 回放能加载地图，但节点在两项安装 TF 查询失败后仍打印 `initialize finished`，随后反复报 `Skipping localization initialization with empty crop: target=0 source=0`。即使静态发布器先启动，定位节点也可能在 DDS 发现完成前查不到变换。旧构造函数忽略了查询失败，里程计回调随后对未成功赋值的安装矩阵求逆，裁剪位姿由此失效。

复查时保留 `static_01`、`static_check_01.pcd` 和零初值。两项 TF 都被查到后，裁剪得到 10,616 个地图点和 4,610 个扫描点，并完成配准初始化。

当前节点在启动定位线程前等待 `base_link ← imu_link` 与 `motion_link ← base_link`，两项共用 **5 秒单调时钟预算**。查询都成功后才打印 `initialize finished`。超时会指出缺失变换的源和目标坐标系，进程以退出码 `1` 结束。即使设置 `publish_motion_tf:=false`，定位计算仍需要两项安装变换。 [定位节点源码](../../open3d_loc/src/global_localization.cpp#L432)

复现静止配准时，先在同一隔离 ROS 域提供上述两项诊断 TF，再从仓库根目录运行定位节点。现场使用实测外参。配准是否完成看 `Localization initialization complete` 日志。

```bash
ros2 run open3d_loc global_localization_node --ros-args \
  --params-file "$PWD/open3d_loc/config/loc_param_g1.yaml" \
  -p path_map:="$HOME/ldy/g1_real_data/maps/static_check_01.pcd" \
  -p use_sim_time:=false -p publish_motion_tf:=false
```

### 双端回归

同一份源码从 Mac 同步到两端，SHA-256 一致。增量重编 `open3d_loc` 在 PC2 用时 56.0 秒，在 4322 用时 35.7 秒，均退出 0。下表的启动用例使用隔离 ROS 域和 `use_sim_time=true`，无 `/clock` 发布者。等待时间从节点打印等待日志起算。

| 启动用例 | PC2/Foxy | 4322/Humble |
|---|---|---|
| 两项 TF 已发布 | 启动成功 | 启动成功 |
| 两项 TF 先后延迟发布 | 1.77 秒后启动 | 1.72 秒后启动 |
| 缺少 `base_link ← imu_link` | 5.03 秒后退出，码 1 | 5.17 秒后退出，码 1 |
| 仅缺 `motion_link ← base_link` | 5.03 秒后退出，码 1 | 5.18 秒后退出，码 1 |

| 链路 | 实测结果 | 主机上的日志 |
|---|---|---|
| PC2 静止回放 | 定位节点启动后延迟发布两项诊断用单位 TF。回放 `static_01` 后，两次初始化 fitness 为 0.9297、0.9298，随后进入稳态配准。 | `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID/tf-startup-check.FS46jh/` 下的 `build.log`、`probe.log`、`loc.log` |
| 4322 行走仿真 | 从 `(0.4, -0.2)` 启动，加载同一次仿真建图导出的 PLY 和二维地图。重定位成功，目标 `(1.2, -0.2)` 返回 `SUCCEEDED`。 | `~/ldy/FAST_LIO-humble-review.qR6RmX/tf-startup-check.Pbacv4/` 下的 `build.log`、`probe.log`、`relocalization.log`、`nav-goal.json` |

## 3. PC2 部署与重编

### 主机与网络

| 项目 | 本轮环境 |
|---|---|
| 主机 | 第二台 G1 的 PC2，主机名 `ubuntu`，公钥登录 `ssh unitree@192.168.123.164` |
| 系统 | aarch64，Ubuntu 20.04.6，内核 `5.10.104-tegra`，8 核 / 15 GiB，根分区剩余约 1.8 TiB；ROS Foxy 与 Noetic 并存，本轮用 Foxy |
| 网络 | `eth0` 为 `192.168.123.164/24`，`wlan0` 为 `172.18.17.116/20`；雷达 `192.168.123.120`，ping 20/20，平均 1.9 ms |
| 雷达 | MID-360，序列号 `2021370048`，MAC `8c:58:23:c0:0c:b6`。与 11-04 第一台 G1 的地图和录包分开登记。 |
| 路径 | 仓库 `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID`；数据 `~/ldy/g1_real_data/{bags,maps,records,config}` |
| 远程观察 | NoMachine 服务已运行，`rviz2` 已安装。 [推进方案中的远程可视化步骤](2026-11-05_suspended_office_plan.md) |

曾出现 `apt`、`pip` 和 `git` 同时无法访问外网。实际路由优先走 `eth0`（metric 20100，低于 WLAN 的 20600），其网关 `192.168.123.1` 不可达。确认该情况后，临时删除该默认路由，外网恢复走 WLAN，机器人网段继续经 `eth0` 直连。重启后需复查。 [持久化配置见部署指南](../g1_real_robot_slam_pc2_b300_guide.md)

```bash
ip route | grep default
ping -c 3 192.168.123.1
sudo ip route del default via 192.168.123.1 dev eth0
getent hosts github.com
```

### Livox SDK、驱动和 FAST-LIO

| 症状 | 处理 |
|---|---|
| 编译找不到 `kLivoxLidarTypeMid360s`、`kLivoxLidarDoubleEchoData` 或 `LivoxLidarDoubleEchoRawPoint` | PC2 的系统 SDK 较旧。先编译仓库固定版本的 SDK，再让驱动使用 `.deps/sdk` 的库和头文件。切换 SDK 后检查旧 CMake 缓存，按受影响包重新配置。 |
| colcon 找不到驱动包 | 从 `package_ROS2.xml` 生成 `package.xml`，从 `launch_ROS2/` 准备 `launch/`。生成文件按机器重新生成。 |
| 驱动启动后无数据 | 示例 JSON 中接收地址为 `192.168.1.5`，雷达地址为 `192.168.1.12`。第二台 G1 将五项接收地址改为 `192.168.123.164`、雷达地址改为 `192.168.123.120`。重新构建后核对安装副本。 |

在仓库根目录准备构建依赖。PC2 的 IP 配置保存在驱动子模块的本机配置中，部署时按实际地址复核。

```bash
cmake -S .deps/src/Livox-SDK2 -B .deps/sdk-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.deps/sdk"
cmake --build .deps/sdk-build --parallel 6
cmake --install .deps/sdk-build

cp .deps/src/livox_ros_driver2/package_ROS2.xml .deps/src/livox_ros_driver2/package.xml
mkdir -p .deps/src/livox_ros_driver2/launch
cp .deps/src/livox_ros_driver2/launch_ROS2/* .deps/src/livox_ros_driver2/launch/
```

从干净 Foxy 环境构建。 [项目构建脚本](../../scripts/build_g1_ros2.sh)

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH CONDA_PREFIX
export PATH=/usr/bin:/bin
source /opt/ros/foxy/setup.bash
colcon build --base-paths .deps/src/livox_ros_driver2 FAST_LIO \
  --packages-select livox_ros_driver2 fast_lio --executor sequential \
  --event-handlers console_direct+ --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=foxy -DBUILD_TESTING=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DLIVOX_LIDAR_SDK_LIBRARY="$PWD/.deps/sdk/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$PWD/.deps/sdk/include" \
  > /tmp/pc2-livox-fastlio-build.log 2>&1 &
```

### Open3D 与 Foxy 兼容

| 症状 | 处理 |
|---|---|
| `tf2_eigen/tf2_eigen.hpp: No such file or directory` | Foxy 使用 `.h`。源码通过 `__has_include` 兼容两种后缀，并移除了未使用的 `rclcpp/wait_for_message.hpp`。 |
| CMake 从错误的 `_IMPORT_PREFIX` 寻找 `libOpen3D_3rdparty_glew.a` | Open3D 的导入配置依赖解包目录。把 `Open3D_DIR` 指向真实的 CMake 目录，保持整个解包目录结构。 |
| `No rule to make target '/usr/lib/aarch64-linux-gnu/liblapacke.so'` | 枚举 `Open3DTargets*.cmake` 和 `Open3DConfig.cmake` 中的绝对库路径。本轮唯一缺失项由 `liblapacke-dev` 提供，PC2 安装版本为 `3.9.0-1build1`。 |

本轮用作者提供的 ARM 预编译 `open3d141_arm.zip`（101 MB，解包约 576 MB）。静态库确认是 AArch64，归档 SHA-256 为 `79fa97c9a2683c3efc5fc7e5025411578ca04e6772bb8635322dca34fe745d5e`。约 155 MB 的定位可执行文件包含静态链接的 Open3D。 [预编译包来源](../../README.md)

在仓库根目录、清理过的 Foxy 环境中构建。以下路径对应本轮解包位置。

```bash
colcon build --base-paths open3d_loc --packages-select open3d_loc \
  --executor sequential --event-handlers console_direct+ --cmake-args \
  -DOpen3D_DIR="$HOME/ldy/vendor/open3d141/lib/cmake/Open3D" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF \
  > /tmp/pc2-open3d-build.log 2>&1 &
```

### 跟踪构建与回写

- 远程构建用 `console_direct+` 输出到后台日志，记录 `$!`。每约 20–30 秒检查进程、日志是否增长以及 `Finished` / `Failed`，终态出现后核对退出码。失败先查 `error:`、`CMake Error`、`No rule to make target` 或 `undefined reference`，再按受影响包重试。
- rsync 上传源码时排除 `.git/`、`build/`、`install/`、`log/`、`.deps/sdk*`、`.deps/root/`、`vendor/`、`doc/`、`tmp/` 和仿真子模块。换机或换架构时重新构建产物。
- PC2 临时修改的源码或配置先用 `rsync -ain` 反向查看差异，再回写所需文件。本轮已回写本机 MID-360 配置。生成的驱动清单、launch 副本和安装目录留在部署机。

## 4. 静止基线采集

### 驱动启动和录包

驱动版本为 1.2.8。本轮用 `xfer_format=1`、`publish_freq=10.0`、`multi_topic=0`、`data_src=0`、`frame_id=livox_frame`。点云发布到 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`），IMU 发布到 `/livox/imu`。仓库的驱动 launch 安装在 `launch_ROS2/` 子目录，按包名查找失败时使用绝对路径。

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
source scripts/walking_env.sh
ros2 launch "$PWD/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
```

在同环境的另一终端录包，每轮使用新编号。

```bash
ros2 bag record -o "$HOME/ldy/g1_real_data/bags/static_01" /livox/lidar /livox/imu
```

### 实测数据

采集时使用 Foxy、`ROS_DOMAIN_ID=72` 和 `rmw_fastrtps_cpp`，`ROS_LOCALHOST_ONLY` 未设置。主仓库基于 `9728ad3b856ab453e39bf555e18418a447672d39`，工作区包含 tf2 头文件兼容和 Livox 驱动配置及生成文件。采集工位、机体支撑状态与现场人员未登记。驱动日志为 PC2 的 `/tmp/livox_driver.log`，原包在 `~/ldy/g1_real_data/bags/static_01`。

| 观察项 | 实测 |
|---|---|
| 录包 | 52.3 MiB，13.408 秒，2,817 条消息 |
| 点云 | 134 帧；消息头单调递增，相邻间隔 99.087 / 99.967 / 101.031 ms（最小 / 中位 / 最大） |
| IMU | 2,683 条；消息头单调递增，相邻间隔 3.163 / 5.090 / 7.094 ms（最小 / 中位 / 最大） |
| 点云结构与时间 | 每帧 19,872–20,160 点，数组长度与 `point_num` 一致，`lidar_id=192`。两路 `frame_id` 均为 `livox_frame`，点云 `timebase` 与 `header.stamp` 一致。 |
| 逐点时间复查 | 134 帧均存在帧内时间局部逆序，133 帧的完整点时间范围由 IMU 时间覆盖。动态样本仍需核对帧尾取值和两路对齐。 |
| 静止加速度 | x/y/z 均值约 +0.0744 / −0.0003 / −0.9997 g；模长均值 1.0025 g、标准差 0.0073 g，范围 0.9891–1.0189 g |
| 静止角速度 | x/y/z 均值约 +0.0291 / −0.0092 / +0.0160 rad/s，模长最大 0.0411 rad/s |
| 系统时钟 | `timedatectl` 已同步，chrony 源为运控电脑 `192.168.123.161`，系统比 NTP 慢约 0.5 ms |

消息统计从 SQLite 录包的 CDR 数据读取。加速度以 g 为单位，FAST-LIO 在初始化后按 `G_m_s2 / mean_acc.norm()` 缩放。静止均值主要沿 −Z，X 分量应在现场安装方向核查中解释。该次 `SIGINT` 后驱动正常退出，UDP 端口释放。退出稳定性还需要更多样本。 [IMU 处理源码](../../FAST_LIO/src/IMU_Processing.hpp)

## 5. FAST-LIO 静止回放与保存

PC2 实装的 Foxy 播放器没有 `--clock`。本节使用原始消息头验证 FAST-LIO，参与终端需使用同一隔离 ROS 域。配置 `mid360.yaml` 订阅 `/livox/custom_msg`，因此将它重映射到录包的 `/livox/lidar`，并保留 `lidar_type: 1` 以订阅 `CustomMsg`。使用新的地图输出名和可写日志目录。 [MID-360 配置](../../FAST_LIO/config/mid360.yaml)

```bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
source scripts/walking_env.sh
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$PWD/FAST_LIO/config/mid360.yaml" \
  -r /livox/custom_msg:=/livox/lidar \
  -p use_sim_time:=false \
  -p map_file_path:="$HOME/ldy/g1_real_data/maps/static_replay_next.pcd" \
  -p debug_log_dir:="$HOME/ldy/g1_real_data/records"
```

在同域另一终端执行 `ros2 bag play ~/ldy/g1_real_data/bags/static_01`，随后保持 FAST-LIO 运行并调用 `ros2 service call /map_save std_srvs/srv/Trigger '{}'`。

| 检查项 | 首轮结果 |
|---|---|
| 初始化 | 依次出现 `IMU Initial Done` 和 `Initialize the map kdtree` |
| 里程计 | `/Odometry_loc` 125 条，`camera_init → body`，12.400 秒约 10 Hz；各轴静止位移绝对值上限为 12.1、6.0、19.5 mm |
| 配准点云 | `/cloud_registered_1` 125 帧，`camera_init` 下每帧 892–954 点 |
| 保存 | `/map_save` 返回 `success=True`；`static_check_01.pcd` 为 433 KB、13,827 点，`DATA binary` |

Open3D 首轮能加载该 PCD，RSS 约 38.7 MiB。定位初始化和稳态配准的复测见 [定位空裁剪与 TF 修复](#2-定位空裁剪与-tf-修复)。

## 6. 其他运行时缺陷与验证边界

| 缺陷 | 修复 | 验证 |
|---|---|---|
| FastRTPS 报 `depth must be <= max_samples`，创建发布器失败 | 五个里程计发布队列深度从 100,000 调为 100 | 定位节点可启动并创建发布器 |
| 直接运行时节点名与 YAML 段名不匹配，空 `initialpose` 被越界访问 | 默认节点名统一为 `global_localization_node`，并要求 `initialpose` 有六个元素 | 有效 YAML 可加载。无参数文件时明确报错并退出 1。 |
| `kf_x/y/z` 方差读为零 | ROS 将嵌套 YAML 展开成点号键，源码声明的是斜杠键。配置改为 `kf_baselink2map/x`、`/y`、`/z`。 | 日志读到 x `[0.001, 0.002]`、y `[0.001, 0.005]`、z `[0.00001, 0.04]` |
| 全零 IMU 加速度导致归一化除零 | 检查加速度均值模长，近零时保持等待初始化，去畸变处也防护除零 | 合成回放把原包 2,683 条 IMU 加速度清零，触发 266 次告警，无初始化完成或 NaN 里程计输出。原包回放可正常初始化与保存。零值后恢复和非有限输入仍待补测。 |
| 点云第四字段未知时整帧丢点 | 完整 XYZ 点云在第四字段未知时按无色点云处理 | 正常点云已通过配准回归。未知字段分支仍需构造消息验证。输入仍需完整 XYZ。 |

[定位节点](../../open3d_loc/src/global_localization.cpp) · [定位参数](../../open3d_loc/config/loc_param_g1.yaml) · [IMU 处理](../../FAST_LIO/src/IMU_Processing.hpp) · [点云转换](../../open3d_loc/src/open3d_conversions/open3d_conversions.cpp)
