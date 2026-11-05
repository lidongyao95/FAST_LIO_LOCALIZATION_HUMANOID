# 2026-11-05 第二台 G1 的 PC2 部署与构建排障

第二台 G1 的 PC2 已完成 ARM64/Foxy 三包构建、MID-360 静止采集、FAST-LIO 回放与 PCD 保存。安装 TF 启动缺陷修复后，同一静止包在 PC2 完成配准，4322 的 x86_64/Humble 行走仿真完成换起点重定位和 Nav2 目标测试。

- **部署与重编**见 §1–10，**实测结果**见 §11–14。
- **安装 TF 空裁剪的根因、修复和双端回归**见 §15。
- 现场安装方向、外参、动态对时、正式建图和独立重定位仍待验收。PC2 本轮验证使用静止采集和隔离回放，运动指令仅用于 4322 的仿真回归。

## 1. 环境与设备

| 项目 | 实测或证据 |
|---|---|
| 登录 | `ssh unitree@192.168.123.164`，已验证 SSH 公钥登录 |
| PC2 | 主机名 `ubuntu`，aarch64，Ubuntu 20.04.6 LTS，内核 `5.10.104-tegra`，8 核，15 GiB RAM，`/` 1.9 TiB 余 1.8 TiB |
| 网络 | `eth0` `192.168.123.164/24`，`wlan0` `172.18.17.116/20`；`ping 192.168.123.120` 20/20，平均 1.9 ms |
| MID-360 | `192.168.123.120`，MAC **`8c:58:23:c0:0c:b6`** |
| 时钟 | `timedatectl` 显示已同步；chrony 源为运控电脑 `192.168.123.161`，系统时间比 NTP 慢 0.5 ms |
| ROS | `/opt/ros/foxy` 与 `/opt/ros/noetic` 并存，本次只用 Foxy |
| 部署路径 | `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID`，数据目录 `~/ldy/g1_real_data/{bags,maps,records,config}` |
| 远程桌面 | `nxserver` 处于 active，`rviz2` 已安装，可按[推进方案 §2.1](2026-11-05_suspended_office_plan.md)使用 NoMachine |

**与 11-04 记录不是同一台整机。** 雷达 MAC 为 `8c:58:23:c0:0c:b6`，与[11-04 记录](2026-11-04_g1_pc2_slam.md)的 `8c:58:23:77:d1:b7` 不同；PC2 上也没有 `~/FAST_LIO_LOCALIZATION_HUMANOID` 与 `~/g1_real_data`。后续地图编号、录包编号和验证状态须与 11-04 那台分开登记。

## 2. 坑一：`eth0` 默认路由失效导致外网与 DNS 全断

`ip route` 中 `default via 192.168.123.1 dev eth0 metric 20100` 的 metric 小于 wlan0 的 20600，但 `192.168.123.1` 完全不通（20 包全丢），`getent hosts github.com` 与 `curl https://www.baidu.com` 均失败，`apt`、`pip`、`git` 不可用。这与 11-04 是同一个问题，换机后复现。

```bash
ip route | grep default          # 确认 eth0 默认路由的 metric 更小
ping -c 3 192.168.123.1          # 确认网关不可达
sudo ip route del default via 192.168.123.1 dev eth0
getent hosts github.com && curl -sI https://www.baidu.com | head -1
```

删除后默认路由回到 `wlan0`，`192.168.123.0/24` 仍经 `eth0` 直连。该删除只作用于当前会话，**重启后可能复发**；需要持久化时按[部署指南 §4](../g1_real_robot_slam_pc2_b300_guide.md)处理，不要在机器人网段保留默认网关。

## 3. 坑二：系统 Livox SDK 过旧，驱动编译报未知符号

PC2 `/usr/local` 下装的是 2023-11-06 的 `liblivox_lidar_sdk_shared.so`（4.8 MB），仓库内驱动源码需要更新的 SDK。直接构建 `livox_ros_driver2` 失败：

```
pub_handler.cpp:135: error: 'kLivoxLidarTypeMid360s' is not a member of 'LivoxLidarDeviceType'
pub_handler.cpp:329: error: 'kLivoxLidarDoubleEchoData' was not declared in this scope
pub_handler.cpp:463: error: 'LivoxLidarDoubleEchoRawPoint' was not declared in this scope
```

判断依据：`/usr/local/include/livox_lidar_def.h` 中没有 `MID360s`，而 `.deps/src/Livox-SDK2/include/livox_lidar_def.h` 中有 `kLivoxLidarDoubleEchoData = 0x11`。

解决：先编译仓库自带 SDK，再构建驱动。[build_g1_ros2.sh](../../scripts/build_g1_ros2.sh) 会优先取 `.deps/sdk/lib/liblivox_lidar_sdk_shared.so`，存在时不再回退到系统库。

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
cmake -S .deps/src/Livox-SDK2 -B .deps/sdk-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.deps/sdk"
cmake --build .deps/sdk-build --parallel 6
cmake --install .deps/sdk-build
```

安装后库为 1.2 MB，`fast_lio` 与驱动随即构建通过。**顺序不能颠倒**：先构建驱动会生成带旧 SDK 依赖的 `build/`，改 SDK 后必须 `rm -rf build/<pkg> install/<pkg>` 再重建，否则增量构建会沿用旧缓存。

## 4. 坑三：驱动包缺少生成的 `package.xml` 与 `launch/`

`livox_ros_driver2` 源码树只提供 `package_ROS1.xml`、`package_ROS2.xml` 和 `launch_ROS1/`、`launch_ROS2/`，`package.xml` 是生成物且被该子模块的 `.gitignore` 忽略。colcon 找不到 `package.xml` 就会跳过整个包。正常路径是运行 `scripts/setup_g1_local_deps.sh` 的第一步，手工等价命令：

```bash
cd .deps/src/livox_ros_driver2
cp -f package_ROS2.xml package.xml
mkdir -p launch && cp -f launch_ROS2/* launch/
```

这两项属于生成物，不在 Mac 仓库里回写。

## 5. 坑四：`MID360_config.json` 是占位 IP

驱动自带的配置是厂家示例，五项接收地址为 `192.168.1.5`、雷达地址为 `192.168.1.12`，直接启动收不到任何数据。本机需改为 PC2 的 `eth0` 地址和实际雷达地址：

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
for f in .deps/src/livox_ros_driver2/config/MID360_config.json \
         install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json; do
  sed -i 's/192\.168\.1\.5/192.168.123.164/g; s/192\.168\.1\.12/192.168.123.120/g' "$f"
done
```

`install/` 下那份是构建时拷贝的副本，**先改源码配置再构建**，否则要改两处。本次已将改好的源码配置回写到 Mac 仓库。

## 6. 坑五：`open3d_loc` 在 Foxy 上编译失败（tf2 头文件后缀）

Foxy 的 `tf2_ros`、`tf2_eigen`、`tf2_geometry_msgs` 只有 `.h` 头文件，`.hpp` 后缀从 Galactic/Humble 才开始提供。源码写的是 `.hpp`，报错：

```
open3d_conversions/RosConversions.h:11:10: fatal error: tf2_eigen/tf2_eigen.hpp: No such file or directory
```

另有一处 `#include <rclcpp/wait_for_message.hpp>`（第 2 行）同属 Galactic 之后才有的头，实际未被使用，直接删除。

三处 include 改为 `__has_include` 双发行版兼容，Foxy 与 Humble 都能编：

```cpp
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
```

改动文件：[global_localization.cpp](../../open3d_loc/src/global_localization.cpp)、[RosConversions.h](../../open3d_loc/include/open3d_conversions/RosConversions.h)、[DpgtConversions.h](../../open3d_loc/include/open3d_conversions/DpgtConversions.h)。用到的 `tf2::fromMsg`、`tf2::toMsg`、`tf2::eigenToTransform` 在两个发行版的 `.h` 头里都存在，无需改调用。

## 7. 坑六：用软链把 Open3D 塞进候选路径会算错 `_IMPORT_PREFIX`

`build_g1_ros2.sh` 只在固定候选路径下找 Open3D。为让脚本识别预编译包，曾把 `Open3DConfig.cmake` 所在目录软链到 `.deps/root/usr/lib/aarch64-linux-gnu/cmake/Open3D`，结果 CMake 以配置文件所在目录反推前缀，去 `.deps/root/usr/lib/lib/libOpen3D_3rdparty_glew.a` 找库：

```
The imported target "Open3D::3rdparty_glew" references the file .../.deps/root/usr/lib/lib/libOpen3D_3rdparty_glew.a but this file does not exist
```

**不要用软链伪装安装前缀。** 直接解包到真实目录，并用 `-DOpen3D_DIR` 指向它：

```bash
mkdir -p ~/ldy/vendor && cd ~/ldy/vendor && unzip -q ../open3d141_arm.zip
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH CONDA_PREFIX
export PATH=/usr/bin:/bin && source /opt/ros/foxy/setup.bash
colcon build --base-paths open3d_loc --packages-select open3d_loc --executor sequential \
  --cmake-args -DOpen3D_DIR="$HOME/ldy/vendor/open3d141/lib/cmake/Open3D" \
               -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
```

预编译包信息：`open3d141_arm.zip` 101 MB，SHA-256 `79fa97c9a2683c3efc5fc7e5025411578ca04e6772bb8635322dca34fe745d5e`，解包 576 MB，`libOpen3D.a` 实测 `Machine: AArch64`，来源为作者网盘（[README §1.3](../../README.md)），作者标注未充分测试。`unset` 那一行是必须的：家目录有 miniconda，遗留的 `CMAKE_PREFIX_PATH`/`CONDA_PREFIX` 会把 Python 和依赖解析到错误位置，构建脚本内部也是这么做的。

## 8. 坑七：Open3D 的 CMake 配置写死 `liblapacke.so`

配置里有一条绝对路径依赖，机器上没装 `liblapacke-dev` 时链接阶段直接停：

```
make[2]: *** No rule to make target '/usr/lib/aarch64-linux-gnu/liblapacke.so', needed by 'global_localization_node'. Stop.
```

先枚举全部绝对路径依赖，确认只缺这一个（5 条里缺 1 条）：

```bash
cd ~/ldy/vendor/open3d141/lib/cmake/Open3D
grep -ohE "/usr/lib/[a-z0-9_/-]+\.so(\.[0-9]+)?" Open3DTargets*.cmake Open3DConfig.cmake | sort -u |
  while read -r f; do [ -e "$f" ] || echo "MISSING $f"; done
sudo apt-get install -y liblapacke-dev     # 本次装的是 3.9.0-1build1
```

装完重建通过，产出 `install/open3d_loc/lib/open3d_loc/global_localization_node`，155 MB。`libOpen3D.a` 是静态库，二进制体积大属正常。

## 9. 远端构建时如何看进度

colcon 默认用 `console_cohesion`，每个包的输出攒到包结束才整块打印，看起来像卡死。两种做法结合使用：

1. 加 `--event-handlers console_direct+`，让编译器输出逐行透传；
2. 构建放后台写日志，每隔约 20–30 秒查进程、日志大小和终态。发现 `Finished`、`Failed` 或进程退出时，核对退出码与包状态，失败先定位错误。

```bash
colcon build ... --event-handlers console_direct+ > /tmp/build.log 2>&1 &

# 另一侧轮询，判断依据三条：进程是否还在、是否出现终态、日志是否还在增长
pgrep -f "colcon build" >/dev/null && echo RUNNING || echo DONE
grep -cE "^\[ *[0-9]+%\] Building" /tmp/build.log
tail -3 /tmp/build.log
```

本次 `open3d_loc` 首次构建终止于 1 min 14 s 的链接错误、`fast_lio` 耗时 2 min 2 s，都在构建结束后才看到结果，属于反例。后续按上面的方式在过程中复查。

## 10. 部署与回写规则

- 上传用 rsync，排除 `.git/`、`build/`、`install/`、`log/`、`.deps/sdk*`、`.deps/root/`、`doc/`、`tmp/` 与两个仿真子模块，仓库实际传输约 97 MB。`doc/` 与仿真子模块体积占大头，且与真机构建无关。
- `build/`、`install/`、`.deps/sdk*`、`.deps/root/`、`vendor/` 都是构建产物，**不双向同步**，换机或换架构必须重建。
- 源码级改动在 Mac 编辑后 rsync 到 PC2；在 PC2 上临时改的配置（如 `MID360_config.json`）用 `rsync -ain` 反向 diff 出清单再拉回，本次已回写该文件。

## 11. 结果与未验证项

| 包 | 产物 | 状态 |
|---|---|---|
| `livox_ros_driver2` | `install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node`，212 KB | 构建通过；已启动并录包，见第 12 节 |
| `fast_lio` | `install/fast_lio/lib/fast_lio/fastlio_mapping`，1.5 MB | 构建通过；已离线运行并保存 PCD，见第 13 节 |
| `open3d_loc` | `install/open3d_loc/lib/open3d_loc/global_localization_node`，155 MB | 构建通过；静止回放完成初始化和稳态配准，见 §15 |

`ros2 pkg prefix` 与 `ros2 interface show livox_ros_driver2/msg/CustomMsg` 在 `scripts/walking_env.sh` 环境下均正常。

PC2 已确认有 Nav2 的 planner、map server、lifecycle manager 和 NavFn，缺少 `nav2_bringup`。现场 A 段正式地图、B 段独立录包与持续定位、C 段规划和代价地图均待验收。4322 仿真结果单列在 §15。

`walking_env.sh` 默认使用 `ROS_DOMAIN_ID=72`，隔离回放需显式设置新的 ROS 域与 `ROS_LOCALHOST_ONLY=1`。

## 12. 静止基线验证结果

驱动已启动并录到第一份静止基线包，方案第 2 节的输入检查通过。

**运行登记。**

| 项 | 记录 |
|---|---|
| 机器 | 第二台 G1，雷达 MID-360 `192.168.123.120`，序列号 `2021370048`，MAC `8c:58:23:c0:0c:b6` |
| 架构与系统 | aarch64，Ubuntu 20.04.6，内核 `5.10.104-tegra`，8 核 / 15 GiB |
| 实测或回放 | 实测，实时采集 |
| ROS | Foxy，`ROS_DOMAIN_ID=72`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，`ROS_LOCALHOST_ONLY` 未设置 |
| 代码 | 主仓库 `9728ad3b856ab453e39bf555e18418a447672d39`；工作区差异为 `open3d_loc` 三个文件的 tf2 头文件兼容改动，以及 `.deps/src/livox_ros_driver2` 的配置与生成文件 |
| 驱动参数 | `xfer_format=1`、`publish_freq=10.0`、`multi_topic=0`、`data_src=0`、`frame_id=livox_frame` |
| 录包标识 | `~/ldy/g1_real_data/bags/static_01`，52.3 MiB / 13.408 s / 2817 条 |
| 时钟 | `timedatectl` 已同步，chrony 源 `192.168.123.161`，系统比 NTP 慢约 0.5 ms |
| 判定 | 静止输入检查通过；动态对齐与 FAST-LIO 未测 |
| 证据路径 | PC2 `/tmp/livox_driver.log`、`static_01` 包内数据、本文第 12 节 |
| 记录缺口 | 采集场地/工位、机体支撑或吊装状态、现场人员均未登记，下次采集前补齐 |

**启动。** `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 报 `file ... was not found in the share directory`，因为该 launch 装在 `share/livox_ros_driver2/launch_ROS2/` 子目录下。用绝对路径启动即可：

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID && source scripts/walking_env.sh
ros2 launch "$PWD/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
```

日志确认 `Livox Ros Driver2 Version: 1.2.8`，序列号 `2021370048`，`successfully set lidar attitude, ip: 192.168.123.120`，工作模式切到 `Normal`、IMU 使能，最后输出 `livox/lidar publish use livox custom format` 与 `livox/imu publish use imu format`。

**话题名与方案不一致。** 实际话题是 `/livox/lidar`（类型 `livox_ros_driver2/msg/CustomMsg`，`xfer_format=1`）和 `/livox/imu`，**不存在 `/livox/custom_msg`**：本仓库的 launch 文件没有做话题重映射。对应配置差异：[mid360.yaml](../../FAST_LIO/config/mid360.yaml) 的 `lid_topic` 是 `/livox/custom_msg`，[g1_mapping.yaml](../../FAST_LIO/config/g1_mapping.yaml) 是 `/livox/lidar`。录包与运行要么用 `/livox/lidar`，要么在 FAST-LIO 侧重映射，两者不要混用。

**录包。** `~/ldy/g1_real_data/bags/static_01`，命令为 `ros2 bag record -o ... /livox/lidar /livox/imu`，`ros2 bag info` 显示 52.3 MiB、13.408 s、2817 条。PC2 上没有 `rosbag2_py`（只有 `ros2bag` CLI），包内统计直接读 SQLite 的 CDR 字节得到：

| 观察项 | 实测 |
|---|---|
| 点云 | 134 帧，相邻头时间差 99.087 / 99.967 / 101.031 ms（最小/中位/最大），单调递增 |
| IMU | 2683 条，相邻头时间差 3.163 / 5.090 / 7.094 ms，单调递增，中位对应约 196.5 Hz |
| 帧坐标系 | 两路均为 `livox_frame` |
| 每帧点数 | `point_num` 19872–20160，`points` 数组长度与之相等 134/134，`lidar_id=192` |
| 时间基准 | `timebase` 与 `header.stamp` 差值 0.000 ms，134/134 一致 |
| 四元数模长 | 1.0000（作为字节布局与解析的自检） |
| 静止加速度 | 均值 x/y/z = `+0.0744 / -0.0003 / -0.9997`，模长均值 1.0025、标准差 0.0073、范围 0.9891–1.0189 |
| 静止角速度 | 均值 `+0.0291 / -0.0092 / +0.0160` rad/s，模长最大 0.0411 rad/s |

**单位判定。** 加速度模长约 1.00，可判定驱动把原始 g 值直接填入 `linear_acceleration`，不是 m/s²。FAST-LIO 可用：[IMU_Processing.hpp](../../FAST_LIO/src/IMU_Processing.hpp) 第 260 行按 `G_m_s2 / mean_acc.norm()` 重新标定，重力方向由归一化后的均值确定。重力沿 -Z，x 轴有约 0.074 g 的静止偏置，与安装姿态相关，需在方向核查中一并解释。

**零偏观察。** 静止角速度模长约 0.034 rad/s（约 2 °/s），与 11-04 记录的 IMU 噪声水平不可直接比较，但明显高于理想静止噪声。FAST-LIO 前段用静止窗口估零偏，若后续出现航向漂移，先复查这一项。

**退出行为。** 本次 `SIGINT` 后日志依次输出 `Livox Lidar SDK Deinit completely!`、`lddc destory`、`lds destory`、`process has finished cleanly`，没有复现 11-04 的 `-11` 段错误，也没有残留的 UDP 端口。样本仅一次，不能据此认为退出问题已消失。

**仍未覆盖。** 点云帧内逐点时间、点云与 IMU 的动态对齐需另用受控运动样本验证；本包的静止检查只支持静止结论。

## 13. FAST-LIO 与 open3d_loc 的离线运行验证

以下是安装 TF 修复前的首轮离线记录。回放使用第 12 节的静止包，运行在隔离域 `ROS_DOMAIN_ID=91`。修复后的结果见 §15。

### 13.1 FAST-LIO：离线建图与 PCD 保存

```bash
cd ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID && source scripts/walking_env.sh
export ROS_DOMAIN_ID=91
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$PWD/FAST_LIO/config/mid360.yaml" \
  -r /livox/custom_msg:=/livox/lidar \
  -p use_sim_time:=false \
  -p map_file_path:="$HOME/ldy/g1_real_data/maps/static_check_01.pcd" \
  -p debug_log_dir:="$HOME/ldy/g1_real_data/records"
ros2 bag play ~/ldy/g1_real_data/bags/static_01        # 另一终端，同域
```

| 观察项 | 实测 |
|---|---|
| 初始化 | 日志出现 `IMU Initial Done`，随后 `Initialize the map kdtree` |
| 里程计 | 125 条 `/Odometry_loc`，坐标系 `camera_init → body`，相邻头时间单调，跨度 12.400 s（约 10 Hz） |
| 配准点云 | 125 帧 `/cloud_registered_1`，坐标系 `camera_init`，每帧 892–954 点（`point_filter_num=3` 降采样后） |
| 静止稳定性 | 12.4 s 内位移上限 \|x\| 12.1 mm、\|y\| 6.0 mm、\|z\| 19.5 mm |
| 地图保存 | `/map_save` 返回 `success=True`，`static_check_01.pcd` 433 KB，`POINTS 13827`，`DATA binary` |
| 调试产物 | `~/ldy/g1_real_data/records/` 下生成 `pos_log.txt`、`mat_out.txt`、`mat_pre.txt`、`dbg.txt` |

回放启动瞬间出现过一次 `No point, skip this scan!`，属首帧无有效点，不影响后续。本结果只证明静止数据的初始化、里程计与 PCD 保存通路可用，不代表动态精度。

### 13.2 open3d_loc：运行时冒烟（ARM 预编译 Open3D）

首次直接运行即失败，暴露三个问题，都在正确用法下排除后才通过。

```
[RTPS_QOS_CHECK Error] INCORRECT TOPIC QOS (rt/baselink2map): depth must be <= max_samples
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  could not create publisher
```

**坑八：发布队列深度超过 FastRTPS 上限。** 源码给 5 个里程计话题用了 `create_publisher(..., 100000)`，超过 `rmw_fastrtps_cpp` 的 `max_samples`，创建 publisher 时直接抛 `RCLError` 退出。里程计只有 10–100 Hz，队列不需要这个量级。[global_localization.cpp](../../open3d_loc/src/global_localization.cpp) 中五处已改为 `100`，并在原处注明原因。

**坑九：`initialpose` 缺失时按下标取元素，直接段错误。** 用 `ros2 run` 直接启动时节点名曾是代码里的 `global_loc_node`，而 `config/loc_param_g1.yaml` 的键名是 `global_localization_node`，参数一条都不生效。`initialpose` 默认值又是空 `vector`，源码随后无条件取 `initialpose_[3]`、`[4]`、`[5]`，越界导致崩溃，记录中退出码为 245。显式命名为 `global_localization_node` 的 launch 可加载同名参数段。

**责任判定：缺校验属于代码缺陷，绕过 launch 是触发条件。** 任何参数没加载成功的路径（键名不符、文件路径错、直接用 `ros2 run`）都会走到越界，故障表现是段错误而不是报错。两处已修：

1. 节点默认名由 `global_loc_node` 改为 `global_localization_node`，`ros2 run` 与 launch 行为一致。launch 里的 `name='global_localization_node'` 保留，作为显式固定。
2. 构造函数在取下标前校验 `initialpose_.size() == 6`，不满足则打印 `RCLCPP_FATAL` 并抛异常；`main` 捕获后 `rclcpp::shutdown()` 并返回 1。

改后验证：

| 场景 | 结果 |
|---|---|
| 不给参数文件（原段错误场景） | 退出码 **1**，日志为 `[FATAL] initialpose 需要 6 个元素 ... 实际收到 0 个。请检查参数文件路径，以及参数文件里的节点名是否与 global_localization_node 一致。` |
| 只给参数文件，不再加 `-r __node:=...` | 参数文件匹配节点名。安装 TF 就绪后才能完成当前版本的启动 |

直接运行前，在同一 ROS 域发布并核对 `base_link ← imu_link` 与 `motion_link ← base_link` 两项安装 TF。下例关闭节点对 `motion_link` 的广播，仍需提供两项安装变换。诊断时的单位变换仅适用于复现本次静止回放。

```bash
ros2 run open3d_loc global_localization_node --ros-args \
  --params-file "$PWD/open3d_loc/config/loc_param_g1.yaml" \
  -p path_map:="$HOME/ldy/g1_real_data/maps/static_check_01.pcd" \
  -p publish_motion_tf:=false
```

**坑十：`kf_baselink2map` 参数读不到。** 即使节点名正确，日志仍是 `kf_x/kf_y/kf_z: [0.000000, 0.000000]`。yaml 用嵌套写法 `kf_baselink2map: {x: [...], y: [...], z: [...]}`，ROS 2 会把嵌套映射解析成点号键 `kf_baselink2map.x`，而源码 `declare_parameter` 用的是斜杠 `kf_baselink2map/x`，两者永远对不上，卡尔曼方差配置始终为 0。

修正方式是把 yaml 写成与源码完全一致的斜杠键，**不是**改成点号键——点号键同样匹配不上斜杠名。[loc_param_g1.yaml](../../open3d_loc/config/loc_param_g1.yaml) 已改为：

```yaml
kf_baselink2map/x: [0.001, 0.002]
kf_baselink2map/y: [0.001, 0.005]
kf_baselink2map/z: [0.00001, 0.04]
```

改后实测日志为 `kf_x: [0.001000, 0.002000]`、`kf_y: [0.001000, 0.005000]`、`kf_z: [0.000010, 0.040000]`，三项均已生效。Foxy 接受参数名里带斜杠，因此不需要动源码。

**首轮结果。** 节点成功加载 13,827 点的 PCD，常驻 RSS 约 38.7 MiB。接入 FAST-LIO 后反复出现 `Skipping localization initialization with empty crop: target=0 source=0`。后来用相同录包、PCD 和零初值做对照，定位到安装 TF 查询失败后继续启动，详见 §15。

### 13.3 同步与构建

发布队列和参数校验改动后，PC2 增量构建 `open3d_loc` 耗时 52.6 秒。源码与 `loc_param_g1.yaml` 均从 Mac 同步，两侧校验和一致。

## 14. 同类缺陷的补查与补充修复

按“输入未校验导致静默异常”这一类回扫全部代码后，又发现两处同类隐患并修复。扫描同时也确认若干位置本来就有保护：`kf_param_x/y/z` 取下标前有 `size() >= 2` 判断与错误分支；`map_save` 在 `pcd_save_en=false` 时返回 `success=false` 与 `Map save disabled.`；`meas.imu.empty()` 在调用 `IMU_init` 之前就返回；`path_map` 为空或读取失败会报错并关闭节点；`open3d_loc` 中除已修的 5 处外没有其他过大的发布队列。

### 14.1 坑十一：IMU 全零时除零，静默产生 NaN

[IMU_Processing.hpp](../../FAST_LIO/src/IMU_Processing.hpp) 有三处用 `mean_acc.norm()` 作除数：初始化时算重力方向、去畸变时把加速度折算到 `G_m_s2`、初始化完成时缩放 `cov_acc`。全零加速度（传感器故障、`imu_topic` 订到全零话题、驱动异常）会让 `norm()` 为 0，重力方向变 NaN，此后整个状态静默成 NaN，日志没有任何报错。

修改：

1. 初始化算重力之前判断 `acc_norm < 1e-6`，成立则打印 `[FAST-LIO] IMU acceleration norm is zero, skip gravity initialisation` 并放弃本次初始化，不再写入 NaN。
2. `init_iter_num > MAX_INI_COUNT` 分支里再判一次，成立时打印 `keep waiting for valid IMU data` 并提前返回，保持 `imu_need_init_` 为真，避免把全零数据判成“初始化完成”。
3. 去畸变处的除数改为局部值并兜底，避免异常输入把状态带成 NaN。

**验证方法**：复制 `static_01` 为 `static_zeroimu`，用 SQLite 把包内 2683 条 IMU 的 `linear_acceleration` 字节清零、保留时间戳与角速度，再回放同一份点云。

| 观察项 | 结果 |
|---|---|
| 校验触发 | 日志出现 266 次 `IMU acceleration norm is zero` |
| 是否误判完成 | `IMU Initial Done` 出现 **0** 次，节点持续等待有效数据 |
| 副作用 | 无 NaN 里程计输出、无崩溃 |
| 正常数据回归 | 原 `static_01` 回放仍输出 `IMU Initial Done`，`/map_save` 返回 `success=True`，PCD 520 KB |

复测完成后已删除合成的 `static_zeroimu` 包，方法与结论保留在本节。

### 14.2 坑十二：点云转换在未知第 4 字段时整帧丢点

[open3d_conversions.cpp](../../open3d_loc/src/open3d_conversions/open3d_conversions.cpp) 用 `fields.size() == 3` 作为“无颜色字段”的判据，否则读取 `fields[3]`，且只处理 `rgb` 与 `intensity` 两个分支。字段数少于 4 时该判据不成立；第 4 个字段是别的名字（如 `normal`、`ring`）时两个分支都不进，**整帧点一个都不 push**，函数返回空云且不报错。

修改：判据改为 `fields.size() < 4`；颜色分支补上 `else` 兜底，未知第 4 字段时退回无色点云。

**验证范围。** 修复安装 TF 后，FAST-LIO 的正常点云可以进入配准。未知第四字段的分支仍待用构造消息验证。XYZ 迭代器在分支前创建，输入仍需提供完整的 XYZ 字段。

### 14.3 改动与构建

| 文件 | 改动 |
|---|---|
| [IMU_Processing.hpp](../../FAST_LIO/src/IMU_Processing.hpp) | 三处零模长判断与兜底除数 |
| [open3d_conversions.cpp](../../open3d_loc/src/open3d_conversions/open3d_conversions.cpp) | 字段数判据改为 `< 4`，补未知字段兜底分支 |

两个文件在 Mac 编辑后同步到 PC2，`fast_lio` 与 `open3d_loc` 一起重建，`fast_lio` 1 min 46 s、`open3d_loc` 29.4 s，`Summary: 2 packages finished`。

## 15. 坑十三：启动时安装 TF 未就绪导致空裁剪

**复现与根因。** PC2 首次直接运行定位节点时，两项安装 TF 查询失败，节点仍打印 `initialize finished`，随后出现 `target=0 source=0`。复查时即使提前启动静态发布器，定位节点仍有一次在 DDS 发现完成前查询失败。构造函数忽略了查询结果，里程计回调随后对未成功赋值的安装变换求逆，导致裁剪位姿无效。两项 TF 均被查到时，使用同一 `static_01`、`static_check_01.pcd` 和零初值，得到 10,616 个地图裁剪点和 4,610 个扫描裁剪点，并完成初始化。

**修复。** `open3d_loc` 在启动定位线程前等待 `base_link ← imu_link` 和 `motion_link ← base_link`。两项查询共用 5 秒的单调时钟预算，因此仿真时钟尚未发布时也会超时。两项查询成功才打印 `initialize finished`。超时日志指出缺失的源、目标坐标系，节点以退出码 `1` 结束。 [定位节点源码](../../open3d_loc/src/global_localization.cpp#L432)

### 15.1 双端重编与启动用例

同一源码由 Mac 同步，两端 SHA-256 一致。仅增量重编 `open3d_loc`，PC2/Foxy 耗时 56.0 秒，4322/Humble 耗时 35.7 秒，均退出 0。启动用例在隔离 ROS 域使用 `use_sim_time=true`，无 `/clock` 发布者。等待时间从节点打印等待日志开始计量。

| 用例 | PC2 · aarch64 / Foxy | 4322 · x86_64 / Humble |
|---|---|---|
| 两项静态 TF 已发布 | 启动成功 | 启动成功 |
| 两项 TF 先后延迟到达 | 等待 1.77 秒后启动 | 等待 1.72 秒后启动 |
| 缺少 `base_link ← imu_link` | 5.03 秒后退出，码 1 | 5.17 秒后退出，码 1 |
| 仅缺少 `motion_link ← base_link` | 5.03 秒后退出，码 1 | 5.18 秒后退出，码 1 |

### 15.2 链路回归与日志

- **PC2 静止回放。** 启动定位节点后延迟发布两项诊断用单位 TF，再回放 `static_01`。两次初始化 fitness 为 0.9297、0.9298，随后进入稳态配准。日志目录为 `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID/tf-startup-check.FS46jh/`，关键文件为 `build.log`、`probe.log` 和 `loc.log`。
- **4322 行走仿真。** 从新起点 `(0.4, -0.2)` 启动，加载此前纯仿真建图生成的 PLY 和配套二维地图。重定位成功，Nav2 目标 `(1.2, -0.2)` 返回 `SUCCEEDED`。日志目录为 `~/ldy/FAST_LIO-humble-review.qR6RmX/tf-startup-check.Pbacv4/`，关键文件为 `build.log`、`probe.log`、`relocalization.log` 和 `nav-goal.json`。

PC2 的证据覆盖安装 TF 等待与静止配准。B 段正式重定位仍需实测安装外参、独立录包和现场参考位姿。
