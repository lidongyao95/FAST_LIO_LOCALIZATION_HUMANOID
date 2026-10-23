# G1 MuJoCo 定位导航配置说明

本文记录当前工作区的代码来源、依赖版本、配置分层、对上游代码的修改和重建方法。
运行步骤见 [`NAVIGATION_MANUAL.md`](NAVIGATION_MANUAL.md)，实测结果见
[`VALIDATION.md`](VALIDATION.md)。

## 1. 适用环境与边界

- 操作系统：Ubuntu 22.04 x86_64。
- ROS：ROS 2 Humble，Fast DDS，`rmw_fastrtps_cpp`。
- 机器人模型：官方 Unitree G1 29-DoF MuJoCo 模型。
- 当前入口：MuJoCo 仿真，包含建图、三维地图重定位和 Nav2 导航。
- 未验收范围：真实 G1、真实 MID-360 外参、ARM64 部署和多楼层场景。

本工作区中使用的仿真域默认为 `ROS_DOMAIN_ID=72`；支架传感器演示默认为
`ROS_DOMAIN_ID=71`。

## 2. 仓库与代码来源

### 2.1 主仓库

| 项目 | 来源 | 基线 |
|---|---|---|
| 当前主仓库 | `deepglint/FAST_LIO_LOCALIZATION_HUMANOID` | `humble` 分支，`df4772ec4797172430e7efe990711d09a529f4ad` |
| 目标 fork | `lidongyao95/FAST_LIO_LOCALIZATION_HUMANOID` | 推送工作区中的 `humble` 分支 |

当前工作区的 `origin` 仍指向 DeepGlint 上游。推送 fork 时应把 fork 添加为独立
remote，或将 `origin` 改为 fork 并另设 `upstream`；不要误认为现有 `origin` 是
个人 fork。

`FAST_LIO`、`open3d_loc` 和 `data` 都属于上述主仓库的历史和目录结构，不是本次
从其他仓库复制到工作区顶层的独立 Git checkout。`FAST_LIO/.gitmodules` 中记录了
`ikd-Tree`，但本基线的 `FAST_LIO/include/ikd-Tree` 是一组由主仓库直接跟踪的普通
文件，当前 `git submodule status` 为空，因此不要把它当作已初始化子模块处理。

FAST-LIO 组件自身源自 FAST-LIO 的 ROS 2 移植版；Open3D 定位组件源自主仓库中的
`open3d_loc`。本修改继续沿用这两个组件的目录和构建边界。

### 2.2 另行拉取的仓库

以下仓库位于被 `.gitignore` 排除的 `.deps/src/`。它们与主仓库独立克隆，没有在主
仓库中形成 submodule：

| 本地目录 | 官方来源 | 固定提交 |
|---|---|---|
| `.deps/src/unitree_mujoco` | `https://github.com/unitreerobotics/unitree_mujoco.git` | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` |
| `.deps/src/unitree_rl_mjlab` | `https://github.com/unitreerobotics/unitree_rl_mjlab.git` | `1425b15f73bd4095f0df53709d7c389c3eb9e790` |
| `.deps/src/unitree_sdk2` | `https://github.com/unitreerobotics/unitree_sdk2.git` | `d1abf001220d3e5f59e00a0c4aaa595e00089a4a` |
| `.deps/src/Livox-SDK2` | `https://github.com/Livox-SDK/Livox-SDK2.git` | `c0796f04c143143899c87a773d9f6b7136453c0b` |
| `.deps/src/livox_ros_driver2` | `https://github.com/Livox-SDK/livox_ros_driver2.git` | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` |

这些固定提交是官方仓库快照，不代表已经对应某个稳定 release tag。

`.deps` 当前还有以下本地状态，不属于主仓库功能修改：

- `unitree_rl_mjlab`、`unitree_sdk2` 和 `Livox-SDK2` 的 Git 工作树干净。
- `unitree_mujoco/COLCON_IGNORE` 是空文件，用于避免 colcon 扫描该目录。
- `livox_ros_driver2/launch/` 是 `launch_ROS2/` 的构建期副本，当前为未跟踪文件。
- 控制策略、ONNX、DDS 构建产物、Open3D 解包库和运行日志都在 `.deps` 或
  `sim/generated` 中，不进入主仓库。

### 2.3 系统与本地安装依赖

| 依赖 | 配置 |
|---|---|
| ROS 2 | `/opt/ros/humble` |
| MuJoCo Python | 3.5.0，位于 `.deps/venv` |
| NumPy | 1.26.4 |
| Open3D | Ubuntu Jammy `0.14.1+dfsg-7build3`，解包并链接到 `.deps/root` |
| Livox SDK2 | 源码固定提交，本地构建到 `.deps/sdk` |
| ONNX Runtime | `unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0` |

Open3D 没有通过 `sudo` 安装到系统。`open3d_loc` 的构建必须显式给出
`.deps/root` 中的 `Open3D_DIR`，不能依赖作者机器上的硬编码路径。

## 3. 配置分层

### 3.1 FAST-LIO 配置

| 文件 | 用途 | 建图输出 | TF |
|---|---|---|---|
| `FAST_LIO/config/g1_mujoco.yaml` | 支架固定的传感器/定位演示 | 关闭 | 保留 |
| `FAST_LIO/config/g1_mapping.yaml` | 自由行走建图 | 保存到 `sim/generated/mapping/map.pcd` | 关闭 |
| `FAST_LIO/config/g1_walking.yaml` | 行走定位与 Nav2 | 关闭 | 关闭 |

三个 G1 配置共同使用：

- LiDAR：`/livox/lidar`。
- IMU：`/livox/imu`。
- 模拟 Livox 数据格式：`lidar_type: 5`，`scan_line: 4`，10 Hz。
- 外参：LiDAR 在 IMU 上方 0.45 m，旋转为单位矩阵。
- `extrinsic_est_en: false`：仿真外参固定，不作在线估计。

该 0.45 m 和外参是仿真中显式定义的传感器安装关系，不是真机 MID-360 标定结果。

`g1_mapping.yaml` 和 `g1_walking.yaml` 设置 `publish.tf_en: false`，让
`g1_nav_tf.py` 成为平面导航 TF 的唯一来源。`g1_mujoco.yaml` 未关闭 FAST-LIO
TF，用于不启动 Nav2 的支架演示。

### 3.2 三维重定位配置

行走定位 launch 使用以下默认值，并允许通过 launch 参数覆盖：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `map_ply` | `sim/generated/room.ply` | 加载的 Open3D 点云地图 |
| `initial_z` | `0.793` | 骨盆/IMU 初始高度 |
| `voxelsize_coarse` | `0.2` m | 粗配准体素 |
| `voxelsize_fine` | `0.08` m | 精配准体素 |
| `threshold_fitness_init` | `0.5` | 初始配准接受阈值 |
| `threshold_fitness` | `0.5` | 连续定位接受阈值 |
| `confidence_loc_th` | `0.6` | 导航门禁最低置信度 |
| `publish_motion_tf` | `false` | 避免重复发布运动 TF |

`open3d_loc` 的点云话题已在行走定位 launch 中重映射：

- `/map` -> `/localization/map_cloud`
- `/scan` -> `/localization/scan_cloud`

这样三维点云不会与 Nav2 使用的栅格 `/map` 同名。

### 3.3 TF 与里程计

行走链路使用以下约束：

1. FAST-LIO 发布 `/Odometry`，但不在配置文件控制的行走阶段发布冲突 TF。
2. `open3d_loc` 输出地图修正后的 `/Odometry_loc`。
3. `scripts/g1_nav_tf.py` 将 `/Odometry_loc` 投影为平面 `/odom` 和
   `odom -> base_footprint -> base_link`。
4. Nav2 使用 `map`、`odom`、`base_footprint`；`use_sim_time` 为 `true`。
5. Open3D 地图、二维栅格和 `/Odometry_loc` 必须处于同一地图坐标约定下。

`global_localization` 的 `publish_motion_tf` 默认仍为 `true`，以保证上游其他 launch
行为不变；G1 行走定位入口显式设为 `false`。

### 3.4 Nav2 配置

`navigation/g1_nav2_params.yaml` 以本机
`/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` 为基线，由
`scripts/generate_nav_params.py` 生成后保存到仓库。主要适配为：

- 移除 AMCL，三维定位由 `open3d_loc` 提供。
- 机器人轮廓改为约 `0.68 m x 0.56 m` 的矩形。
- 全局和局部代价地图使用 `/livox/lidar` 的 `PointCloud2` 障碍层。
- 局部窗口为 `5 m x 5 m`，全局与局部地图分辨率均为 `0.05 m`。
- DWB 最大线速度 0.2 m/s，最大角速度 0.3 rad/s。
- Nav2 控制器和恢复行为输出 `/cmd_vel_nav`；导航门禁统一转发为控制器命令。
- `g1_navigation.launch.py` 的 `map_yaml` 参数覆盖文件中的机器绝对路径，实际
  应传入本次运行要加载的二维地图。

如果重新生成参数，生成器会写入生成时本机的仓库绝对路径；运行时应以
`map_yaml:=...` 覆盖，避免仓库迁移后加载旧路径。

### 3.5 控制器配置

`scripts/build_g1_controller.py` 只接受以下两个固定提交：

- `unitree_sdk2`: `d1abf001220d3e5f59e00a0c4aaa595e00089a4a`
- `unitree_rl_mjlab`: `1425b15f73bd4095f0df53709d7c389c3eb9e790`

构建时从 `unitree_rl_mjlab/deploy/robots/g1/config` 复制官方策略配置，保留
`Passive`、`FixStand`、`Velocity` 三个状态。策略 ONNX、观测定义和状态机来自
官方仓库；本仓库的 `controller/main.cpp` 是仿真适配层，增加键盘、ROS 速度、
导航独占控制和状态文件。`controller/dds_bridge.cpp` 把 MuJoCo 状态送入官方
`rt/lowstate` / `rt/lowcmd` DDS 链路。

生成的控制器和策略位于 `.deps/g1-controller`，二进制位于
`.deps/controller-build`，均不提交。

## 4. 对原始项目的修改

### 4.1 FAST-LIO

| 文件 | 修改 |
|---|---|
| `FAST_LIO/launch/mapping.launch.py` | 新增 `debug_log_dir` launch 参数，避免运行日志写回源码目录 |
| `FAST_LIO/src/laserMapping.cpp` | 新增 `publish.tf_en`；修正发布 Path 的 header 时间；调试矩阵和位置日志可写入指定目录 |
| `FAST_LIO/rviz/fastlio.rviz` | 适配 G1 话题、点云显示和初始视角；隐藏重复的全局 TF/里程计显示 |
| `FAST_LIO/config/g1_*.yaml` | 新增 G1 仿真 LiDAR/IMU 外参、参数和建图/定位 profile |

`publish.tf_en` 默认为 `true`，因此未显式配置的上游使用方式仍保持原行为。

### 4.2 Open3D 定位

| 文件 | 修改 |
|---|---|
| `open3d_loc/CMakeLists.txt` | 删除作者机器的硬编码 `Open3D_DIR`，允许 CMake 参数传入 |
| `open3d_loc/src/global_localization.cpp` | `/map` 改为 transient-local；新增 `publish_motion_tf`；空点云跳过 ICP；析构时 join 定位线程 |
| `open3d_loc/launch/g1_*.launch.py` | 新增支架演示、行走建图、行走定位和 Nav2 入口 |

空点云保护用于避开 FAST-LIO 首帧尚未准备好时 Open3D ICP 对空输入抛异常。它不是
定位漂移修复，也不能替代正确的初始位姿、时间同步和地图匹配。

### 4.3 新增仿真、控制和工具

- `sim/README.md`：支架传感器演示的范围和构建说明。
- `scripts/g1_mujoco_sensors.py`：生成官方 G1 场景、房间几何、IMU 和 10 Hz
  射线 LiDAR；真值话题只用于评估，不进入 LIO。
- `scripts/g1_walking_sim.py`：自由行走仿真，作为唯一 MuJoCo 所有者，接管
  官方 DDS 低层接口，并发布仿真时钟、关节、释放状态和动态障碍服务。
- `scripts/g1_command_bridge.py`：ROS 速度与控制器之间的事件式本地 IPC，
  统一施加速度限幅和导航独占门禁。
- `scripts/g1_nav_tf.py`：将三维定位里程计转换为 Nav2 平面里程计和 TF。
- `scripts/g1_operator.py`：按状态和新鲜度执行站稳、释放支架、启用导航和停止，
  不使用固定等待时间作为成功依据。
- `scripts/start*.sh`：分阶段启动和重复进程锁。
- `scripts/export_g1_map.py`：从 FAST-LIO PCD 生成 PLY、PGM、YAML 和元数据。
- `scripts/check_g1_localization.py`、`validate_walking.py`、
  `validate_nav_goal.py`：时间对齐、行走和导航验证工具。

## 5. 依赖重建

在一个空的工作区中准备本地依赖：

```bash
mkdir -p .deps/src

git clone https://github.com/unitreerobotics/unitree_mujoco.git \
  .deps/src/unitree_mujoco
git -C .deps/src/unitree_mujoco checkout \
  1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d

git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git \
  .deps/src/unitree_rl_mjlab
git -C .deps/src/unitree_rl_mjlab checkout \
  1425b15f73bd4095f0df53709d7c389c3eb9e790

git clone https://github.com/unitreerobotics/unitree_sdk2.git \
  .deps/src/unitree_sdk2
git -C .deps/src/unitree_sdk2 checkout \
  d1abf001220d3e5f59e00a0c4aaa595e00089a4a

git clone https://github.com/Livox-SDK/Livox-SDK2.git \
  .deps/src/Livox-SDK2
git -C .deps/src/Livox-SDK2 checkout \
  c0796f04c143143899c87a773d9f6b7136453c0b

git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  .deps/src/livox_ros_driver2
git -C .deps/src/livox_ros_driver2 checkout \
  21445540f0d100dc86a7e6df312dd70bbdb4afdf
```

MuJoCo/Python 和控制构建还需按 `sim/README.md` 的说明准备
`.deps/venv`、`.deps/root` 和 `.deps/sdk`。随后先构建控制器：

```bash
/usr/bin/python3 scripts/build_g1_controller.py
```

再构建主要 ROS 包：

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL=3

colcon build --base-paths .deps/src/livox_ros_driver2 FAST_LIO open3d_loc \
  --packages-select livox_ros_driver2 fast_lio open3d_loc --executor sequential \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_LIBRARY="$PWD/.deps/sdk/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$PWD/.deps/sdk/include" \
  -DOpen3D_DIR="$PWD/.deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DBUILD_TESTING=OFF
```

## 6. 运行配置速查

所有交互终端先执行：

```bash
cd /path/to/FAST_LIO_LOCALIZATION_HUMANOID
source scripts/walking_env.sh
```

`walking_env.sh` 会清理外部 workspace overlay，只加载 `/opt/ros/humble` 和本项目
`install`，并设置 Fast DDS、本地动态库及日志目录。需要改变 domain 时应在 source
前设置：

```bash
export ROS_DOMAIN_ID=72
```

典型分阶段启动顺序：

```bash
./scripts/start_walking_sim.sh
./scripts/start_g1_controller.sh
./scripts/g1_operator.sh prepare
./scripts/start_g1_localization.sh \
  map_ply:=$PWD/sim/generated/mapping/map.ply \
  initial_x:=1.0 initial_y:=-1.0 initial_yaw:=0.3
./scripts/g1_operator.sh status
./scripts/start_g1_navigation.sh \
  map_yaml:=$PWD/sim/generated/mapping/map.yaml
./scripts/g1_operator.sh enable-nav
```

建图入口必须在仿真和控制器运行后单独启动：

```bash
./scripts/start_g1_mapping.sh
```

停止导航和机器人：

```bash
./scripts/g1_operator.sh stop
```

## 7. 生成物与忽略规则

根目录 `.gitignore` 排除以下内容：

```text
/build/
/install/
/log/
/.deps/
/sim/generated/
__pycache__/
```

因此 Git 提交不包含第三方源码、编译产物、策略二进制、运行日志、点云地图或验证
输出。需要从干净 clone 恢复运行环境时，必须按第 5 节重新准备依赖和生成地图。

## 8. 验证范围

当前记录验证的是本机 MuJoCo 仿真：

- 三维建图与地图保存。
- 使用生成地图和已知初始位姿的 Open3D 重定位。
- Nav2 目标导航、动态障碍和取消。
- 键盘速度控制、导航独占门禁和分阶段退出。

尚未证明真实 G1 传感器时间、外参、底盘高度、网络 DDS 和 ARM64 环境可用。使用
真机前必须重新采集标定和测试数据，不能直接把仿真坐标参数当作真机配置。
