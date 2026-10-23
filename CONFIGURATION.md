# G1 MuJoCo 定位导航配置说明

本文记录当前工作区的代码来源、依赖版本、配置分层、对上游代码的修改和重建方法。
运行步骤见 [`NAVIGATION_MANUAL.md`](NAVIGATION_MANUAL.md)，实测结果见
[`VALIDATION.md`](VALIDATION.md)。

## 1. 本地环境与适用边界

### 1.1 本地环境准备

首次运行前，需要准备以下本地环境：

| 项目 | 当前配置 | 作用 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 x86_64 | 本项目脚本和依赖针对的平台 |
| ROS | ROS 2 Humble | 提供节点、话题、服务和构建系统 |
| 通信中间件 | eProsima Fast DDS | 在节点之间传输 ROS 消息并执行节点发现 |
| RMW 适配层 | `rmw_fastrtps_cpp` | 把 ROS 2 的 RMW 接口连接到 Fast DDS |

除系统 ROS 外，还需要在主仓库中准备 `.deps` 目录、Python 虚拟环境、Open3D、
Livox SDK、Unitree MuJoCo/RL/SDK 源码和控制策略。具体来源、固定提交和构建命令见
第 2、3 节；这些依赖不能只靠拉取主仓库源码获得。

有图形界面时默认使用 NVIDIA PRIME 渲染；没有 `DISPLAY` 时脚本使用 EGL。有
`DISPLAY` 但需要排除 NVIDIA PRIME 时，可设置 `G1_MUJOCO_GPU=0`。

### 1.2 ROS 2 Humble、Fast DDS 与 RMW 的关系

这三项不是三个并列的运行环境：

1. **ROS 2 Humble** 是 ROS 发行版，限定 API、构建方式和官方二进制包版本。
2. **Fast DDS** 是 ROS 2 底层使用的 DDS 通信实现，负责节点发现、消息序列化和
   网络传输。
3. **`rmw_fastrtps_cpp`** 是 ROS 2 中间件抽象层 RMW 对 Fast DDS 的 C++ 适配器。
   `fastrtps` 是该适配器沿用的历史名称；底层项目现在通常称为 eProsima Fast DDS。

`scripts/walking_env.sh` 中设置了：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

因此 `RMW_IMPLEMENTATION` 是选择 RMW 实现的环境变量，而
`rmw_fastrtps_cpp` 是它的值。显式设置该变量可以避免不同终端因已有 overlay 或
用户配置选择了不同 RMW 实现，进而出现同一 domain 内节点互相不可见的问题。

### 1.3 安装与验证

如果 ROS 2 Humble 尚未安装，应先按
[ROS 2 Humble Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
配置 `packages.ros.org` 软件源。只补充本工作区要求的 RMW 包时执行：

```bash
sudo apt update
sudo apt install ros-humble-rmw-fastrtps-cpp
```

`ros-humble-rmw-fastrtps-cpp` 会通过 apt 依赖自动安装兼容版本的
`ros-humble-fastrtps`、`ros-humble-fastcdr` 和其他 ROS 依赖，通常不需要单独手工
安装底层 DDS 库。

检查软件包和 RMW 解析结果：

```bash
source /opt/ros/humble/setup.bash
dpkg-query -W -f='${Package} ${Version}\n' \
  ros-humble-rmw-fastrtps-cpp ros-humble-fastrtps ros-humble-fastcdr
ros2 pkg prefix rmw_fastrtps_cpp
```

本项目已验证的二进制包版本为：

| apt 软件包 | 已验证版本 |
|---|---|
| `ros-humble-rmw-fastrtps-cpp` | `6.2.10-1jammy.20260907.210327` |
| `ros-humble-fastrtps` | `2.6.12-1jammy.20260723.233918` |
| `ros-humble-fastcdr` | `1.0.29-1jammy.20260226.010952` |

这些版本号不能混用其他 ROS 发行版。安装和升级时优先使用 ROS 2 Humble 对应的
apt 软件源，不要在 Humble 工作区中混入 Jazzy、Rolling 或手工编译的 Fast DDS
库。手工安装不同版本的 `libfastdds`、`libfastcdr` 或 RMW 适配器可能造成 ABI、
消息类型支持和动态库冲突；如果遇到加载或发现异常，先检查系统 ROS 和 RMW 动态库：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
printf 'ROS_DISTRO=%s\nRMW_IMPLEMENTATION=%s\n' "$ROS_DISTRO" "$RMW_IMPLEMENTATION"
ldd "$(ros2 pkg prefix rmw_fastrtps_cpp)/lib/librmw_fastrtps_cpp.so" | \
  grep -E 'fastrtps|fastcdr' || true
```

### 1.4 ROS domain

`ROS_DOMAIN_ID` 是 DDS 的逻辑通信分区，不是操作系统或 ROS 发行版。不同 domain
中的节点即使连接同一网络，也不会互相发现。本工作区中：

- 自由行走、定位和 Nav2 默认使用 `ROS_DOMAIN_ID=72`。
- 支架传感器演示默认使用 `ROS_DOMAIN_ID=71`。
- 参与同一仿真的所有终端必须使用相同 domain。

同一套仿真终端应统一使用 Fast DDS。不要只把部分终端切换为 CycloneDDS 或其他
RMW 实现，否则即使 domain 相同，节点也可能无法互相发现。运行时如何设置 domain
并加载工作区见第 6 节。

### 1.5 适用边界

本项目当前验证的是 Ubuntu 22.04 x86_64 上的单机、单层室内 MuJoCo 仿真链路：

- 官方 G1 29-DoF 模型在合成房间中的传感器仿真。
- 使用 FAST-LIO 建图、保存点云并重新加载地图。
- 在已知地图和合理初始位姿下执行 Open3D 三维重定位。
- Nav2 在二维栅格地图上的目标导航、动态障碍和任务取消。
- 同一主机和同一 ROS domain 内的 Fast DDS 通信。

以下内容没有经过本项目验收，不能根据仿真结果推定可用：

- 真实 G1 上的运控、通信、急停和安全策略。
- 真实 MID-360 的安装外参、时间同步和雷达噪声。
- ARM64 平台、跨主机组网、不同 RMW 实现之间的通信。
- 当前 RTX 4090 和 PRIME 环境之外的其他 GPU、驱动及 EGL 组合。
- 多楼层、跨楼层地图切换、室外大场景和长期运行漂移。
- 未经标定的初始位姿、与地图不匹配的传感器数据或损坏地图。

真机部署前必须重新采集传感器标定和运行数据，并单独验证定位、导航和安全边界。

## 2. 仓库与代码来源

### 2.1 主仓库

| 项目 | 来源 | 基线 |
|---|---|---|
| 原始项目 | `deepglint/FAST_LIO_LOCALIZATION_HUMANOID` | `humble` 分支，`df4772ec4797172430e7efe990711d09a529f4ad` |

原始仓库对本项目的主要价值是提供了可继续开发的 G1 定位骨架，而不只是代码出处：

- **定位算法基线**：保存了 FAST-LIO 与 `open3d_loc` 的完整源代码、构建依赖和
  点云地图配准流程，使本项目能够保留 LiDAR-Inertial 里程计和三维地图重定位的
  核心实现，而不是重新实现定位算法。
- **G1 与 ROS 2 Humble 集成起点**：原始 `humble` 分支已经提供 G1 场景下的
  LiDAR/IMU 话题约定、外参示例、定位 launch 和 Open3D 构建入口，为仿真侧复用
  相同的数据接口和坐标关系提供了依据。
- **地图和坐标关系参考**：`data/map.ply`、原始定位参数和 TF 处理说明了上游使用
  的点云地图格式与里程计接口约定。本项目的三维定位仍沿用这些基础概念。
- **兼容性回归基线**：原始 launch、真实设备配置和默认算法行为仍保留在仓库中。
  本项目新增的 `g1_*` 配置和 launch 作为并行入口存在，修改时仍可对照原入口检查
  是否意外改变了真实设备或既有使用方式。

原始仓库本身不是完整的 G1 自主导航应用：它没有本项目的 MuJoCo 自由行走控制、
分阶段启动、建图保存、Nav2 代价地图与目标导航链路。因此本项目是在其定位核心和
G1 集成基础上补齐仿真、控制、导航和验证流程，不能把新增的完整闭环归功于原始
仓库。

`FAST_LIO`、`open3d_loc` 和 `data` 都属于上述主仓库的历史和目录结构，不是本次
从其他仓库复制到工作区顶层的独立 Git checkout。`FAST_LIO/.gitmodules` 中记录了
`ikd-Tree`，但本基线的 `FAST_LIO/include/ikd-Tree` 是一组由主仓库直接跟踪的普通
文件，当前 `git submodule status` 为空，因此不要把它当作已初始化子模块处理。

FAST-LIO 组件自身源自 FAST-LIO 的 ROS 2 移植版；Open3D 定位组件源自主仓库中的
`open3d_loc`。本修改继续沿用这两个组件的目录和构建边界。

### 2.2 第三方源码 submodule

以下仓库作为主仓库的 Git submodule 固定在 `.deps/src/`。主仓库记录每个 submodule
的官方 URL 和精确提交，不会把约 1 GB 第三方源码写入主仓库历史：

| Submodule 路径 | 官方来源 | 固定提交 |
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
- 这些子模块中的本地未跟踪文件不会改变主仓库记录的提交，但会使子模块工作树
  显示为 dirty；主仓库只应更新 submodule 指针，不应把第三方仓库的本地文件直接
  加入主仓库。
- 控制策略、ONNX、DDS 构建产物、Open3D 解包库和运行日志都在 `.deps` 或
  `sim/generated` 中，不进入主仓库。

## 3. 初始化、依赖与构建

### 3.1 一键初始化

首次获取主仓库并准备所有项目本地依赖：

```bash
git clone REPOSITORY_URL FAST_LIO_LOCALIZATION_HUMANOID
cd FAST_LIO_LOCALIZATION_HUMANOID
./scripts/setup_g1_local_deps.sh
```

将 `REPOSITORY_URL` 替换为实际使用的仓库 clone 地址。已经 clone 主仓库时只需在
仓库根目录执行 `setup_g1_local_deps.sh`。

该脚本依次完成：

1. 拉取并检出五个固定版本的源码 submodule。
2. 为 Livox ROS 2 驱动生成 `package.xml` 和 launch 注册目录。
3. 准备 Python 3.10 虚拟环境及固定版本的 MuJoCo、NumPy。
4. 解包 Open3D C++ 库并检查动态依赖。
5. 编译并安装 Livox SDK2。

脚本可重复执行，已经完成的步骤会复用。它不安装 ROS 2 Humble 或
`rmw_fastrtps_cpp`，这些系统前置条件见第 1.3 节。脚本执行前需具备
`apt-get`、Git、CMake、C++ 编译器、`dpkg-deb` 和 `ldd`。

初始化后可检查 submodule 的实际提交：

```bash
git submodule status --recursive
```

正常首次构建的完整主路径是：

```text
3.1 setup_g1_local_deps.sh
  -> 3.3 build_g1_controller.py
  -> 3.4 build_g1_ros2.sh
  -> 第 6.1 节 source walking_env.sh
  -> 第 6.2 节启动
```

`git submodule status` 只用于额外核查；`setup_g1_local_deps.sh` 已经执行并打印该
结果，正常流程无需重复执行。

### 3.2 本地依赖组成与手工配置参考

如果已经成功执行第 3.1 节的 `setup_g1_local_deps.sh`，本节 3.2.1 至 3.2.3 的
所有操作都可以省略；脚本已经完成这些步骤并执行了相应检查。

本节用于说明 submodule 之外仍需要在 `.deps` 中准备的本地依赖，以及脚本内部
做了什么。只有需要理解配置原理、检查现有环境或排查脚本失败时，才需要查看和执行
下面的手工步骤；正常构建应直接从第 3.3 节继续。

下表不是 submodule 清单，也不重复第 1.1、1.3 节中的 ROS 系统环境：

| 本地依赖 | 版本或位置 |
|---|---|
| MuJoCo Python | 3.5.0，位于 `.deps/venv` |
| NumPy | 1.26.4 |
| Open3D C++ 库 | Ubuntu Jammy `0.14.1+dfsg-7build3`，解包并链接到 `.deps/root` |
| Livox SDK2 构建产物 | 使用 submodule 源码构建，输出到 `.deps/sdk` |

这些依赖之所以不能由 submodule 本身提供，是因为 submodule 只固定源码提交：

- Python 包必须装入与 ROS Humble Python 3.10 匹配的虚拟环境。
- Open3D C++ 库需要在当前 Ubuntu 版本上解包，并提供 CMake 配置、头文件和共享库。
- Livox SDK 必须针对当前 CPU 架构和构建参数生成动态库。

Unitree MuJoCo 模型、Unitree RL/SDK 源码、Livox 驱动源码和 ONNX Runtime 都由已
初始化到固定提交的 submodule 提供，不在上表中重复列出。ROS 2 Humble 和
`rmw_fastrtps_cpp` 的安装与验证见第 1.3 节。

#### 3.2.1 MuJoCo Python 与 NumPy

使用系统 Python 3.10 创建独立虚拟环境。不要使用可能存在的 Anaconda Python
3.13 环境，因为 ROS 2 Humble 的 Python 模块按 3.10 构建：

```bash
sudo apt update
sudo apt install python3-venv python3-pip

/usr/bin/python3 -m venv .deps/venv
.deps/venv/bin/python -m pip install --upgrade pip
.deps/venv/bin/python -m pip install \
  'mujoco==3.5.0' 'numpy==1.26.4'
```

如果系统没有 `python3-venv`，`setup_g1_local_deps.sh` 也支持使用 `uv`
创建同一 Python 3.10 环境并安装上述固定版本；手工配置仍推荐安装
`python3-venv`。

运行时会由 `scripts/walking_env.sh` 加载 ROS 的 Python 模块，不依赖虚拟环境中的
system site-packages。依赖配置阶段还没有本地 `install` overlay，因此验证时直接
加载系统 ROS：

```bash
source /opt/ros/humble/setup.bash
.deps/venv/bin/python - <<'PY'
import mujoco, numpy, rclpy
print(mujoco.__version__, numpy.__version__, rclpy.__file__)
PY
```

预期 MuJoCo 为 `3.5.0`、NumPy 为 `1.26.4`，并且 `rclpy` 来自
`/opt/ros/humble`。

#### 3.2.2 Open3D C++ 库

原始仓库包含的是 `open3d_loc` 项目源码，不包含 Open3D C++ 库本身。
`open3d_loc/CMakeLists.txt` 使用 `find_package(Open3D REQUIRED)`，其源码通过
`<open3d/Open3D.h>` 调用点云读取、降采样、法向估计、ICP 和 FPFH 配准接口，所以
构建 `open3d_loc` 时仍必须提供兼容版本的 Open3D C++ 库。

原始 README 让用户解压作者提供的 `open3d141.zip` 并修改 CMake 中的
`Open3D_DIR`。本项目改用 Ubuntu Jammy 的二进制包，将其解包到项目本地目录，避免
依赖网盘、作者机器路径或系统级安装。

本项目使用 Ubuntu Jammy 提供的 Open3D 二进制，不解包到系统目录，也不执行
`sudo apt install libopen3d-dev`。先准备一个空的 `.deps/debs` 目录，然后下载并
解包 Open3D 及其运行库：

```bash
mkdir -p .deps/debs .deps/root
cd .deps/debs

apt download \
  libopen3d-dev libopen3d0.14 libfilament1.9 \
  liblapacke liblzf1 libstb0 libtinyobjloader1 libtmglib3

for package in ./*.deb; do
  dpkg-deb -x "$package" ../root
done

cd ../..
```

构建 `open3d_loc` 时通过以下 CMake 参数指定本地解包目录：

```text
-DOpen3D_DIR=$PWD/.deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D
```

运行时由 `scripts/walking_env.sh` 将
`.deps/root/usr/lib/x86_64-linux-gnu` 加入 `LD_LIBRARY_PATH`。检查共享库依赖：

```bash
LD_LIBRARY_PATH="$PWD/.deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}" \
  ldd .deps/root/usr/lib/x86_64-linux-gnu/libOpen3D.so.0.14.1 |
  grep 'not found' || echo 'Open3D runtime dependencies OK'
```

只有输出 `Open3D runtime dependencies OK` 才表示动态库依赖已完整。

#### 3.2.3 Livox SDK2

Livox 源码来自 `.deps/src/Livox-SDK2` submodule，但编译器生成的共享库和静态库是
本机构建产物，因此安装到被忽略的 `.deps/sdk`：

```bash
cmake -S .deps/src/Livox-SDK2 -B .deps/sdk-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/.deps/sdk"
cmake --build .deps/sdk-build -j3
cmake --install .deps/sdk-build
```

构建 `livox_ros_driver2` 时使用第 3.4 节给出的
`LIVOX_LIDAR_SDK_LIBRARY` 和 `LIVOX_LIDAR_SDK_INCLUDE_DIR` 参数。运行时库路径
由第 6 节的 `scripts/walking_env.sh` 配置。

### 3.3 构建控制器

`setup_g1_local_deps.sh` 不构建 G1 控制器，因此本步骤不能省略。本地依赖准备
完成后构建控制器和 DDS 桥：

```bash
/usr/bin/python3 scripts/build_g1_controller.py
```

构建脚本会再次检查 `unitree_sdk2` 和 `unitree_rl_mjlab` 的 submodule 提交，并将
策略配置复制到 `.deps/g1-controller`。

### 3.4 构建 ROS 2 工作区

`setup_g1_local_deps.sh` 只准备 Livox SDK 等依赖，不构建
`fast_lio`、`open3d_loc` 和 `livox_ros_driver2`，因此本步骤不能省略。构建阶段只
加载系统 ROS 和本机依赖路径，不加载尚未完成的本地
`install/local_setup.bash`。使用封装脚本完成构建：

```bash
./scripts/build_g1_ros2.sh
```

脚本会检查 `.deps` 依赖和三个 ROS 包源码，清理外部 overlay，加载系统 ROS，
按固定参数构建，并检查三个目标程序是否生成。

脚本内部执行的等价核心命令如下，仅用于理解构建参数或排查失败：

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

构建成功后才会生成 `install/local_setup.bash`。从此时开始，运行时终端才应加载
`scripts/walking_env.sh`；具体顺序见第 6 节。

## 4. 配置分层

### 4.1 FAST-LIO 配置

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

### 4.2 三维重定位配置

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

### 4.3 TF 与里程计

行走链路使用以下约束：

1. FAST-LIO 发布 `camera_init` 坐标系下的局部 `/Odometry_loc`；它不是地图绝对
   位姿，且行走配置关闭 FAST-LIO 自身的冲突 TF。
2. `open3d_loc` 根据三维地图配准计算 `map -> odom`，并发布 `/odom2map`、
   `/baselink2map` 和地图坐标下的 `/localization_3d`。
3. `scripts/g1_nav_tf.py` 将局部 `/Odometry_loc` 投影为平面
   `odom -> base_footprint -> base_link`。
4. Nav2 通过 `map -> odom -> base_footprint` 组合全局与局部坐标；三个 frame 均
   使用 `use_sim_time`。
5. 验收全局位置时必须检查 `map -> base_footprint`，不能把局部 `/odom` 坐标直接
   与地图真值比较。

`global_localization` 的 `publish_motion_tf` 默认仍为 `true`，以保证上游其他 launch
行为不变；G1 行走定位入口显式设为 `false`。

### 4.4 Nav2 配置

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
- `ensure_g1_nav_lifecycle.py` 按固定顺序配置并激活五个 Nav2 lifecycle 节点；
  不再依赖 Humble lifecycle manager，避免其在 `map_server` 配置响应处卡住。

如果重新生成参数，生成器会写入生成时本机的仓库绝对路径；运行时应以
`map_yaml:=...` 覆盖，避免仓库迁移后加载旧路径。

### 4.5 控制器配置

`scripts/build_g1_controller.py` 只接受以下两个固定提交：

- `unitree_sdk2`: `d1abf001220d3e5f59e00a0c4aaa595e00089a4a`
- `unitree_rl_mjlab`: `1425b15f73bd4095f0df53709d7c389c3eb9e790`

构建时从 `unitree_rl_mjlab/deploy/robots/g1/config` 复制官方策略配置，保留
`Passive`、`FixStand`、`Velocity` 三个状态。策略 ONNX、观测定义和状态机来自
官方仓库；本仓库的 `controller/main.cpp` 是仿真适配层，增加键盘、ROS 速度、
导航独占控制和状态文件。`controller/dds_bridge.cpp` 把 MuJoCo 状态送入官方
`rt/lowstate` / `rt/lowcmd` DDS 链路。

构建脚本会清理 Conda 和 ROS overlay 环境，并固定使用 Ubuntu 的
`/usr/lib/x86_64-linux-gnu/cmake/fmt`，避免误用 Anaconda 中不兼容的 `fmt` 版本。

生成的控制器和策略位于 `.deps/g1-controller`，二进制位于
`.deps/controller-build`，均不提交。

#### 4.5.1 本地键盘控制

键盘功能不是独立的 ROS 节点，也不是 `.deps/g1-controller/config.yaml` 中的运行
参数。它由仿真适配层 `controller/main.cpp` 直接管理：

- 输入设备使用 `unitree_rl_mjlab` 中已有的 `Keyboard` 读取类。
- 只有控制器的标准输入是 TTY 时才启用键盘；重定向、后台运行或非交互启动会关闭
  键盘，但 `/cmd_vel` 入口仍可使用。
- 键盘读取会临时关闭终端回显和规范输入模式。如果控制器异常退出后终端行为异常，
  可在该终端执行 `stty sane` 恢复。

按键与控制器动作的当前映射为：

| 按键 | 动作 |
|---|---|
| `W` / `S` | 前进 `0.15 m/s` / 后退 `0.12 m/s` |
| `A` / `D` | 横向 `+0.15 m/s` / `-0.15 m/s` |
| `Q` / `E` | 左转 / 右转 `0.2 rad/s` |
| 空格 | 立即将键盘速度置零 |
| `1` | 进入 `Passive` |
| `2` | 进入 `FixStand` |
| `3` | 仅在实际状态为 `FixStand` 时进入 `Velocity` |

每次方向键输入会设置一个 0.25 秒的键盘速度有效期，持续运动需要终端继续产生按键
事件。松开按键或停止输入后，速度在有效期结束时回到零，或回到仍然新鲜的 ROS 手动
速度命令。横移速度不能低于 `0.15 m/s`；该策略对 `0.08 m/s` 的横向命令存在启动
死区，按键看似没有反应。

速度控制权的优先级固定为：

1. 导航门禁开启且 `/cmd_vel_nav` 新鲜时，由 Nav2 独占控制。
2. 导航未开启时，0.25 秒内的键盘速度优先。
3. 键盘速度失效后，0.5 秒内的 `/cmd_vel` 手动命令生效。
4. 没有任何新鲜命令时输出零速度。

Nav2 门禁开启时，键盘不能修改运动模式或速度。`scripts/g1_command_bridge.py`
仍持续读取 `/cmd_vel`、`/cmd_vel_nav`，通过本地命令文件交给控制器；控制器再把
实际 FSM 状态写入 `.deps/g1-controller/status-<domain>.txt`，供 operator 和验证
脚本检查。

当前按键映射、速度限幅和 0.25 秒有效期直接写在源码中，没有单独的 YAML 参数。
修改后需要重新执行 `scripts/build_g1_controller.py` 才会生效；这些修改和普通源码
一样通过 Git 管理，生成的控制器与状态文件不提交。

## 5. 对原始项目的修改

### 5.1 FAST-LIO

| 文件 | 修改 |
|---|---|
| `FAST_LIO/launch/mapping.launch.py` | 新增 `debug_log_dir` launch 参数，避免运行日志写回源码目录 |
| `FAST_LIO/src/laserMapping.cpp` | 新增 `publish.tf_en`；修正发布 Path 的 header 时间；调试矩阵和位置日志可写入指定目录 |
| `FAST_LIO/rviz/fastlio.rviz` | 适配 G1 话题、点云显示和初始视角；隐藏重复的全局 TF/里程计显示 |
| `FAST_LIO/config/g1_*.yaml` | 新增 G1 仿真 LiDAR/IMU 外参、参数和建图/定位 profile |

`publish.tf_en` 默认为 `true`，因此未显式配置的上游使用方式仍保持原行为。

### 5.2 Open3D 定位

| 文件 | 修改 |
|---|---|
| `open3d_loc/CMakeLists.txt` | 删除作者机器的硬编码 `Open3D_DIR`，允许 CMake 参数传入 |
| `open3d_loc/src/global_localization.cpp` | `/map` 改为 transient-local；新增 `publish_motion_tf`；空点云跳过 ICP；析构时 join 定位线程 |
| `open3d_loc/launch/g1_*.launch.py` | 新增支架演示、行走建图、行走定位和 Nav2 入口 |

空点云保护用于避开 FAST-LIO 首帧尚未准备好时 Open3D ICP 对空输入抛异常。它不是
定位漂移修复，也不能替代正确的初始位姿、时间同步和地图匹配。

### 5.3 新增仿真、控制和工具

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
- `scripts/setup_g1_local_deps.sh`：初始化第三方源码 submodule，并准备 Python、
  Open3D 和 Livox SDK 本地依赖。
- `scripts/build_g1_ros2.sh`：检查本地依赖并构建三个 ROS 2 包。
- `scripts/start*.sh`：分阶段启动和重复进程锁。
- `scripts/export_g1_map.py`：从 FAST-LIO PCD 生成 PLY、PGM、YAML 和元数据。
- `scripts/check_g1_localization.py`、`validate_walking.py`、
  `validate_nav_goal.py`：时间对齐、行走和导航验证工具。

## 6. 运行配置速查

### 6.1 何时加载 `walking_env.sh`

`walking_env.sh` 是**构建完成后的运行时环境入口**。必须已经成功执行第 3.4 节，
并生成 `install/local_setup.bash`，再在每个新开的交互终端中执行：

```bash
cd /path/to/FAST_LIO_LOCALIZATION_HUMANOID

export ROS_DOMAIN_ID=72  # 可省略；省略时由 walking_env.sh 回退到 72
source scripts/walking_env.sh
```

`walking_env.sh` 会清理外部 workspace overlay，只加载 `/opt/ros/humble` 和本项目
`install`，并设置 Fast DDS、`ROS_DOMAIN_ID`、本地动态库、日志目录和图形渲染环境。
环境变量只影响当前 shell，新终端必须重新 source。

使用 `./scripts/start_*.sh`、`g1_operator.sh` 或验证脚本时，脚本内部会自动 source
`walking_env.sh`。如果在同一终端手工执行 `ros2 topic`、`ros2 service` 等命令，
则必须先显式 source，避免连接到缺少本项目 overlay 或使用不同 RMW/domain 的环境。
不要在 `setup_g1_local_deps.sh` 或 `colcon build` 阶段 source 该文件。

### 6.2 分阶段启动

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
/.deps/*
!/.deps/src/
!/.deps/src/*
/sim/generated/
__pycache__/
```

因此主仓库只跟踪 `.deps/src/` 下五个 submodule 的 URL 和提交指针，不跟踪第三方
源码内容、编译产物、策略二进制、运行日志、点云地图或验证输出。需要从干净 clone
恢复运行环境时，先按第 3 节执行初始化和本地依赖配置，再完成构建和生成地图。

## 8. 验证范围

当前记录验证的是本机 MuJoCo 仿真：

- 三维建图与地图保存。
- 使用生成地图和已知初始位姿的 Open3D 重定位。
- Nav2 目标导航、动态障碍和取消。
- 键盘速度控制、导航独占门禁和分阶段退出。

尚未证明真实 G1 传感器时间、外参、底盘高度、网络 DDS 和 ARM64 环境可用。使用
真机前必须重新采集标定和测试数据，不能直接把仿真坐标参数当作真机配置。
