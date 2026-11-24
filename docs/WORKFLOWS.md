# G1 使用流程：纯仿真与实机部署

先选路线，再执行该路线中的步骤。脚本名中的 `01`～`05` 是入口编号，**不表示所有用户都要从 1 顺序执行到 5**。纯仿真使用自检和 `03_simulation.sh`；实机部署使用自检、建图、定位和 Web，可按需插入地图预演。

直接跳转：[纯仿真路线](#纯仿真路线) · [实机部署路线](#实机部署路线) · [DUFOMap 动态点清理](#用-dufomap-清理多帧扫描) · [地图处理选项](#地图处理的常用选项) · [域与日志](#ros-域日志与结束会话)

## 先分清四种运行方式

| 方式 | 环境和数据从哪里来 | 谁在运动 | 用于判断什么 |
| --- | --- | --- | --- |
| 纯仿真 | 内置 MuJoCo 房间，运行时生成虚拟 LiDAR/IMU | MuJoCo 中的 G1 | 软件链能否完成建图、重定位、导航、障碍与取消回归 |
| 实机地图的仿真预演 | 实机 PCD 近似成场景，运行时重新生成虚拟 LiDAR/IMU | MuJoCo 中的 G1 | 近似现场中的场景、地图、定位和规划是否配套 |
| 实机录包回放 | 回放已经录下的 LiDAR/IMU，不重新模拟传感器 | 不驱动机器人；数据轨迹固定 | 历史输入上的建图、定位与时间/TF 问题能否复现 |
| PC2 实时运行 | 现场 MID-360 和 IMU，经 FAST-LIO 提供实时点云与里程计 | 本页只提供规划，不驱动机器人 | 现场输入、定位、地图、代价地图与规划链是否可用 |

**根本区别是运行时的输入和控制对象，而不只是地图是不是来自实机。** 回放中给 Nav2 一个新目标不会改变录包轨迹；扫描场景预演可以闭环运动，但运动发生在仿真中。两者均不能代替实机运动验收。

以下命令均从仓库根目录执行。Mac 可以做离线地图处理和 SSH 操作；MuJoCo/ROS 步骤在对应 Ubuntu 主机执行。脚本的 `--help` 列出参数。除录制、导出、自检等有限任务外，会话留在前台，按 Ctrl+C 停止本次启动的进程。

## 启动前的进程预检

每次重启或切换阶段时，先在实际运行 ROS 的 Linux 主机检查旧会话。采集入口检查建图和定位进程，持续定位入口检查旧定位，Web 入口检查旧导航；仿真统一入口检查整套会话。检查发现冲突会列出 PID 并停止本次启动，加载环境本身不清理进程。

以重启实机网页规划为例，先设置 B1 的 ROS 域，再执行：

```bash
# 默认只列出本仓库、当前用户、当前 ROS 域的导航进程。
./scripts/env/g1_cleanup_stale.sh --list --scope navigation
# 优先在旧启动终端 Ctrl+C；终端已丢失时，再显式清理清单中的进程。
./scripts/env/g1_cleanup_stale.sh --stop --scope navigation
./scripts/env/g1_cleanup_stale.sh --check --scope navigation
```

`navigation` 保留正在提供数据的雷达、FAST-LIO 和 Open3D；重启定位用 `localization`，建图用 `mapping`。结束整个仿真会话后，可检查 `--scope all`。支持 `--domain N` 显式指定域；清理只依次发送 SIGINT、SIGTERM，仍未退出会报出 PID，不强制杀死。不属于本仓库的节点还需结合 ROS 图检查，并回到它的原启动终端处理。

## 纯仿真路线

目标：不用连接真实 G1，在内置房间完成“走动建图 → 保存 → 重启定位 → 目标导航”。本路线不需要先执行实机 `02_mapping.sh record`。

### A1. 准备仿真机并自检

运行位置：Ubuntu x86_64 / ROS 2 Humble。首次使用先按[仿真环境与配置](CONFIGURATION.md#首次安装的最短路径)安装和构建。

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
./scripts/workflows/01_self_check.sh sim
```

完成判据：依赖、ROS 包和仿真控制器产物检查通过。这只说明可以继续启动，不代表运行链路已验收。

如果宿主机的 ROS 缺少 `nav2`/`rviz2`、又没有 root 可以安装（例如 4622），用同一批命令加上叠加包装脚本：

```bash
./scripts/env/ros_overlay_ns.sh ./scripts/workflows/01_self_check.sh sim
./scripts/env/ros_overlay_ns.sh ./scripts/simulation/03_simulation.sh map sim/generated/room_run02
```

包装脚本把 `~/ldy/logs/*/deps` 中解包的额外 ROS 包以只读 overlay 挂进一个独立用户命名空间，其余参数和判据都不变；可用 `ROS_DEPS_DIR` 指定别的层目录。A2、A3 的 `03_simulation.sh` 命令在 4622 上同样要加这个前缀。

另外 4622 是 SSH 登录的图形会话，启动前先让子进程继承到显示器，否则 MuJoCo 和 RViz 会起不来：

```bash
# 换成自己的图形会话号（4622 上常用 :11）
export DISPLAY=:11
xdpyinfo >/dev/null && echo display-ok
```

### A2. 在虚拟房间里建图并保存

```bash
# 选择尚不存在的输出目录（已有目录会被拒绝，例如换 room_run02）。
./scripts/simulation/03_simulation.sh map sim/generated/room_run02
```

等待控制器站稳、支架释放、FAST-LIO 收到虚拟传感器数据，随后进入实时建图控制：**按住** `w/a/s/d` 前后左右移动、`q/e` 转向，松手即停（终端按键重复超时 0.45 s 就判定松开），不需要回车，也不需要输入秒数；轻点一下只走约 0.45 s，按空格或 `x` 立即停下，按回车保存地图并结束。需要长时间走就按住不放，或连续敲同一个方向键。走遍起点、目标及连接区域即可。

若把输入用管道喂给脚本（非交互终端，例如自动化脚本），会退回按行模式：写成“方向 + 秒数”如 `w 5`，空行保存。

完成判据：`/map_save` 成功，会话结束，目录内有 `map.pcd`、`map.ply`、`map.pgm`、`map.yaml` 和 `metadata.json`。PCD 来自本轮虚拟 LiDAR/IMU 的 FAST-LIO 建图；PLY 给 Open3D 定位，PGM/YAML 给 Nav2。

### A3. 重启虚拟机器人，用刚保存的地图导航

```bash
./scripts/simulation/03_simulation.sh nav sim/generated/room_run02
```

脚本启动同一个内置房间，加载 A2 的三维地图与二维图，检查定位和 Nav2 后开启仿真运动门禁。在 RViz 中用 **Nav2 Goal** 点选已建图的空闲区域，观察 MuJoCo 中的机器人、实时点云和路径是否一致。

完成判据：定位持续更新，目标被接受，虚拟 G1 执行路径并到达。仅看到地图或规划线，还不能说明闭环导航成功。需要自动记录结果时，先 Ctrl+C 结束上一轮，再运行：

```bash
./scripts/simulation/03_simulation.sh nav sim/generated/room_run02 \
  --goal 3.0 0.0 --obstacle-after 2
```

目标需位于 A2 已覆盖且可达的区域。结果写入本轮日志目录的 `nav_goal.json`；脚本检查目标状态及障碍插入结果。`--headless` 可关闭窗口，`--start X Y --start-yaw RAD` 可选择覆盖区内的新起点。手动逐组件调试见[仿真运行手册](NAVIGATION_MANUAL.md)。

## 实机部署路线

目标：用真实 G1 的传感器生成并核对现场地图，在 PC2 上持续定位，再查看网页与规划。地图预演在 Ubuntu 仿真机完成，属于可选支线，结束后回到 PC2 继续。

```text
PC2 环境与传感器 → 现场采集 → 导出并核对实机地图 → PC2 持续定位 → Web 仅规划
                                  └→ 可选：生成扫描场景 → MuJoCo 预演 ─┘
```

### B1. 准备 PC2 环境

运行位置：PC2 / ARM64 / Ubuntu 20.04 / ROS 2 Foxy。Mac 用 SSH 登录，后续 PC2 命令在 SSH 会话内执行。首次部署见[PC2 部署指南](../g1_real_robot_docs/g1_real_robot_slam_pc2_b300_guide.md)；历史硬件记录按[实机文档入口](../g1_real_robot_docs/README.md)中的日期和设备区分。

每个参与本轮实机运行的 PC2 终端都先执行：

```bash
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export G1_DATA="$HOME/ldy/g1_real_data"
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
```

```bash
./scripts/workflows/01_self_check.sh robot
```

完成判据：所需 ROS 包和本机部署环境检查通过。此自检不查询雷达流或运控模式。换机/换架构应重新构建；源码同步与 `install/` 更新要求见[脚本索引](../scripts/README.md#部署时要一起更新什么)。

### B2. 采集并导出现场地图

PC2 执行；已有可用 PCD 时可跳过录制，直接导出：

```bash
./scripts/workflows/02_mapping.sh record B1_run01
```

入口复用活跃的 Livox；没有雷达端点时启动已配置的驱动，并启动 FAST-LIO 和录包。按现场采集安排覆盖要导航的区域，回到终端按 Enter 保存。本入口负责采集，机器人移动按现场已有操作方式完成。

完成判据：`$G1_DATA/maps/B1_run01.pcd` 保存成功，`$G1_DATA/bags/B1_run01/metadata.yaml` 中雷达和 IMU 消息数均非零。原始日志在 `$G1_DATA/records/B1_run01/`。这说明数据已落盘，地图质量还需核对。

若需用 DUFOMap 去除动态点，在采集期间另开终端运行[逐帧扫描采集](#用-dufomap-清理多帧扫描)，或之后回放录包再采集。`record` 保存的单张 PCD 不包含 DUFOMap 所需的逐帧射线起点，不能直接作为它的输入；普通 `export` 也不会自动运行 DUFOMap。

导出可以在 PC2 或已装 NumPy/OpenCV 的地图处理机执行。跨机时先复制 PCD；以下两条路径均指执行机器上的文件：

```bash
./scripts/workflows/01_self_check.sh offline
./scripts/workflows/02_mapping.sh export /path/to/B1_run01.pcd /path/to/B1_map01 --plaster
```

`/path/to/...` 需替换为实际路径，输出目录须尚不存在。完成后检查墙、门口、起点、目标及未知区。地图起点处于占用格时，先查遮挡、坐标和地图覆盖，不能仅凭导出成功继续。

| 导出文件 | 用途 |
| --- | --- |
| `map.ply` | 现场点云降采样后的三维定位图，保留输入坐标 |
| `map.yaml`、`map.pgm` | 同坐标系二维规划图，未观测区域保留为未知 |
| `metadata.json` | 来源、分辨率、地面拟合及栅格统计 |
| `plaster.png`、`surface.ply`、`plaster.json` | 加 `--plaster` 才生成的可视预览与地面参数 |

**`--plaster` 不生成 `scene.xml`，也不会启动 MuJoCo。** 只需实时定位和网页规划时，直接进入 B4。

### B3. 可选：在仿真机预演实机地图

运行位置：完成 A1 环境准备的 Ubuntu 仿真机。把已修正安装方向的 PCD 复制过去，**用独立输出目录**生成近似场景：

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
.deps/venv/bin/python scripts/mapping/g1_scan_scene.py \
  --pcd /path/to/B1_run01.pcd --out sim/generated/B1_preview01
./scripts/simulation/scan_preview.sh sim/generated/B1_preview01
```

这里重新生成 `scene.xml` 和配套的 `map.ply/map.yaml/map.pgm`；定位图取自近似碰撞几何，地面经过调平和人工延伸。运行链是“虚拟 LiDAR/IMU → FAST-LIO → Open3D → Nav2 → 虚拟 G1”，没有接入 PC2 的实时雷达或硬件速度输出。

完成判据：场景能加载，虚拟点云与配套地图对齐，预演目标及障碍用例符合预期。它有助于发现场景或地图配套问题；真实标定、传感器延迟、行走振动和运控响应仍由现场验证。

**B4/B5 使用 B2 导出并核对的实机地图，不直接使用 B3 的仿真地图。** 两个目录虽然都叫 `map.ply`、`map.yaml`，几何、坐标与空闲区假设可能不同。生成原理、初始高度及石膏版历史场景见[扫描场景预演手册](../g1_real_robot_docs/scan_mujoco_preview.md)。

### B4. 回到 PC2，启动持续定位并检查状态

将 B2 的整组地图放到 PC2，例如 `$G1_DATA/maps/B1_map01/`。在完成 B1 环境设置的终端执行：

```bash
./scripts/real_robot/04_robot_ready.sh localize \
  --map "$G1_DATA/maps/B1_map01/map.ply"
```

按提示输入本次测得的 `map ← camera_init` 初始对齐，顺序为 `X Y Z ROLL PITCH YAW`，单位米和度；也可用 `--initial` 传入。`camera_init` 是本次 FAST-LIO 启动的局部坐标系，此处初值是它到地图的对齐，不是把机体在地图上的位置直接照填。不要照抄仿真起点或 `initial_z=0.793`。

入口保留已有 FAST-LIO，或启动驱动与 FAST-LIO，再启动 Open3D 持续配准。已有旧 Open3D 或 `map → body` 对齐时会报错，先确认并结束冲突会话。保持本终端运行，另开一个完成 B1 环境设置的 PC2 终端检查：

```bash
./scripts/real_robot/04_robot_ready.sh status
# 可选：另外查询运控模式，NIC 换成机器人通信网卡名。
./scripts/real_robot/04_robot_ready.sh status --interface NIC
```

完成判据：雷达/IMU、FAST-LIO 机体云和里程计有新数据，配准健康且 `map → body` 更新。还需结合地图与现场核对位置和朝向。`status` 是只读检查，不负责启动定位，也不表示已经获得实机行走许可。SDK 查询工具见[构建记录](../g1_real_robot_docs/debug_log/2026-11-10_pc2_live_nav_web.md#sdk-会话二进制的构建)。

### B5. 启动 Web 仅规划

保持 B4 定位终端运行，在另一个完成 B1 环境设置的 PC2 终端执行：

```bash
./scripts/real_robot/05_web_navigation.sh "$G1_DATA/maps/B1_map01" --port 18081
```

在 Mac 上转发网页端口；无线连接可换成现场确认的 PC2 地址：

```bash
ssh -N -L 18081:127.0.0.1:18081 unitree@192.168.123.164
```

打开 [Web 页面](http://127.0.0.1:18081/)。先核对位姿箭头、实时雷达点和地图是否对齐，再点选目标查看路径。

网页的“规划考虑实时障碍”默认开启，只影响全局规划：关闭后使用静态地图障碍，实时雷达点仍可显示。两个“显示”选项只控制画面。切换后旧路径失效，需要重新规划；状态未知或切换失败会显示原因，详见[开关说明](../scripts/real_robot/web/README.md#实时障碍参与规划)。

网页还提供“点群判定半径”“最少支持点数”“障碍膨胀距离”三个滑块。拖动预览，松手后应用并使旧路径失效；点群阈值只影响实时障碍，膨胀距离同时影响静态和实时障碍。Foxy 修改膨胀距离时会短暂停止规划并重载代价地图，页面等待确认后才恢复规划。

此入口启动地图、规划器、全局代价地图、规划障碍输入节点和 Web，不启动控制器、局部代价地图、导航行为树或速度桥。网页点击目标仅请求 `ComputePathToPose` 路径。

完成判据：地图、位姿、代价地图、实时雷达图层和规划器就绪；现场可通行目标能规划，阻断目标符合预期。用同一目标分别检查开关两种状态下的路径和代价地图。

本路线到此完成实时感知、定位与规划。实际自主行走需要另行实现控制链，验证停止、超时、断链和接管。旧版本的测试保存在[历史行走记录](../g1_real_robot_docs/2026-11-10_g1_autonomous_walk_plan.md)，其中已删除的命令不属于当前操作流程。

## 地图处理的常用选项

首次运行可沿用默认值；需要调图时再查本节。

- 录制可加 `--seconds 60` 自动结束，或用 `--point-filter 3 --surface-voxel 0.5 --mapping-voxel 0.5` 调整 FAST-LIO。雷达话题自动识别 `/livox/lidar` 或 `/livox/custom_msg`，也可指定 `--lidar-topic`。
- 导出预设 `detail`、`standard`、`quick` 的三维体素 / 二维分辨率分别为 0.03/0.05、0.06/0.08、0.12/0.12 m，用 `--preset` 选择，或分别用 `--voxel`、`--resolution` 覆盖。二维投影使用三维降采样前的点。
- 二维图支持 `--max-range`、`--floor-band`、`--obstacle-min`、`--obstacle-max`。默认拟合地面分类；已整平点云可显式选 `--floor-model percentile`。地面拟合失败时先检查地面回波覆盖。
- `--plaster` 默认剔除离地 0.16 m 以下的回波，在 15 m 半径、离地 2.8 m 内以 0.08 m 网格生成预览。用 `--plaster-ground-cut`、`--plaster-radius`、`--plaster-height`、`--plaster-grid` 调整，只影响预览文件。
- 离线 Python 可用 `G1_PYTHON=/path/to/venv/bin/python` 指定。导出需要 NumPy/OpenCV；石膏预览另需 SciPy、scikit-image、Matplotlib。

## 用 DUFOMap 清理多帧扫描

这是导出地图前的可选清理步骤。代码入口是 `scripts/workflows/02_mapping.sh dufo`，
底层调用 `scripts/mapping/dufomap_clean.py`；算法本体来自 Python 环境中的
`dufomap==1.1.1` 包，因此仓库没有单独的 DUFOMap 算法源码目录。

DUFOMap 需要多帧点云、精确配对时间戳和每帧雷达原点；单张 PCD 可直接导出，无需此步骤。实际采集或录包回放时，保持 FAST-LIO 运行，在同一 ROS 域的另一个终端执行：

```bash
./scripts/workflows/02_mapping.sh capture-scans /path/to/scans \
  --lidar-in-body X Y Z --frames 300
G1_PYTHON=/path/to/dufomap-venv/bin/python \
  ./scripts/workflows/02_mapping.sh dufo /path/to/scans /path/to/B1_clean01 --d-p 1 --plaster
```

`X Y Z` 是当前挂载与 FAST-LIO 配置下 LiDAR 在 `body` 中的实测平移（米）。采集可用 `--idle-timeout` 代替固定帧数。后处理使用 `dufomap==1.1.1`，历史环境为 Linux x86_64，可复制扫描到兼容机器处理。默认 `--d-s 0.2` m、`--d-p 2` 体素；`--d-p 1` 是示例候选，需核对误删情况。独立清空复扫可用 `--revisit-after STAMP_NS` 指定起始纳秒时间戳，见[验证记录](superpowers/plans/2026-11-05-dufomap-sim.md)。

## ROS 域、日志与结束会话

| 入口 | 未设置 `ROS_DOMAIN_ID` 时的默认域 | 默认工作流日志 |
| --- | --- | --- |
| `03_simulation.sh`、`scan_preview.sh`、自检 `sim` | 74 | `sim/generated/workflow_logs/` |
| PC2 采集、定位、Web、自检 `robot` | 223 | `$G1_DATA/records/workflows/` |
| 手动 `start_*.sh` 与仿真检查脚本 | 72 | 见各脚本输出 |
| 支架传感器演示 `run_g1_mujoco.sh` | 71 | `sim/generated/logs/` |

已导出的域会覆盖默认值。同一会话的所有终端必须一致，尤其是在域 74 运行统一入口后，另开终端手动执行 `ros2` 或检查脚本时：

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source scripts/env/walking_env.sh
```

PC2 实时入口改用域 223。仿真、录包回放与现场实时链分开运行；实机入口会拒绝域内的 `/clock` 或仿真真值发布者。实时 PC2 使用系统时间，MuJoCo 行走链使用仿真时钟；Foxy 录包回放的时钟处理见[离线回放说明](../g1_real_robot_docs/inverted_mid360.md)。

入口启动时打印本轮日志目录，可用 `G1_WORKFLOW_LOGS` 改变位置。失败时先查对应组件日志和 `probe-*.log`；进程冲突按[启动前的进程预检](#启动前的进程预检)处理。

统一入口按 Ctrl+C 清理本次启动的进程。实机 Web 和定位分属两个终端，先停 Web，再停定位；复用的外部驱动或 FAST-LIO 仍由原终端管理。纯仿真会话里导航门禁一直保持开启，带或不带 `--goal` 都可以在同一轮里连续点选多个目标；`--goal` 只多跑一次自动验证，验证结束不会关门禁。要换地图或换起点时再 Ctrl+C 重开一轮。
