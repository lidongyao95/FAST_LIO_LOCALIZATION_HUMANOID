# 实机点云生成场景后的 MuJoCo 导航预演

这是[实机部署路线](../docs/WORKFLOWS.md#实机部署路线)中可选的地图预演，在 Ubuntu 仿真机运行。
场景来源于实机点云；运行时的 G1、LiDAR、IMU 和运动全部由 MuJoCo 模拟。
本流程不读取 PC2 的实时雷达，也不向真实 G1 发送速度。

| 对比项 | 内置房间纯仿真 | 本页的实机点云预演 | PC2 实时定位与规划 |
| --- | --- | --- | --- |
| 环境来源 | 仓库定义的虚拟房间 | 已采集 PCD 的近似几何 | 当前真实现场 |
| 运行时输入 | 虚拟 LiDAR/IMU | 在近似场景中重新生成的虚拟 LiDAR/IMU | 现场 MID-360 与 IMU |
| 使用的地图 | 虚拟房间中 FAST-LIO 建图后导出 | 场景生成器另行输出的配套地图 | 经核对的现场点云图与二维底图 |
| 目标执行 | 虚拟 G1 行走 | 虚拟 G1 行走 | 默认网页只规划；实际行走另验 |

没有现场点云、只想跑通软件链时，用[纯仿真路线](../docs/WORKFLOWS.md#纯仿真路线)。
已有录包、只想复现那段输入时，用[离线回放](inverted_mid360.md)；回放不会响应新目标改变轨迹。

`02_mapping.sh export --plaster` 生成图片、表面点云和地面参数，**不生成可运行的 `scene.xml`**。
本页第 2 节用 `g1_scan_scene.py` 单独生成场景。第 5 节则保留特定数据集的石膏版历史场景，
使用实验脚本和 `world-*` 文件，不是通用 `scan_preview.sh` 的目录格式。

本流程将已修正倒装方向的 PCD 转换为近似仿真场景，随后按
[仓库正常启动顺序](../docs/NAVIGATION_MANUAL.md#4-重启并加载生成地图)运行
行走策略、FAST-LIO、Open3D 和 Nav2。MuJoCo 显示 G1 的物理运动，
RViz 用于观察地图、定位和规划，并点击设置目的地及最终朝向。

## 1. 数据与模型范围

下文以 `tmp/g1_real_data/replay/static_01_inverted.pcd` 为例，
由静止录包离线建图获得，生成方法见[倒装补偿与回放](inverted_mid360.md)。
原始录包和生成结果保留在本地，不随代码提交；其他机器需自行提供对应 PCD。
依赖为现有 `.deps/venv` 的 MuJoCo、NumPy，以及仓库原有 ROS 导航环境。

生成器先拟合近似水平地面并调平到 Z=0，在 8 米半径内裁剪扫描：

- 扫描显示使用 9 cm 体素采样，以小方块保留扫描外观，不参与物理碰撞。
- 障碍物使用 25 cm 碰撞体素，每个体素至少包含两个扫描点。
- 地面延伸成有限平坦地板，导航地图将其作为可行区域，边界标为障碍。
- PLY 定位地图由相同碰撞几何采样，PGM/YAML 投影同一几何，保持三维场景与导航地图一致。

这是稀疏扫描的仿真近似，延伸地板和空闲区域属于人工假设；遮挡、孔洞和薄物体
不完整，不能当作完整环境重建或真机可通行性证明。地面变换保存在
`metadata.json` 的 `world_from_camera_init`，不会改写输入 PCD。
仿真使用仓库原有合成传感器安装关系，而非真机安装标定。

生成器输出的 `map.ply` 来自近似碰撞几何，`map.yaml` 将人工延伸地板视为空闲区。
它们虽然与实机导出地图同名，却具有不同的来源、坐标和空闲区假设。
请使用独立目录；回到 PC2 时加载实机地图，不直接部署这组仿真地图。

## 2. 生成场景和配对地图

在仓库根目录执行；生成器只写资产并编译检查，不启动展示或导航进程：

```bash
source scripts/env/walking_env.sh
.deps/venv/bin/python scripts/mapping/g1_scan_scene.py \
  --pcd "$PWD/tmp/g1_real_data/replay/static_01_inverted.pcd" --auto-start
```

默认输出 `sim/generated/real_scan/`，可用 `--out`、`--radius` 指定目录和裁剪半径。

`--auto-start` 会在 0.25 m 网格上挑选离原点最近、与碰撞体保持至少 0.75 m 余量的出生点，并打印 `auto spawn <x> <y> clearance <m>`。实机点云常把原点处的立柱或人当成障碍，此时不加该参数会报 `Scan obstacles overlap robot spawn`；需要固定出生点仍可用 `--start X Y`。

Mac 仓库根有统一入口，可省去手工拷 PCD：

```bash
./scripts/simulation/03_preview.sh prepare pc2:/home/unitree/ldy/g1_real_data/maps/B1_run03.pcd \
  --name B1_run03_preview --pc2 unitree@<当次无线IP>
./scripts/simulation/03_preview.sh play sim/generated/B1_run03_preview
```
首次确认输入已经完成倒装方向修正，且传感器原点在地面上方；地面拟合失败、障碍物
覆盖默认出生区域时生成器会报错，需检查输入。不要在仿真运行期间覆盖场景资产。

| 文件 | 用途 |
|---|---|
| `scene.xml` | 含官方 29 自由度 G1、支架和动态障碍接口的 MuJoCo 场景 |
| `map.ply` | 与仿真碰撞几何一致的 Open3D 定位地图 |
| `map.pgm` / `map.yaml` | 同坐标系的 Nav2 占据地图 |
| `metadata.json` | 来源、坐标变换、地面拟合和体素统计 |

XML 引用本机官方 G1 模型绝对路径，搬迁后需重新生成。
默认房间与 `sim/generated/scene.xml` 保持原有用途。

可在启动前执行独立的稳定性、落球碰撞及离屏渲染检查：

```bash
MUJOCO_GL=egl .deps/venv/bin/python scripts/checks/check_g1_scan_scene.py --render
```

输出 `validation.json`、`collision_validation.json` 和 `preview.png`。
该检查临时使用骨盆支撑和关节 PD，不提供另一套交互行走入口；
落球只验证选定碰撞体素，不代表全场景碰撞质量或步行能力通过验收。

## 3. 按正常顺序启动

场景目录生成后，可从仓库根目录在一个终端启动。脚本使用该目录中的场景和两张地图，并复用纯仿真的控制器、定位和 Nav2 启动顺序。

```bash
./scripts/simulation/scan_preview.sh sim/generated/real_scan
```

该统一入口默认域为 74。若需另开终端查询状态，先导出相同的 `ROS_DOMAIN_ID` 再加载环境。
导航就绪后在 RViz 点选目标，按第 4 节观察与停止。

下方是另一种启动方式：分终端调试。先结束统一入口，再按下方步骤启动，不要叠加运行。

所有终端都在仓库根目录，默认 `ROS_DOMAIN_ID=72`，直接执行 `ros2` 前先
`source scripts/env/walking_env.sh`。若自行指定域，必须每个终端一致。
不要同时运行离线录包、旧的 FAST-LIO 或多个时钟发布者于相同域。
每阶段按日志或状态确认就绪后再继续，避免仿真尚在加载就启动控制器导致 DDS 端口争用。

终端 1，启动行走仿真，等待 `DDS mode: base held until /g1/release`：

```bash
./scripts/simulation/start_walking_sim.sh --model-path "$PWD/sim/generated/real_scan/scene.xml"
```

终端 2，启动控制器，等待 `FSM: Start Passive`：

```bash
./scripts/simulation/start_g1_controller.sh
```

终端 3，启动原有 FAST-LIO/Open3D 定位并绑定三维地图：

```bash
./scripts/simulation/start_g1_localization.sh \
  map_ply:="$PWD/sim/generated/real_scan/map.ply" \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.793 initial_yaw:=0.0 \
  rviz:=false
```

这里的初始高度 0.793 m 属于官方仿真骨盆模型；不要用于真机。
仿真传感器已经按正向坐标输出，不开启倒装输入补偿。

终端 4，检查定位和站立，再释放支架：

```bash
./scripts/simulation/g1_operator.sh status
./scripts/simulation/g1_operator.sh prepare
./scripts/simulation/g1_operator.sh status
```

确认 `controller_mode: 3`、`released: true`、`odom_fresh: true`，
定位连续有效后，在终端 5 启动 Nav2 和唯一的导航 RViz：

```bash
./scripts/simulation/start_g1_navigation.sh map_yaml:="$PWD/sim/generated/real_scan/map.yaml"
```

等待日志 `All G1 Nav2 lifecycle nodes active`，再在终端 4 开启运动门禁：

```bash
./scripts/simulation/g1_operator.sh enable-nav
```

确认 `nav_ready: true`。仿真真值仅用于验证，导航定位继续来自 FAST-LIO/Open3D。

## 4. 点击目的地和朝向、停止

RViz 顶部选择 **Nav2 Goal**，在地图可行区域按住左键，拖动箭头设置最终朝向，
松开下发。先选择近距离目标，确认地图、实时点云和规划路径对齐，
MuJoCo 中 G1 同步运动。不要同时运行自动发目标测试，以免相互抢占。

停止导航：

```bash
./scripts/simulation/g1_operator.sh stop
```

结束整次仿真时先停止导航，再依次 Ctrl-C 关闭终端 5、3、2、1。
若定位跳变、机器人倾倒或地图对齐异常，停止导航并检查前序阶段。

本次[调试与验证记录](debug_log/2026-11-04_scan_navigation.md)区分了离线建图、
物理碰撞和 Nav2 到达结果。这些结果不代替真机导航验收。

## 5. 4322 石膏版手动导航

本节使用实测扫描重建石膏场景，并在 4322 的 MuJoCo 中运行模拟 G1。下文命令以 B300 门口生成的一套场景和配套地图为例。
彩虹对比、DUFOMap、石膏预览和原坐标系二维地图见
[点云过滤与选参说明](hang_a02_point_cloud_filters.md)。石膏 OBJ 用于显示，
碰撞和模拟雷达使用同场景的平地与独立障碍体素。

### 场景文件

在 4322 上使用 `~/ldy/hang-a02-filter-comparison/plaster-map/mujoco/` 中成套的
`scene.xml`、`plaster.obj`、`world-cloud.ply`、`world-map.yaml` 和
`world-map.pgm`。`trial.json` 保存仿真变换与试跑结果。场景 XML 引用 4322 的绝对路径。
需要重建时，按[实验脚本说明](../scripts/experiments/hang_a02/README.md)运行 `trial.py`
和 `navigation_assets.py`。后者将点云和二维图转换到仿真 world，定位和导航均使用转换后的文件。

### 启动与交接

从 Mac 可用 `ssh -B ipsec0 -p 4322 sjtu@202.121.181.124` 登录。
每个终端执行同样的环境设置。这里用上次试跑的域 149，若该域已有会话，先检查其状态，
或为整套新会话选择一个空闲域。可视窗口打开在 4322 的 `:0` 桌面。

```bash
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export DISPLAY=:0 ROS_DOMAIN_ID=149 ROS_LOCALHOST_ONLY=1
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
scene="$HOME/ldy/hang-a02-filter-comparison/plaster-map/mujoco"
```

按顺序在独立终端执行。每一步就绪后再继续。

| 顺序 | 命令 | 就绪判据 |
|---|---|---|
| 1 仿真 | `./scripts/simulation/start_walking_sim.sh --model-path "$scene/scene.xml" --start-x 3.987606 --start-y 5.249173` | MuJoCo 窗口显示石膏场景，日志出现 `DDS mode`。 |
| 2 控制 | `./scripts/simulation/start_g1_controller.sh` | 日志出现 `FSM: Start Passive`。 |
| 3 定位 | `./scripts/simulation/start_g1_localization.sh map_ply:="$scene/world-cloud.ply" initial_x:=3.987606 initial_y:=5.249173 initial_z:=0.793 initial_yaw:=0.0 rviz:=false` | Open3D 完成初始化且位姿持续更新。 |
| 4 站立 | `./scripts/simulation/g1_operator.sh prepare` | `status` 返回 `controller_mode: 3`、`released: true`、`odom_fresh: true`。 |
| 5 Nav2 | `./scripts/simulation/start_g1_navigation.sh map_yaml:="$scene/world-map.yaml" rviz:=false` | 日志出现 `All G1 Nav2 lifecycle nodes active`。 |
| 6 RViz | 见下方命令 | 显示石膏版配套二维图，导航目标工具可用。 |
| 7 开启手动导航 | `./scripts/simulation/g1_operator.sh enable-nav` | `status` 返回 `nav_ready: true`。 |

```bash
source scripts/env/walking_env.sh
rviz_config="$scene/manual/Plaster-Navigation.rviz"
if [ ! -s "$rviz_config" ]; then
  rviz_config="$PWD/open3d_loc/rviz_cfg/g1_navigation.rviz"
fi
glxinfo -B | grep 'OpenGL renderer'
rviz2 -d "$rviz_config" --ros-args -p use_sim_time:=true &
rviz_pid=$!
# 等窗口打开后核对 RViz 进程的 GPU 占用。
nvidia-smi | grep "$rviz_pid"
```

默认 RViz 配置打开后可缩放并平移到地图坐标约 `(4.0, 5.25) m`。
选择 **Nav2 Goal**，从机器人附近的空闲区开始，按住左键拖动设定朝向后松开。
停止时运行 `./scripts/simulation/g1_operator.sh stop`，然后按 RViz、Nav2、定位、控制、仿真的顺序关闭。

上次已在 4322 验证模拟定位、Nav2 激活、运动门禁和 `/navigate_to_pose` 服务端就绪，
并确认 RViz 使用 RTX 4090。日志出现过代价地图因点云时间早于 TF 缓存而丢帧。
现场复现时同时观察两张代价地图及 TF。手动到点结果以实际试跑记录为准。

## 02 后可选 03 一体化入口

在操作电脑的仓库根执行 `03_preview.sh play pc2:/绝对/采集.pcd --name 新名称 --pc2 unitree@当次无线IP`：集成上传与 SHA-256 校验、自动安全起点、场景/配套地图生成、服务器导航启动。`prepare` 只准备，`launch sim/generated/名称` 复用已生成场景；已有场景也可直接 `play`。导航参数放在 `--` 后，例如 `-- --headless --goal X Y`。

默认服务器 4622，连接失败或 GPU 空闲显存 <2 GiB 时尝试 4322；显式 G1_SIM_PORT 不回退。默认仿真域 75，不得用实机 223。自动读取生成元数据的出生点，启动前检查动态库并可复用服务器已有只读 ROS 覆盖层。生成进程日志与 pid/status 在服务器 `~/ldy/g1_real_data/records/preview/`，每 25 秒检查进程/日志增长/终态。

生成器支撑 weld 使用显式零相对位姿，避免非零自动出生点保留编译前的原点偏移。仿真派生图和实机原图独立；03 可跳过，预演结束回到 PC2 直接用原实机图执行 04。
