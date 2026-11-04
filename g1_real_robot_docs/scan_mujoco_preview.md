# 真机扫描场景的 MuJoCo 与 RViz 导航

本流程将已修正倒装方向的 PCD 转换为近似仿真场景，随后按
[仓库正常启动顺序](../NAVIGATION_MANUAL.md#4-重启并加载生成地图)运行
行走策略、FAST-LIO、Open3D 和 Nav2。MuJoCo 显示 G1 的物理运动，
RViz 用于观察地图、定位和规划，并点击设置目的地及最终朝向。

## 1. 数据与模型范围

本次输入为 `tmp/g1_real_data/replay/static_01_inverted.pcd`，
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

## 2. 生成场景和配对地图

在仓库根目录执行；生成器只写资产并编译检查，不启动展示或导航进程：

```bash
source scripts/walking_env.sh
.deps/venv/bin/python scripts/g1_scan_scene.py \
  --pcd "$PWD/tmp/g1_real_data/replay/static_01_inverted.pcd"
```

默认输出 `sim/generated/real_scan/`，可用 `--out`、`--radius` 指定目录和裁剪半径。
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
MUJOCO_GL=egl .deps/venv/bin/python scripts/check_g1_scan_scene.py --render
```

输出 `validation.json`、`collision_validation.json` 和 `preview.png`。
该检查临时使用骨盆支撑和关节 PD，不提供另一套交互行走入口；
落球只验证选定碰撞体素，不代表全场景碰撞质量或步行能力通过验收。

## 3. 按正常顺序启动

所有终端都在仓库根目录，默认 `ROS_DOMAIN_ID=72`，直接执行 `ros2` 前先
`source scripts/walking_env.sh`。若自行指定域，必须每个终端一致。
不要同时运行离线录包、旧的 FAST-LIO 或多个时钟发布者于相同域。
每阶段按日志或状态确认就绪后再继续，避免仿真尚在加载就启动控制器导致 DDS 端口争用。

终端 1，启动行走仿真，等待 `DDS mode: base held until /g1/release`：

```bash
./scripts/start_walking_sim.sh --model-path "$PWD/sim/generated/real_scan/scene.xml"
```

终端 2，启动控制器，等待 `FSM: Start Passive`：

```bash
./scripts/start_g1_controller.sh
```

终端 3，启动原有 FAST-LIO/Open3D 定位并绑定三维地图：

```bash
./scripts/start_g1_localization.sh \
  map_ply:="$PWD/sim/generated/real_scan/map.ply" \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.793 initial_yaw:=0.0 \
  show_rviz:=false
```

这里的初始高度 0.793 m 属于官方仿真骨盆模型；不要用于真机。
仿真传感器已经按正向坐标输出，不开启倒装输入补偿。

终端 4，检查定位和站立，再释放支架：

```bash
./scripts/g1_operator.sh status
./scripts/g1_operator.sh prepare
./scripts/g1_operator.sh status
```

确认 `controller_mode: 3`、`released: true`、`odom_fresh: true`，
定位连续有效后，在终端 5 启动 Nav2 和唯一的导航 RViz：

```bash
./scripts/start_g1_navigation.sh map_yaml:="$PWD/sim/generated/real_scan/map.yaml"
```

等待日志 `All G1 Nav2 lifecycle nodes active`，再在终端 4 开启运动门禁：

```bash
./scripts/g1_operator.sh enable-nav
```

确认 `nav_ready: true`。仿真真值仅用于验证，导航定位继续来自 FAST-LIO/Open3D。

## 4. 点击目的地和朝向、停止

RViz 顶部选择 **Nav2 Goal**，在地图可行区域按住左键，拖动箭头设置最终朝向，
松开下发。先选择近距离目标，确认地图、实时点云和规划路径对齐，
MuJoCo 中 G1 同步运动。不要同时运行自动发目标测试，以免相互抢占。

停止导航：

```bash
./scripts/g1_operator.sh stop
```

结束整次仿真时先停止导航，再依次 Ctrl-C 关闭终端 5、3、2、1。
若定位跳变、机器人倾倒或地图对齐异常，停止导航并检查前序阶段。

本次[调试与验证记录](debug_log/2026-11-04_scan_navigation.md)区分了离线建图、
物理碰撞和 Nav2 到达结果。这些结果不代替真机导航验收。
