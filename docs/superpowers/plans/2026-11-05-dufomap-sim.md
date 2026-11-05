# DUFOMap 仿真实施计划

**目标** 在 4322 验证离线动态点清理，输出可供 Open3D 和二维地图导出使用的 PCD。

**架构** 按时间戳配对 FAST-LIO 的 body 点云与里程计，使用 LiDAR 在 body 中的平移恢复射线起点。离线调用固定版本的 DUFOMap，按其动态点标签筛选原始地图坐标点。先用确定性射线仿真确认动态清除和静态保留，再跑 MuJoCo → FAST-LIO → DUFOMap。

- [x] 在隔离目录安装 dufomap 1.1.1 的 x86_64 wheel，确认 API。
- [x] 实现逐帧数据接收与清理 CLI，校验帧、时间戳、外参与有限坐标。
- [x] 在 4322 运行确定性射线场景，分别记录转向、换位回扫和持续占用的结果。
- [x] 使用隔离 ROS 域运行 MuJoCo 和 FAST-LIO，核对逐帧匹配、地图格式与导出结果。
- [x] 记录指标、命令和边界，提交独立分支。

范围限纯仿真。现有 4322 仿真安装仅用于读取依赖与运行已编译的 FAST-LIO，试验输出写入单独目录。

## 在 4322 复现

在隔离 Python 环境安装 `dufomap==1.1.1`。ROS 采集端使用 Humble 自带的
`rclpy`、`message_filters` 和 `sensor_msgs_py`。仿真脚本用已有 MuJoCo 环境，
工作树需要可读的 G1 模型子模块。三个终端分别 source Humble 与 FAST-LIO
overlay，给本轮分配一个空闲 ROS 域，并设置 `ROS_LOCALHOST_ONLY=1`。

```bash
# 终端 1，从仿真工作树运行
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$PWD/FAST_LIO/config/g1_mujoco.yaml"
```

```bash
# 终端 2，先启动采集
python3 scripts/record_dufomap_scans.py /tmp/g1-dufo-scans \
  --lidar-in-body 0 0 0.45 --frames 150 --timeout 45
```

```bash
# 终端 3，使用与 ROS 兼容的 MuJoCo Python 环境
python scripts/g1_mujoco_sensors.py --headless --transient-obstacle
```

采集结束后停止两个仿真进程。离线环境运行以下命令。

```bash
python scripts/check_dufomap_sim.py
python scripts/dufomap_clean.py /tmp/g1-dufo-scans /tmp/g1-dufo-clean.pcd
python scripts/export_g1_map.py /tmp/g1-dufo-clean.pcd \
  --out /tmp/g1-dufo-map --known-free-radius 0
```

每个 NPZ 文件名是扫描时间戳（纳秒）。`points` 是 `camera_init` 中的
XYZ，`pose` 是同坐标系下的雷达起点和单位四元数。调用使用世界坐标点，
射线起点由 FAST-LIO 机体姿态旋转固定外参平移得到。数据从仿真配置的固定
外参读取后通过命令显式传入。输出 PCD 保留逐帧点的密度和重复观测。

## 合成射线验证

| 工况 | 障碍点清除率 | 墙面点保留率 |
| --- | ---: | ---: |
| 转向，障碍持续占用 | 0.00% | 100.00% |
| 障碍消失，换位后窄视场回扫 | 约 42.4% | 100.00% |
| 障碍消失，换位后扩大回扫视场 | 99.89% | 100.00% |

合成测试用单墙、箱体、已知射线起点，量化覆盖差异。宽视场工况是仿真假设，
实际采集可通过多视角回扫增加覆盖。

## MuJoCo 到地图导出的结果

4322 本轮目录为 `~/ldy/dufomap-sim.WWmdzT`，使用既有 Humble FAST-LIO 二进制。
基线和临时箱体工况各采到 150 对同时间戳的扫描和里程计。基线保留全部
403,227 点。临时箱体工况输入 403,227 点，输出 397,777 点。以
`camera_init` 中的 XYZ 范围 `[1.55, -0.45, -0.75]–[2.45, 0.45, 1.0]`
检查箱体附近，区域内点数从 9,044 降到 3,594，区域外 394,183 点全部保留。
区域统计包含框内其他回波，不能当作带语义真值的箱体召回率。
`dynamic-exported/` 已成功生成 PLY、PGM 和 YAML。

该 MuJoCo 工况是固定支撑位置下撤走临时箱体，覆盖范围与相对转向场景的关系
由上面的合成射线试验验证。真实吊架、真机采集、PC2/Foxy 以及清理后独立重定位
尚未验证。

## 随机障碍闭环复扫和 RViz 验收

`scripts/dufomap_loop_sim.py` 让 MuJoCo 的支撑式 G1 传感器沿闭合轨迹走两圈，
首圈在四个区域内随机选取箱体位置并分时刷新，次圈箱体退场后复扫。采集和
FAST-LIO 处理真实仿真的 LiDAR/IMU 消息。箱体位置仅写入验收文件，清理脚本
只读取点云、位姿和清场复扫的时间边界。闭环采集采用支撑轨迹，RViz 验收采用
行走控制器和独立重定位。

第一终端启动 FAST-LIO，第二终端将采集参数改为 `--frames 610 --timeout 90`，
第三终端运行以下命令，每个种子用独立目录。

```bash
python scripts/dufomap_loop_sim.py "$RUN/truth.json" --seed 4322 --speed 1.5 \
  | tee "$RUN/sim.log"
stamp=$(sed -n 's/^CLEAR_PASS_NS=//p' "$RUN/sim.log")
python scripts/dufomap_clean.py "$RUN/scans" "$RUN/clean.pcd" --revisit-after "$stamp"
python scripts/check_dufomap_loop.py "$RUN" --revisit-after "$stamp"
```

`--revisit-after` 启用第二道检查。DUFOMap 按自由空间证据剔除动态点后，首圈保留点
还需要与复扫点在 0.15 m 内相互印证。验收脚本把只用复扫得到的二维图作为参照，
区分临时箱体的假占用和邻近固定障碍投影膨胀出的占用。保持默认的 10 cm 栅格、
障碍高度带和零起点填补半径。

| 随机种子 | 箱体范围内的三维障碍高度带回波 | 箱体附近二维占用 | 清理后多出的二维占用 |
| --- | ---: | ---: | ---: |
| 4322 | 56,835 → 0 | 144 → 0 | 0 |
| 9417 | 55,573 → 0 | 152 → 7 | 0 |

9417 剩余的 7 个占用格同样出现在复扫参照图，是相邻固定障碍投影的占用。
两轮在障碍高度带以下仍分别留有 210、218 个近地回波。本结论覆盖两次
随机场景和一次无临时障碍的复扫流程，依赖复扫覆盖和 FAST-LIO 配准质量。

交互验证使用首个种子的 `clean-map/map.ply` 与 `clean-map/map.yaml`，
按正常顺序启动行走仿真、Open3D 和 Nav2。走行仿真的障碍层使用
`/livox/lidar` 的 `PointCloud2`，坐标为已有 TF 的 `lidar_link`。
只在此次仿真通过 `params_file:=` 传入该覆盖，避免使用缺少动态 TF 的
`body` 扫描作为代价地图输入。原会话的 RViz 点击能送达 BT Navigator，
其失败日志来自地图规划。新图上的首次实际 RViz 点击由约 (0.37, -0.21)
发往 (2.01, -0.24)，收到 `Goal succeeded`，实际移动约 1.44 m。
