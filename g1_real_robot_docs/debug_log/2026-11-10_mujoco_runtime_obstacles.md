# 2026-11-10 G1 仿真临时障碍与代价地图排查

## 添加临时障碍

本次用原始 MuJoCo 房间、三维定位地图和二维导航地图。测试人员通过服务选择障碍出现的时机，坐标和移动路线使用内置预设，同一组参数可复现。地图生成仍使用原来的房间几何，新增物体只进入 MuJoCo 场景。

保持仿真、控制器、定位、导航的启动顺序。在仓库根目录执行，四个终端都使用同一个 `ROS_DOMAIN_ID`。

```bash
export ROS_DOMAIN_ID=153
./scripts/start_walking_sim.sh --moving-obstacle --fixed-obstacles 3
```

随后分别启动 `start_g1_controller.sh`、`start_g1_localization.sh show_rviz:=false` 和 `start_g1_navigation.sh`，均在 `scripts/` 下。4322 上本次使用用户确认的 `DISPLAY=:0`，在该桌面核对两个窗口，并用 `nvidia-smi` 核对 RViz 和 MuJoCo 进程。

- 移动障碍为红色薄板，X 向厚 2 cm，Y 向宽 50 cm，高 1.5 m。它按仿真时间以 0.2 m/s 沿 Y 轴往返。未启用移动预设或自定义路线时，原有 `/g1/obstacle` 仍可放入静止方块。
- 固定障碍为红橙色方块，最多三个，`--fixed-obstacles N` 指定启动时放入预设中的前 N 个。激活后，几何体可以参与 MuJoCo 碰撞和雷达射线检测。
- [场景生成](../../scripts/g1_mujoco_sensors.py)创建几何体，[仿真节点](../../scripts/g1_walking_sim.py)管理放入、移除和移动。移动位置可从 `/g1/obstacle_pose` 读取。

```bash
export ROS_DOMAIN_ID=153
source /opt/ros/humble/setup.bash
ros2 service call /g1/obstacle std_srvs/srv/SetBool "{data: true}"
ros2 service call /g1/fixed_obstacle_1 std_srvs/srv/SetBool "{data: true}"
```

将 `true` 改为 `false` 即可移除，其他固定障碍用 `/g1/fixed_obstacle_2` 和 `/g1/fixed_obstacle_3`。重启仿真后控制器从 Passive 启动。发目标前执行 `./scripts/g1_operator.sh prepare` 和 `./scripts/g1_operator.sh enable-nav`，确认 `controller_mode=3`、`released=true`、`nav_ready=true`，再用 RViz 的 **Nav2 Goal**。

## 为什么没有绕开

MuJoCo 射线测试分别击中了三个固定障碍，但修复前读取全局和局部代价地图，障碍附近的采样格仍为可通行。Nav2 日志反复出现 `body` 点云因 TF 时间戳被丢弃。进一步用实际收到的 `/cloud_registered_body_1` 消息做同时间戳 TF 查询，发现监听器中没有 `body` 帧。

FAST-LIO 把机体系点云标为 `body`，而行走仿真的 [FAST-LIO 配置](../../FAST_LIO/config/g1_walking.yaml#L51)原先设了 `publish.tf_en: false`。这关闭了 FAST-LIO 的 `camera_init → body` 发布，导致 Nav2 无法把点云转换到全局 `map` 或局部 `odom`。修复是在这份仿真配置中将 `tf_en` 改为 `true`，然后重启加载该配置的仿真栈。[Nav2 障碍层配置](../../navigation/g1_nav2_params.yaml#L158)继续使用原来的点云话题。

## 4322 验证范围

修复后，以点云消息的时间戳查询 `body → odom` 和 `body → map` 均成功。三个固定障碍的全局代价地图对照如下，数字是各障碍邻域中值为 100 的格数。

| 操作 | 障碍 1 | 障碍 2 | 障碍 3 |
| --- | ---: | ---: | ---: |
| 放入 | 15 | 13 | 9 |
| 移除，约 5 秒后采样 | 5 | 8 | 1 |
| 再放入 | 15 | 13 | 9 |

标记恢复，移除后仍有占用残留。起点处三个固定障碍超出局部滚动窗口，该对照只验收了全局障碍标记和部分清除。修复前的移动障碍试验最终超时取消，且未记录接触事件，不能据此判定主动绕行。修复后的固定障碍绕行、移动障碍避让和堵路停车仍待端到端验收。验收时应同时记录全局与局部占用、计划路径、速度和 MuJoCo 接触，并用无障碍的同一目标做对照。
