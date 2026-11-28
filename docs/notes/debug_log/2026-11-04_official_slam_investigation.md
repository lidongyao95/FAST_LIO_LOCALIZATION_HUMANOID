# 2026-11-04 官方 SLAM 栈历史实测记录

从已停用的官方栈操作指南保留的现场观测与静止试验原文。当前项目使用 FAST-LIO 路线；以下记录只描述当日设备和试验条件，不作为当前操作步骤。

## 0. 先看实测结论

2026-11-04 通过 PC2 上的 `unitree_sdk2` 客户端做了分阶段核查。第一阶段（重启 `lidar_driver` 之前）的结果如下：

```text
RobotStateClient::ServiceList  ret=0  size=28
  lidar_driver   status=0  protect=0
  unitree_slam   status=0  protect=0
  emergency_stop status=0  protect=0
  motion_switcher status=0 protect=0
  ...

rt/slam_info      6 秒收到 32 条（约 5.3 Hz）
  首条 type=ctrl_info, errorCode=0, info="not init"
  stateMachine.state="ready", ctrName="not init", isOpenPlan=false
  obsInfo.state=false, progress.completion_percentage=0.0

rt/lowstate       3 秒收到 3107 条（约 1 kHz）
rt/utlidar/cloud_livox_mid360   6 秒 0 条
rt/utlidar/imu_livox_mid360     6 秒 0 条
rt/unitree/slam_mapping/odom    6 秒 0 条
rt/unitree/slam_relocation/odom 6 秒 0 条
```

可以确定：

1. **官方 `unitree_slam` 正在运行**，按约 5 Hz 发布 `ctrl_info`；初始状态是 `ready / not init`。
2. **本次遇到并解决了一个真实问题：`lidar_driver` 没有出流。** 初始时点云/IMU 话题 0 条，`1801` 返回 `501 Lack of lidar or imu data`；通过 `ServiceSwitch("lidar_driver", 0)` → `1` 重启后恢复：点云约 10 Hz（每帧 20064 点），IMU 约 200 Hz。
3. **建图已验证（静止）**：`1801` 返回成功，`rt/unitree/slam_mapping/points` 与 `odom` 约 10 Hz；`1802` 保存成功，PCD 路径为 `/home/unitree/slam_probe_20261104.pcd`。
4. **重定位已验证（静止）**：`1804` 返回成功，`rt/unitree/slam_relocation/points` 与 `odom` 约 10 Hz，`pos_info` 约 10 Hz；`odom` 的 `frame_id=map`、`child_frame_id=base_link`。
5. **PC2 本地的 `mscli` 只显示 2 个本地服务**，不能代表整机服务；整机服务列表要通过 `RobotStateClient` 或 App 查看。根据“整机有 slam、PC2 本地没有 slam”可判断官方 SLAM 运行在 PC1。
6. `1802` 使用 `/tmp` 路径会返回 `errorCode=12, "The directory is illegal."`；保存路径必须放在 `/home/unitree/` 等允许目录。
7. 全流程没有发送运动命令；测试结束后已重启 `unitree_slam`，恢复到 `ready / not init`。

下面按由浅入深的顺序说明怎么用。

## 6. 现场实测记录（2026-11-04）

### 6.1 做了什么

在 PC2 上用 `/usr/local/lib/libunitree_sdk2.a` 编译并运行了以下程序，全部不涉及运动：

1. `ServiceList` / `ServiceSwitch`：查询、重启服务；
2. `slam_info` / `lowstate` 订阅：确认 DDS 通信；
3. 第一次建图尝试：`1801` + `1802`，保存路径用 `/tmp`；
4. 重启 `lidar_driver`，确认点云与 IMU 恢复；
5. 第二次建图：`1801` 建图 8 秒，`1802` 保存到 `/home/unitree/`；
6. 重定位：用刚保存的 PCD 调 `1804`，观察 10 秒；
7. 重启 `unitree_slam`，恢复到初始空闲状态。

### 6.2 分阶段结果

第一阶段：`lidar_driver` 未出流。

```text
ServiceList size=28
  lidar_driver status=0
  unitree_slam status=0

rt/utlidar/cloud_livox_mid360   0 条
rt/utlidar/imu_livox_mid360     0 条
DDS domain 0~5                  0 条

1801 → ret=1
  {"succeed":false,"errorCode":501,"info":"Lack of lidar or imu data.","data":{}}

1802 → ret=1
  {"succeed":false,"errorCode":12,"info":"The directory is illegal.","data":{}}
```

第二阶段：重启 `lidar_driver` 后恢复。

```text
stop  lidar_driver → ret=0 status=1
start lidar_driver → ret=0 status=0

rt/utlidar/cloud_livox_mid360   6 秒 59 条   每帧 width=20064
rt/utlidar/imu_livox_mid360     6 秒 1189 条
```

第三阶段：建图。

```text
1801 → ret=0
  {"succeed":true,"errorCode":0,"info":"Successfully started mapping.","data":{}}

建图 8 秒：
  slam_info=126
  cloud=83
  imu=1685
  slam_mapping/points=80
  slam_mapping/odom=80

mapping_info 示例：
  currentPose ≈ (x=-0.0012, y=-0.0036, z=0.0009)
  四元数 ≈ (q_w=0.99983, q_x=0.00011, q_y=-0.01814, q_z=-0.00255)

1802 → ret=0
  {"succeed":true,"errorCode":0,"info":"Save pcd successfully.","data":{}}

保存路径：
  /home/unitree/slam_probe_20261104.pcd


结束后回到：
  type=ctrl_info, state="ready", info="not init"
```

第四阶段：重定位。

```text
1804 → ret=0
  {"succeed":true,"errorCode":0,"info":"Successfully started re-location.","data":{}}

10 秒内：
  slam_relocation/points=97
  slam_relocation/odom=97
  slam_relocation/global_map=1（只发一次）

6 秒内 slam_info 类型统计：
  ctrl_info=30, pos_info=60, robot_data=3

pos_info 示例：
  {"type":"pos_info","errorCode":0,"data":{
    "currentPose":{"x":0.0064,"y":-0.0265,"z":0.0457,
                   "q_w":0.99982,"q_x":-0.00095,
                   "q_y":-0.01869,"q_z":-0.00179},
    "pcdName":"slam_probe_20261104",
    "address":"/home/unitree/slam_probe_20261104.pcd"}}

relocation odom：
  frame_id=map
  child_frame_id=base_link
  x=0.0064 y=-0.0265 z=0.0457
```

第五阶段：恢复。

```text
stop  unitree_slam → ret=0 status=1
start unitree_slam → ret=0 status=0

恢复后：
  type=ctrl_info, state="ready", info="not init"
  cloud 仍正常（约 10 Hz）
```

### 6.3 解释

- `ServiceState.status` 的实测语义：`0` 运行，`1` 停止。`lidar_driver` 从 `status=1` 切到 `status=0` 后点云恢复。
- 第一次 `501` 的根因是 `lidar_driver` 没有真正出流；重启后点云和 IMU 正常，`1801` 就能成功。
- 建图链路、`mapping_info`、保存 PCD 都已验证；但这次是**静止建图**，PCD 只包含当前视野，不能当作可用环境地图。
- 重定位链路已验证：`1804` 能加载刚保存的 PCD，`pos_info` 以约 10 Hz 输出，`odom` 的坐标系是 `map → base_link`，位姿在原点附近。
- `1802` 的 `12` 只是路径限制；换成 `/home/unitree/` 后保存成功。
- 本次在 PC1 的 `/home/unitree/` 下留下了探针地图 `slam_probe_20261104.pcd`，不需要时可以手动删除。

### 6.4 尚未验证的内容

以下都必须让机器人实际移动，本次没有执行：

- 行走建图的地图质量、重影与回环；
- 大范围场景（官方建议不超过 45 m × 45 m）；
- `1102` 是否绕障；
- `dog_mode` 改成 1 或 3 后的真实行为；
- 到点误差、停止反应、遥控接管与暂停恢复；
- App 导航与 `1102` 的行为差异。

其中“是否绕障”是下一步最关键的测试：在直线路径上放一个高于 50 cm 的箱子，分别观察 App 和 `1102`。

