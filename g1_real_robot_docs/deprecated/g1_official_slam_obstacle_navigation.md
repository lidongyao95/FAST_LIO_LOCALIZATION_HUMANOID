# G1 官方 SLAM 栈实战：平地建图与绕障导航（由浅入深）

> **本方案已弃用。** 当前项目不使用 G1 官方 SLAM 栈（PC1 上的 `unitree_slam`、`slam_mapping`/`slam_relocation` 等），建图、定位与导航统一走本仓库的 FAST-LIO 路线。本文仅作为历史调研与排障记录保留，不再维护，结论不代表当前方案。

- 日期：2026-11-04
- 适用对象：G1 Edu+，整机 PC1 + 开发电脑 PC2（Jetson Orin NX / Ubuntu 20.04 / ROS 2 Foxy）
- 关联文档：[真机验证状态](../README.md)、[PC2 与 B300 指南](../g1_real_robot_slam_pc2_b300_guide.md)、[2026-11-04 调试记录](../debug_log/2026-11-04_g1_pc2_slam.md)
- 本文范围：只讲官方栈如何建图、定位和绕障。

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

## 1. 第一层：知道官方栈是什么

### 1.1 四个组成部分

| 组成                                | 作用                               | 使用者     |
| ----------------------------------- | ---------------------------------- | ---------- |
| `lidar_driver` 服务               | 驱动 MID-360，输出点云与 IMU       | 官方栈内部 |
| `unitree_slam` 服务               | 建图、重定位、拓扑导航、规划与控制 | 官方栈内部 |
| Unitree Explore App                 | 图形化建图、定位、选点导航         | 零代码用户 |
| `unitree_sdk2` + `slam_operate` | 用 api-id 调用建图/定位/导航       | 开发者     |

### 1.2 官方公开接口

```text
服务名：slam_operate
版本：1.0.0.1

1801  开始建图
1802  结束建图并保存 PCD
1804  初始化位姿 / 开始重定位
1102  位姿导航
1201  暂停导航
1202  恢复导航
1901  关闭 SLAM
```

### 1.3 能力边界

官方文档写得很明确：

- 静态、室内、平坦场景；
- 场景 X/Y 方向不超过 45 m；
- `1102` 单段目标不超过 10 m；
- 文档描述 `1102` 为直线运动，障碍物应高于 50 cm；
- 剧烈运动可能导致定位丢失；
- 不要同时使用 App 导航和 SDK 接口导航。

### 1.4 闭源情况

- 模块内部没有 C++ 源码，只有 launch、配置、消息定义和编译产物；
- `unitree_slam_example` 是客户端示例，不是模块实现；
- 可以调接口、可以改模块内部 yaml，但不能改算法，OTA 还可能覆盖修改。

## 2. 第二层：最快跑通（只用 App）

适合第一次上手，不写代码。

### 2.1 确认服务

App 路径：

```text
设备 → 配置信息 → 运维服务 → 服务状态
```

确认 `lidar_driver` 和 `unitree_slam` 已开启。若固件 1.5.3 之后默认关闭，在这里手动开启。

### 2.2 建图

1. 把机器人放在建图起点，记下位置和朝向。
2. 在 App 的地图/建图页面开始建图。
3. 遥控机器人低速走完目标区域，尽量多走一遍，最后回到起点附近。
4. 结束建图并保存地图。

官方 SDK 对应的动作是 `1801` 开始、`1802` 保存。保存路径建议在 PC1 上固定为：

```text
/home/unitree/test1.pcd
/home/unitree/test2.pcd
...
```

不要使用 `/tmp` 等目录：实测 `1802` 会返回 `errorCode=12, "The directory is illegal."`。

### 2.3 定位

1. 把机器人放回建图起点附近。
2. 在 App 中加载刚才的地图并初始化定位。
3. 确认 App 中的机器人位置与实际位置一致。

SDK 对应 `1804`，需要给初始位姿和 PCD 路径。

### 2.4 导航

1. 在 App 地图上选择目标点。
2. 下发导航，观察机器人是否到点。
3. 在直线路径上放一个高于 50 cm 的箱子，观察行为。

### 2.5 绕障设置

如果 App 导航走的是模块内部的拓扑导航链路，默认障碍策略是“停止”，因为模块内部默认：

```yaml
path_process/dog_mode : 2   # 1 重新规划 2 停止 3 绕开
```

要让它绕开，需要改模块内部配置（见第 5 层）。App 是否直接暴露这个选项，以实际版本为准。

### 2.6 本层验收

- 能保存并重新加载 PCD；
- 重定位后位置与实际一致；
- 能走到 App 设置的目标点；
- 记录默认遇障行为：停止、重规划还是绕开。

## 3. 第三层：用官方示例 keyDemo

适合在没有 App 或需要脚本化验证时使用。

### 3.1 获取与编译

```bash
# 下载 unitree_slam_example
# 解压后进入 example 目录
mkdir build && cd build
cmake ..
make
./keyDemo eth0
```

### 3.2 按键流程

```text
q  开始建图
w  结束建图
a  开始重定位
s  把当前位姿加入任务列表
d  执行任务列表
f  清空任务列表
z  暂停导航
x  恢复导航
```

典型操作：

1. `q` 开始建图，遥控走完区域并回到起点；
2. `w` 结束建图；
3. `a` 在起点重定位；
4. 遥控到目标点，按 `s` 记录；
5. 回到起点，按 `d` 执行任务列表。

### 3.3 必须知道的限制

已核对 keyDemo 源码：`d` 键会对任务列表逐点调用 `1102`，`mode=1`，等待 `task_result.is_arrived`。

所以：

- 它验证的是建图、重定位、逐点到达；
- **它不验证全局规划和绕障**；
- 遇到障碍时表现为停止，不代表官方栈没有绕障能力。

### 3.4 本层验收

- keyDemo 能完整跑通“建图 → 定位 → 到点”；
- `rt/slam_key_info` 能看到 `task_result`；
- 记录每段路径长度是否都 ≤ 10 m。

## 4. 第四层：用 SDK 自己调 api-id

适合接入自己的程序、批量采集数据、做自动化验收。

### 4.1 开启服务

使用 `unitree_sdk2` 的 `RobotStateClient`：

```cpp
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

unitree::robot::b2::RobotStateClient rs;
rs.SetTimeout(25.0f);
rs.Init();

int status = 0;
rs.ServiceSwitch("lidar_driver", 1, status);
usleep(200000);
rs.ServiceSwitch("unitree_slam", 1, status);
```

先查再开：

```cpp
std::vector<unitree::robot::b2::ServiceState> list;
rs.ServiceList(list);
for (auto& s : list)
    std::cout << s.name << " status=" << s.status << std::endl;
```

如果 `1801` 仍然返回 `501`，执行一次 `lidar_driver` 重启（`0` → `1`），再用订阅确认 `rt/utlidar/cloud_livox_mid360` 有数据。本次实测就是通过这一步恢复的。

### 4.2 建图

```json
{"data": {"slam_type": "indoor"}}
```

调用 `1801`。开始后遥控行走，速度不要太快。

结束并保存：

```json
{"data": {"address": "/home/unitree/test1.pcd"}}
```

调用 `1802`。

保存路径必须在允许的目录下，使用 `/home/unitree/`；实测 `/tmp` 会返回 `errorCode=12, "The directory is illegal."`。

### 4.3 定位

```json
{
  "data": {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "q_x": 0.0, "q_y": 0.0, "q_z": 0.0, "q_w": 1.0,
    "address": "/home/unitree/test1.pcd"
  }
}
```

调用 `1804`。初始位姿要与建图起点一致。

### 4.4 导航：如何告诉机器人目的地

目的地本质上是**地图坐标系下的一个位姿**，不是“走到某个房间”这种语义目标。官方接口只接受位置和朝向。

前提：

1. `lidar_driver` 已出流；
2. `1804` 已加载地图并完成重定位；
3. `pos_info` 正常，位置与实际一致；
4. 没有同时使用 App 导航。

#### 4.4.1 目的地坐标从哪里来

| 来源 | 说明 |
|---|---|
| App 地图选点 | App 自己换算成地图坐标，适合人工操作 |
| 当前位姿 | 订阅 `pos_info` 的 `currentPose` |
| 拓扑点 | keyDemo 按 `s` 记录当前位姿，按 `d` 逐点执行 |
| RViz / 地图工具 | 确认 Fixed Frame 是 `map` 后再取坐标 |

坐标系约定：地图原点通常是建图起点的 Mid-360 IMU 原点，X 轴朝机器人前方，Z 轴朝上，单位米。

#### 4.4.2 请求格式

```json
{
  "data": {
    "targetPose": {
      "x": 2.0,
      "y": 0.0,
      "z": 0.0,
      "q_x": 0.0,
      "q_y": 0.0,
      "q_z": 0.0,
      "q_w": 1.0
    },
    "mode": 1
  }
}
```

| 字段 | 含义 |
|---|---|
| `x/y/z` | 地图系下的位置，单位米；`z` 通常填 0 |
| `q_x/q_y/q_z/q_w` | 目标朝向的四元数 |
| `mode` | 固定填 1 |

调用 `1102`。

只关心偏航角时：

```text
q_x = 0
q_y = 0
q_z = sin(yaw / 2)
q_w = cos(yaw / 2)
```

例子：

```text
向前 2 米，朝向不变
{"data":{"targetPose":{"x":2.0,"y":0.0,"z":0.0,"q_x":0.0,"q_y":0.0,"q_z":0.0,"q_w":1.0},"mode":1}}

原地转向 90 度（yaw = π/2）
{"data":{"targetPose":{"x":0.0,"y":0.0,"z":0.0,"q_x":0.0,"q_y":0.0,"q_z":0.70711,"q_w":0.70711},"mode":1}}
```

#### 4.4.3 C++ 最小调用

```cpp
#include <cstdio>

class NavClient : public unitree::robot::Client
{
public:
    NavClient() : Client("slam_operate", false) {}

    void Init()
    {
        SetApiVersion("1.0.0.1");
        UT_ROBOT_CLIENT_REG_API_NO_PROI(1102);
    }

    int32_t GoTo(float x, float y, float z,
                 float qx, float qy, float qz, float qw,
                 std::string& out)
    {
        char buf[512];
        snprintf(buf, sizeof(buf),
            R"({"data":{"targetPose":{"x":%.3f,"y":%.3f,"z":%.3f,)"
            R"("q_x":%.5f,"q_y":%.5f,"q_z":%.5f,"q_w":%.5f},"mode":1}})",
            x, y, z, qx, qy, qz, qw);
        return Call(1102, buf, out);
    }
};
```

返回：

```json
{"succeed":true,"errorCode":0,"info":"","data":{}}
```

`succeed=true` 只代表请求被接受，不代表已到达。

#### 4.4.4 长距离与多段导航

官方文档限制单段不超过 10 米，且描述为直线运动。长距离目标要自己拆段：

1. 把目标拆成多个 ≤ 10 米的位姿点；
2. 发送第一段；
3. 等待 `task_result.is_arrived=true` 或 `ctrl_info.is_arrived=true`；
4. 再发送下一段；
5. 任一段失败就停止后续发送。

不要一次性连续下发多段，否则无法判断哪一段失败。

#### 4.4.5 到达、暂停与恢复

到达判断：

- `rt/slam_key_info` 的 `task_result.is_arrived`；
- `rt/slam_info` 的 `ctrl_info.is_arrived`。

暂停与恢复：

- `1201` 暂停导航；
- `1202` 恢复导航；
- `1901` 是关闭 SLAM，不是普通取消导航，不要拿它当停止按钮。

详细字段见 4.5。

#### 4.4.6 限制

- 单段 ≤ 10 米；
- 文档描述为直线运动，低于 50 cm 的障碍可能看不到；
- 是否绕障取决于模块内部 `dog_mode` 和实际链路，不能只靠 `1102` 参数控制；
- 目标位姿必须使用与重定位相同的地图坐标系。

### 4.5 接收状态

订阅两个话题：

```text
rt/slam_info
rt/slam_key_info
```

`ctrl_info` 的实测字段（本次抓到的真实数据，机器人未初始化）：

```json
{
  "type": "ctrl_info",
  "errorCode": 0,
  "info": "not init",
  "data": {
    "currentPose": {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},
    "targetPose":  {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},
    "startPose":   {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},
    "targetNodeName": 0,
    "is_arrived": false,
    "obsInfo": {"state": false, "time": 0.0},
    "progress": {"used_time":0.0,"last_time":0.0,"completion_percentage":0.0},
    "total_distance": -1.0,
    "stateMachine": {
      "state": "ready",
      "ctrName": "not init",
      "isOpenPlan": false,
      "isPause": false,
      "isBack": false,
      "isRotate": false,
      "isClimbStairs": false,
      "vx": 0.0077, "vy": 0.0018, "vyaw": 0.0004
    }
  }
}
```

要点：

- 姿态字段实测是 `roll/pitch/yaw`，不是四元数；文档其他位置的 `pos_info` 使用四元数，二者都要兼容。
- `isOpenPlan` 表示规划是否开启；
- `obsInfo.state` 表示是否检测到障碍；
- `stateMachine.state` 和 `ctrName` 可以用来判断当前处于 ready、follow 还是暂停。

`task_result` 示例：

```json
{
  "type": "task_result",
  "errorCode": 0,
  "data": {"targetNodeName": 9999, "is_arrived": false}
}
```

`pos_info` 示例（本次重定位实测）：

```json
{
  "type": "pos_info",
  "errorCode": 0,
  "data": {
    "currentPose": {
      "x": 0.0064, "y": -0.0265, "z": 0.0457,
      "q_w": 0.99982, "q_x": -0.00095,
      "q_y": -0.01869, "q_z": -0.00179
    },
    "pcdName": "slam_probe_20261104",
    "address": "/home/unitree/slam_probe_20261104.pcd"
  }
}
```

`pos_info.currentPose` 是四元数，可以直接作为下一段导航的起点参考；`ctrl_info` 里的位姿是欧拉角，二者不要混用。

### 4.6 本层验收

- 能独立完成 1801/1802/1804/1102；
- 能解析 `ctrl_info`、`pos_info` 和 `task_result`；
- 能正确区分“调用成功”和“实际到达”；
- 能把一个目标位置加偏航角转换成 `targetPose` 四元数。

## 5. 第五层：绕障到底怎么开（深入）

### 5.1 模块内部链路

从模块的包组成可以还原出这条链路：

```text
MID-360 点云 + IMU
  → lio_sam_ros2（里程计/重定位）
  → occ_grid_mapping（占据栅格）
  → map_server / merger_map（地图加载与合并）
  → nav2_planner_astar（A* 全局路径，/global_path）
  → local_planner / DWA（局部避障，local/cost_map）
  → 速度命令
```

关键证据：

- `nav2_planner_astar/config/config.yaml`：`map_topic: /merger_map`、`path_topic: /global_path`；
- `local_planner/configs/params.yaml`：`Planner/planner_name: dwa`、`occ_map_name_topic: local/cost_map`；
- `task/task/navigation_to_pose.py`：调用 `nav2_simple_commander` 的 `BasicNavigator.goToPose()`；
- `graph_process/config/graph_params.yaml`：`path_process/dog_mode` 三种障碍策略。

### 5.2 默认停障的开关

```yaml
path_process/dog_mode : 2
# 1：重新规划
# 2：停止
# 3：绕开
```

默认是 2。想绕开，需要在 PC1 上把该值改成 3（或改成 1 让它重规划），然后重启 `unitree_slam` 服务。

配置路径（以实际安装为准）：

```text
/unitree/module/graph_pid_ws/src/graph_process/config/graph_params.yaml
/unitree/module/graph_pid_ws/src/graph_process/config/graph_params_muiltfloor.yaml
/unitree/module/graph_pid_ws/src/graph_process/config/graph_params_muiltdog.yaml
```

### 5.3 相关参数

| 文件                                      | 参数                                     | 作用                         |
| ----------------------------------------- | ---------------------------------------- | ---------------------------- |
| `graph_params.yaml`                     | `path_process/dog_mode`                | 1 重规划 / 2 停止 / 3 绕开   |
| `local_planner/configs/params.yaml`     | `ObstaclesInsideDis`                   | 距目标多少米内只检测内圈障碍 |
| 同上                                      | `sleep_wait`                           | 障碍持续阻碍的等待时间       |
| 同上                                      | `Planner/max_vel_x`、`max_vel_theta` | DWA 速度上限                 |
| 同上                                      | `Planner/acc_lim_x`、`acc_lim_theta` | 加速度限制                   |
| `nav2_planner_astar/config/config.yaml` | `map_topic`、`path_topic`            | 全局地图与路径话题           |

注意：

- 这些参数带有机器狗命名和默认值，**不能直接照搬到 G1**；
- 修改后要重启 `unitree_slam`；
- OTA 可能覆盖配置，升级前备份；
- 这是非公开接口，官方不承诺兼容。

### 5.4 `1102` 到底会不会绕障

现在有两层证据，但还缺一个现场实测：

| 证据         | 说明                                                              |
| ------------ | ----------------------------------------------------------------- |
| 官方文档     | `1102` 写的是直线、≤ 10 m、障碍应高于 50 cm                    |
| keyDemo 源码 | 逐点调用 `1102`，不涉及 A* 或 `dog_mode`                      |
| 模块内部     | 有 A*、DWA、`dog_mode`，还有走 `nav_to_pose` 的 Nav2 任务节点 |

因此：

- **不能从公开文档断言 `1102` 会绕障**；
- **也不能因为 keyDemo 停障就断言整个官方栈不会绕障**；
- 必须在直线路径上放障碍，实测 `1102` 和 App 两种入口。

### 5.5 如何观测是否真的绕障

订阅 `rt/slam_info`，记录以下字段：

```text
stateMachine.state         ready / follow / ...
stateMachine.ctrName       控制器名称
stateMachine.isOpenPlan    是否开启规划
stateMachine.isPause       是否暂停
obsInfo.state              是否检测到障碍
obsInfo.time               障碍持续时间
progress.completion_percentage
targetNodeName
is_arrived
```

建议同时录屏或用 RViz 观察。官方文档给出的配置：

```text
/unitree/module/unitree_slam/rviz2/mapping.rviz
/unitree/module/unitree_slam/rviz2/relocation.rviz
```

实际路径可能因固件版本不同而变化，以安装目录为准。

### 5.6 本层验收

- 障碍物放在直线路径上，机器人不是简单停死；
- 能观察到重规划或绕行轨迹；
- `obsInfo.state` 与 `dog_mode` 的行为一致；
- App 与 `1102` 的行为差异有明确记录。

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

## 7. 验收清单与排障

### 7.1 建图验收

- 建图起点与终点接近，地图无明显重影；
- PCD 能在 `1804` 中重新加载；
- 场景范围不超过 45 m；
- 地面、墙角、门口与实际一致。

本次已验证：`1801/1802`、mapping 点云与 odom、PCD 保存。本次是静止建图，行走建图质量仍需单独验收。

### 7.2 定位验收

- 初始位姿正确；
- `ctrl_info` 的 `currentPose` 随移动连续变化；
- 不出现 `503`；
- 静止时位姿不漂移。

本次已验证：`1804` 能加载 PCD，`pos_info` 约 10 Hz，`odom` 为 `map → base_link`，静止位姿在原点附近。运动状态下的精度与漂移仍需单独验收。

### 7.3 绕障验收

| 项目     | 通过条件             |
| -------- | -------------------- |
| 默认行为 | 与 `dog_mode` 一致 |
| 改 3 后  | 能绕开静态障碍       |
| 改 1 后  | 能重新规划           |
| 动态障碍 | 能停车并恢复         |
| 人工接管 | 遥控器介入后导航暂停 |

### 7.4 常见错误码

| errorCode | 含义                | 排查                                                                              |
| --------- | ------------------- | --------------------------------------------------------------------------------- |
| 0         | 正常                | 无                                                                                |
| 501       | 缺少 IMU 或雷达数据 | 先重启 `lidar_driver`（0 → 1），确认点云/IMU 话题有数据；再看网段与雷达目的 IP |
| 503       | 定位漂移            | 初始位姿、地图质量、剧烈运动                                                      |
| 12        | 保存目录不合法      | 改用 `/home/unitree/` 下的路径                                                  |

### 7.5 安全要求

- 首次测试用开阔平地，障碍高于 50 cm；
- 现场有人持遥控器，明确停止职责；
- 不要在 App 导航时同时调 SDK 导航；
- 不要把 `succeed=true` 或 `is_arrived=true` 当作实际停止的证明。

## 附录 A：话题与 JSON

```text
rt/unitree/slam_mapping/points          PointCloud2
rt/unitree/slam_mapping/odom            Odometry
rt/unitree/slam_relocation/points       PointCloud2
rt/unitree/slam_relocation/odom         Odometry
rt/unitree/slam_relocation/global_map   PointCloud2
rt/slam_info                            String(JSON)
rt/slam_key_info                        String(JSON)
```

`rt/slam_info` 的 `type`：

- `robot_data`：温度、电量、CPU；
- `pos_info`：当前位姿（四元数）、PCD 名称与路径；
- `ctrl_info`：目标点、状态机、障碍、进度（实测位姿为欧拉角）。

`rt/slam_key_info` 的 `type`：

- `task_result`：`targetNodeName`、`is_arrived`。

## 附录 B：本次核查命令

```bash
# 在 PC2 上
ssh unitree@192.168.123.164

# 本地服务（只反映 PC2）
/unitree/sbin/mscli listservice

# 整机服务（推荐）
/tmp/query_slam_services eth0

# 订阅心跳
/tmp/probe_slam_stack eth0

# DDS 对照
/tmp/probe_lowstate eth0
```

## 参考

- [SLAM 导航服务接口（中文）](https://support.unitree.com/home/zh/G1_developer/slam_navigation_services_interface)
- [SLAM and Navigation Services Interface（英文）](https://support.unitree.com/home/en/G1_developer/slam_navigation_services_interface)
- [如何进行建图和导航](https://serviceconsole.unitree.com/mobile/index.html#/help/03060702)
- [G1 固件 1.5.3 后用程序拉起 lidar_driver 与 unitree_slam 服务](https://serviceconsole.unitree.com/mobile/index.html#/help/031361)
- [SLAM 服务接口的常见报错](https://serviceconsole.unitree.com/mobile/index.html#/help/0311c5)
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_slam_example 下载](https://oss-global-cdn.unitree.com/static/dd442abf94ec44f599095cb15c3e298b.zip)
