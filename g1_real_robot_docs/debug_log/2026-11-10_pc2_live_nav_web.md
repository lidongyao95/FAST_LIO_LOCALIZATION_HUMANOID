# 2026-11-10 PC2 实机网页与静止自动对齐

## 当天结果

- PC2 MID‑360 实测点云 9.99 Hz、IMU 199.98 Hz，8 秒抽样无非递增时间戳。
- 主底图为 4322 的 `~/ldy/hang-a02-filter-comparison/plaster-map/map.pgm`，与 `clean.pcd` 配套，来自吊着走一圈的 `hang_A_02`。PC2 副本在 `~/ldy/g1_real_data/maps/hang_A_02_plaster_preview/`。石膏版参数沿用生成脚本默认值，包括 1.5 m 近距、近地面豁免、0.6 m/s² IMU 门限、关闭时间抽样和 DUFOMap 0.2 m / 1 体素。每 2 秒抽样的 `combined` 是对照图。
- 轻量网页由 HTML Canvas 和 Python/rclpy 后端组成，经 SSH 回环转发。订阅 `/map`、TF 和 `/global_costmap/costmap_raw`，点目标调用 `ComputePathToPose`。静态对照用 18083，实时预览用 18085。Foxglove WebSocket 曾验证能收到点云、栅格和 TF，本轮现场使用轻量页。
- 原始 IMU 静止 Z 约 −1 g。FAST-LIO 已对点云和 IMU 启用 roll-180 补偿。初版的固定零航向和固定离地高度造成错位。15 帧抽样中，每帧中位数 177 个地面点落入障碍高度带。机器人下放后重新拟合地面，初次调平测得约 1.28 m 离地。
- 自动对齐在调平后每 4° 搜索航向，使用 FFT 栅格相关搜索平移，再用三维最近邻做平面 ICP 式精配准。首次拟合在原会话约 97% 重叠，但重启 FAST-LIO 后复用该变换仅剩 7%，实时叠加因此下线。
- 修正后采集、计算、发布和复测都在同一个 FAST-LIO 会话内。12 帧用于拟合，三个子批次的最佳解一致。该会话的局部原点在底图约 `(-0.029, 0.084) m`，匹配航向约 43.16°，离地调平偏移约 1.255 m。上述值绑定当时的局部坐标系。
- TF 生效后用另 12 帧验证，结构点 20 cm 内重叠率 97.5%，内点 RMS 3.7 cm。实时 costmap 版本持续更新，抽样较静态图有 2,489 格变化，其中 58 格新增致命障碍。该计数不等同于独立障碍物数量。
- 仅验收静止实时叠图。网页箭头是静止显示用的近似位姿，机体外参、运动跟踪、实机目标执行、速度桥、停止与接管仍待验收。

复跑见[实机逐步运行手册](../g1_pc2_live_nav_web_runbook.md)。仿真沿用原有入口及配置。

## 无线接入、连续定位与网页分层

### Mac 与 PC2 的连接

1. PC2 的 `wlan0` 已接入 `sic-guest`。Mac 接入同一 SSID，凭据按现场约定输入。
2. 本次 DHCP 地址为 Mac `172.18.28.93`、PC2 `172.18.17.116`，同属 `172.18.16.0/20`。换网或重连后重新核对地址。Mac 原先在另一子网，去 PC2 的流量走 `utun4`，SSH 失败。
3. Mac 网线拔掉后，无线 SSH 和 PC2 到 MID-360 `192.168.123.120` 的连通性均实测通过。Mac 外网 HTTPS 也实测可用。保留 PC2 到机器人和雷达的网线。
4. 使用既有有线主机密钥校验无线地址。在 Mac 打开网页转发：

   ```bash
   ssh -o HostKeyAlias=192.168.123.164 -o StrictHostKeyChecking=yes \
     -o ExitOnForwardFailure=yes -N -L 127.0.0.1:18085:127.0.0.1:18085 \
     unitree@172.18.17.116
   ```

   打开 [PC2 网页预览](http://127.0.0.1:18085/)。定位、代价地图和规划计算在 PC2 进行。

### 测试记录

| 项目 | 结果 |
| --- | --- |
| 同点冷启动 | 雷达持续运行，FAST-LIO 独立启动三次。独立新帧重叠率 94.5%、98.6%、98.6%，内点 RMS 3.7、3.5、3.4 cm。位姿估计最大相差 2.3 cm、0.52°。 |
| 无线静止对齐 | 拔掉 Mac 网线后复验 98.6%、3.4 cm。换位重新对齐至约 (4.09, 2.41) m、95.2°，新帧复验 89.8%、5.1 cm。 |
| 连续里程计网页 | 保持同一 FAST-LIO 进程，换位后箭头自动更新约 5.3 m。复验 80.5%、6.1 cm，低于 85% 重叠门槛。现场目视结构贴合，新增障碍是否解释低分未独立量化。 |
| 真机仅规划 | 空闲目标 (6.25, 6.75) m 返回 1.38 m 路径；封闭目标 (4.35, 7.65) m 中止。 |
| 放入和移走纸箱 | 同一目标路径 11.43 → 11.99 → 11.41 m。放箱时估计航向改变约 64°；移走前后约 5 mm、0.01°，266 个新增致命格中有 132 个退出致命状态。未标记纸箱对应格，未单独归因。 |
| 位姿稳定性与网页 | 前 56 个有效样本最大偏移 1.45 cm、0.17°，末段 5 次网页请求超时。对照复测 Mac 4/61 超时，PC2 本机 HTTP 61/61 成功；TF 发现后 58/58 成功且时间戳更新。后续正式静止漂移复测和往返测试已跳过。 |
| Open3D 静止粗测 | 首轮实际读到零初值，未通过。将初始地图对齐写入独立 YAML 后，两次初始化评分均约 0.787；停止前 32 次稳态评估均高于 0.50。 |
| Open3D 接网页并换位 | 先旁路验证 Open3D，再把地图链切成 `map → odom → camera_init → body`。同一 FAST-LIO 会话内估计位移约 9.5 m，换位后抽查 16 次配准均超过 0.50（0.687–0.743）。网页与代价地图持续更新。 |
| 网页雷达层 | 实机接口验证返回 646 个 `map` 系点。紫色显示实时障碍点，红橙色保留给全局规划代价。 |

### 代码与复现说明

- [网页后端](../../scripts/real_robot/web/server.py) 新增 `--robot-frame`，默认保留 `base_footprint`。实机用 `body`。订阅 `/cloud_registered_body_1`，按点云时间查询地图 TF，以 0.4–8 m 和地图系高度 0.15–1.8 m 筛选，按 0.1 m 平面格抽稀，经 `/api/obstacles` 输出。数据超过 2 秒不再返回。
- [网页前端](../../scripts/real_robot/web/index.html) 新增独立紫色雷达图层开关。蓝色机器人、橙色目标、绿色规划路径和红橙色全局代价沿用原色。雷达点表示近期观测，代价地图表示规划器使用的融合栅格。
- [导航 launch](../../open3d_loc/launch/g1_navigation.launch.py) 在仅规划模式支持 `robot_frame:=body`，只覆盖全局代价地图的机体参考帧。启动步骤见[运行手册](../g1_pc2_live_nav_web_runbook.md)。
- 连续配准粗测使用原 PCD 的 XY，按 `z′ = 0.078349821x + 0.006797867y + 0.996902764z + 1.347228408` 转换 Z，使用同一 FAST-LIO 会话静止对齐结果作为 `initialpose`。Open3D 的欧拉角参数以度计，旋转组合为 Rx·Ry·Rz。诊断参数为 `threshold_fitness_init=0.50`、`threshold_fitness=0.50`、`loc_frequence=0.5`，点云地图话题重映射到 `/rough_loc/map_cloud`，避免与 Nav2 的 `/map` 冲突。
- Open3D 使用单位安装外参并关闭运动 TF 发布。切换时停掉旧的 `map → camera_init` 发布器，补上单位 `odom → camera_init`，由 Open3D 独占 `map → odom`。这部分是临时运行配置。绝对定位精度、真实外参与自主走行仍待验收。

提交前核对 Mac、PC2 和 4322 上三个改动的运行文件 SHA-256 一致。两端 Python 语法编译通过。PC2/Foxy 用实机点云验证紫色图层数据接口；4322/Humble 在隔离 ROS 域用合成 TF 和点云验证仅规划启动、代价地图、网页及点云坐标变换。本次代码改动无需重编原生 ROS 包。

## 重启复测与 Open3D 使用约定

PC2 重启后自动接回 `sic-guest`，`wlan0` 仍为 `172.18.17.116/20`。本次重启复用 DHCP 地址，后续仍以实际地址为准。

现场观察到接入 Open3D 后，网页上的点云叠图明显更清晰。FAST-LIO 提供去畸变点云和连续局部位姿。Open3D 把实时点云反复与已知三维地图配准，在通过评分门槛时更新 `map → odom`，可减小移动后累计的平移和航向偏差。网页和全局代价地图使用这条地图坐标链，墙面、门框等结构因此更可能贴合底图，点云叠图中的错位和重影也会减轻。这是现场目视观察与机制解释。原始雷达采样质量和改善幅度尚未用同一批点云做定量对照。

以后在已有三维地图的真机定位、实时网页和导航联调中，默认启用 Open3D 持续配准，并检查初始化、稳态评分及地图 TF 是否更新。纯 FAST-LIO 模式保留给建图、隔离诊断和对照实验。现阶段的单位外参用于功能粗测，正式验收使用实测安装外参与独立参考位姿。

## 晚场复跑补充

沿用静止对齐加仅规划的流程，补充以下判据，均已写入[运行手册](../g1_pc2_live_nav_web_runbook.md)。

| 项目 | 结果 |
| --- | --- |
| 无线直连 | `ping 172.18.17.116` 13 ms；`ssh unitree@ubuntu.local` 可解析并登录，mDNS 同时返回 `192.168.123.164` 与 `172.18.17.116`。 |
| 新地址主机键 | 无线地址首次登录报 `Host key verification failed`。`ssh-keyscan -t ed25519 172.18.17.116` 与有线那条公钥逐字节相同，确认同一台机器后再登记。 |
| 隧道端口与 Host 白名单 | 把本地端映射成 18086 时 `/` 和 `/api/state` 均 403 `Invalid host`；同一隧道加 `Host: 127.0.0.1:18085` 后返回完整状态，说明是服务端白名单只认 `127.0.0.1:18085` 和 `localhost:18085`，与网络无关。 |
| ROS CLI 假阴性 | `ros2 topic list` 一度只列出 `/parameter_events`、`/rosout`，`ros2 topic hz` 对 `/livox/lidar`、`/livox/imu`、`/Odometry` 全无输出，同 shell 的 `probe_topics.py` 却是 10.00 Hz 与 200.03 Hz。CLI 守护进程缓存和 QoS 不匹配都会造成假阴性，核对以 probe 脚本为准。 |
| 启动顺序错误 | 静态 TF 未发布时先起 Nav2，`global_costmap` 停在激活阶段，`ensure_g1_nav_lifecycle.py` 超时后整个 launch 以 `G1 Nav2 lifecycle startup failed (exit 1)` 退出，日志末尾向 `map_server`、`planner_server` 发 `SIGINT`。`map → camera_init` 生效后重启即出现 `All G1 Nav2 lifecycle nodes active`。 |
| 网页缓存 | 页面在数据就绪前打开会一直不显示雷达点，硬刷新后恢复。同一时间 `/api/obstacles` 连续两次返回 404、411 点，范围 x −5.8…7.4 m、y −7.5…7.2 m，判据指向浏览器侧。 |
| 本轮对齐数值 | 拟合结果 x −0.3896、y 0.3495、z 1.3054、yaw 2.2481、pitch −0.0520、roll 0.0151；最优候选 0.943、次优 0.607，20 cm 重叠 0.942，内点 RMS 4.8 cm；换 `map` 系复验 97.0%、3.4 cm。与上文同点复测量级一致。 |
| 仅规划闭环 | `POST /api/plan` 对 (1.0, 1.0) m 返回路径；`/api/state` 的 `pose`、`planner`、`map`、`costmap` 均有效，`costmap` 版本随轮询变化。 |

### SDK 会话二进制的构建

准备带动力测试时在 PC2 找不到 `g1_loco_zero_session`：在 `/home`、`/tmp`、`/opt`、`/usr/local/bin` 下只有仓库里的四个源码文件，没有任何可执行产物。上一轮的产物没有留在仓库或固定目录，记录中提到的 `/tmp/g1-three-end.*`、`/tmp/g1-dry-nav.*` 都已不存在（只剩 `/tmp/g1-static-lio.*`），所以无法复用。`~/unitree_sdk2/build/bin/g1_loco_client` 是官方示例，交互协议与桥要求的 stdin `velocity`/`stop` 行协议不同，不能顶替。

仓库没有构建入口，只能单独编译。要链接 unitree SDK 自带的 DDS，不能链 `-L/usr/local/lib`：那样运行时会撞上 ROS Foxy 自带的 CycloneDDS 0.7.0，报 `undefined symbol: ddsi_sertype_v0`（该符号从 0.10 才有）。用 SDK 仓库的 `thirdparty` 并把 rpath 指向它：

```bash
R=$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID
S=$HOME/unitree_sdk2
mkdir -p /tmp/g1-loco-build
g++ -O2 -std=c++17 -I$R/scripts -I$S/include -I$S/thirdparty/include/ddscxx \
  -o /tmp/g1-loco-build/g1_loco_zero_session $R/scripts/real_robot/g1_loco_zero_session.cpp \
  -L$S/lib/aarch64 -lunitree_sdk2 \
  -L$S/thirdparty/lib/aarch64 -lddscxx -lddsc -lpthread \
  -Wl,-rpath,$S/thirdparty/lib/aarch64
g++ -O2 -std=c++17 -I$S/include -I$S/thirdparty/include/ddscxx \
  -o /tmp/g1-loco-build/g1_loco_preflight $R/scripts/real_robot/g1_loco_preflight.cpp \
  -L$S/lib/aarch64 -lunitree_sdk2 \
  -L$S/thirdparty/lib/aarch64 -lddscxx -lddsc -lpthread \
  -Wl,-rpath,$S/thirdparty/lib/aarch64
```

即使这样编译，**由速度桥拉起**的会话仍会加载错库：桥进程继承了 ROS 的 `LD_LIBRARY_PATH`，其中的 `libddsc.so.0` 优先生效。用一个只设库路径的包装脚本作为 `zero_session_executable` 可以绕开：

```bash
printf '#!/bin/sh\nexec env LD_LIBRARY_PATH=%s/thirdparty/lib/aarch64 /tmp/g1-loco-build/g1_loco_zero_session "$@"\n' "$S" > /tmp/g1-loco-build/g1_loco_session.sh
chmod +x /tmp/g1-loco-build/g1_loco_session.sh
```

`/tmp` 下的产物重启即失，正式验收前应改放到仓库外的固定目录并记录 SHA-256。`scripts/real_robot/g1_loco_sender_session.hpp` 仓库里已有，但 PC2 侧内容较新，改动后记得反向同步回 Mac。

只读预检 `g1_loco_preflight eth0` 返回 `fsm_id_result=0 fsm_id=0 fsm_mode_result=0 fsm_mode=0`。查询成功但 `fsm_id` 不是 802，速度会话会立刻回 `control_revoked` 并拒绝发速，这一条可用于开跑前的模式核对。

### Open3D 行走入口的实机化修复

带动力测试要接 Open3D，才发现 [g1_walking_localization.launch.py](../../open3d_loc/launch/g1_walking_localization.launch.py) 只能跑 MuJoCo：内部无条件 include FAST-LIO（`g1_walking.yaml`、`use_sim_time:=true`），Open3D 节点、rviz、`g1_nav_tf.py` 三处时钟写死为仿真，`publish_motion_tf` 固定 false 因而不发布 `map → odom`；四条安装 TF 用 `--frame-id/--child-frame-id` 写法，Foxy 的 `static_transform_publisher` 直接以 `exited due to not having the right number of arguments` 退出（`odom → camera_init` 缺失，`global_localization_node` 随后因 `imu_link` 不存在而启动失败）。

改动集中在同一个 launch，默认值保持仿真行为：

- 新增 `use_sim_time`（默认 `true`）、`start_lio`（默认 `true`）、`lio_config_file`（默认 `g1_walking.yaml`）、`publish_motion_tf`（默认 `false`），并透传到 FAST-LIO 子 launch、Open3D 节点、rviz 和 `g1_nav_tf.py`。
- 四条安装 TF 改为位置式参数。Humble 会打印旧式参数弃用警告，Foxy 正常。

实机调用方式：`start_lio:=false use_sim_time:=false publish_motion_tf:=true map_ply:=<clean.pcd>`，再给实测 `initialpose`（x、y、z 为米，roll、pitch、yaw 为度）。

| 主机 | 结果 |
| --- | --- |
| PC2/G1 · Foxy · 域 223 | 停掉旧 `map → camera_init` 后按上述参数启动。首次配准 fitness 0.754 与 0.766，`/localization_3d_registration_ok` 采样 20/20 为 true，`map → odom` 持续更新（约 −0.36, 0.32, yaw 2.25），`odom → camera_init` 为单元变换，`map → body` 合成正确，网页位姿与代价地图版本恢复更新，仅规划仍在线。 |
| 4322/Humble | 四个新参数解析正确，Open3D 节点、四条静态 TF、`g1_nav_tf` 均启动，`odom → camera_init` 可查询。首轮缺 Open3D 运行时库，补齐搜索路径后节点可正常初始化，见下文。 |
| 三端一致性 | Mac、PC2、4322 上该文件 SHA-256 均为 `6ae8e72d5874b872353eb49258fa22699bd987ba77f33b0b4be75b9499780345`。 |

启动后现场读数为：FAST-LIO 局部里程计 (−0.765, −0.488) m、航向相对对齐时刻转过约 76°，与对齐时的原点不同；Open3D 在该偏差下仍稳定通过配准门槛，`map → body` 合成为 (0.48, 0.02) m。机体若实际未移动，这一偏差需要单独归因后再放行动力测试。

### 4322 上 Open3D 运行时库

在 4322/Humble 上启动 `global_localization_node` 时先报 `liblapacke.so.3: cannot open shared object file`，补 `anaconda3/envs/twin/lib` 后接着报 `liblzf.so.1`，看起来像系统缺包。

实际不是缺包，而是缺库搜索路径。4322 的 Open3D 不走系统安装，`build/open3d_loc/CMakeCache.txt` 里 `Open3D_DIR` 指向仓库内的本地前缀 `.deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D`；`scripts/env/setup_g1_local_deps.sh` 已经用 `apt-get download` + `dpkg-deb -x`（无需 root）把 `libopen3d0.14`、`libfilament1.9`、`liblapacke`、`liblzf1`、`libstb0`、`libtinyobjloader1`、`libtmglib3` 解包到同一目录，`.deps/debs` 下还留着这些 deb 缓存。缺的只是运行时把这个目录放进 `LD_LIBRARY_PATH`。

判据：`LD_LIBRARY_PATH=.deps/root/usr/lib/x86_64-linux-gnu ldd install/open3d_loc/lib/open3d_loc/global_localization_node | grep -c "not found"` 返回 0。

按此路径启动后，节点在域 189 正常初始化、加载 `hang_A_02.pcd` 并停在 `Waiting for Odometry_loc...`，与「无 LIO 输入」的预期一致。结论是 4322 不需要 sudo 安装任何系统包，只需要在启动前导出：

```bash
export LD_LIBRARY_PATH="$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID/.deps/root/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
```

这条提示已加进 `scripts/env/setup_g1_local_deps.sh` 的输出，避免下次又从 `liblapacke`、`liblzf` 逐个试。

### 首次带动力指令的结果

现场条件满足后（`fsm_id=802`）发了机头前方 0.3 m 的目标，链路走到「指令被机器人接受」，但机体未移动。

- 门禁：硬件模式要求 `control_permission_topic` 是独立的绝对话题，用 `/g1/hw_control_permission`；PC2 部署版第 68 行的 `Node.get_fully_qualified_name` 在 Foxy 不存在，会抛 `AttributeError`，改成 `get_namespace()` + `get_name()` 后通过。
- 使能：首轮返回「waiting for 2 seconds of continuous valid localization and graph」，第 2 次返回 `enabled for hardware`。
- 执行：桥输出 64 个样本、峰值 0.267 m/s。该峰值是 DWB 0.25 m/s² 与桥 0.3 m/s² 共同限幅的结果，0.3 m 行程内到不了 0.3 m/s。
- 未移动：`map → body` 全程只在 ±1.5 cm 噪声内抖动，最终位移 0.027 m；Nav2 因 15 秒内未完成 0.15 m 进度返回 `ABORTED`。SDK 对每条速度都回 `send_result=0`，说明是机体侧未执行，而非链路拒收。
- 中途停止：约 5 秒时桥报 `map TF stale` 自行停止。实测 `map → odom` 约 9 Hz，966 个采样中 12 个年龄超过 0.3 秒门限（最大 0.423 秒）。
- 收尾：`disable` 成功，`request_sdk_zero` 返回「无活动会话」，机体静止。

待确认：机体在该运控状态下是否需要额外的速度控制确认；以及把 Open3D 的 `loc_frequence` 提高或放宽桥的 0.3 秒陈旧门限，二者选一。

### 0.6 m 复测与真正的阻塞点

把目标从 0.3 m 改成前方 0.6 m（目标 (−0.073, −0.221)，起点 (0.483, 0.005)），结果与 0.3 m 相同：`candidate_velocity` 全程 0.300 m/s，桥输出只有 0 或 1 个样本，Nav2 仍以 `ABORTED` 结束。**距离不是原因**，问题在速度输出的持续性。

这一轮又暴露四个具体问题：

1. **桥的同步 I/O 会饿死自己的订阅。** 桥在 20 Hz 定时器回调里做一次 SDK 往返（`select` 等 220 ms），阻塞期间 scan、odometry、control 的回调都得不到执行，健康时间戳自然过期，于是报 `scan, odometry, map TF stale` 自停。实测 `/Odometry_loc` 与 `/cloud_registered_1` 本身都是 10 Hz 正常。
2. **spawn 后的第一次交换经常超时。** 新建的 DDS 参与者在首次 `GetFsmId` + `SetVelocity` 上耗时超过 220 ms，桥直接判 `SDK velocity unconfirmed` 并退出使能。重试使能偶尔能成功一次，但随即被第 1 条打断。
3. **`loc_frequence` 的语义是周期（秒）。** 改成 0.2 后 `map → odom` 从 2 Hz 提到 10 Hz（8 秒 80 次），最大年龄 0.301 秒，仍会偶尔踩到 0.3 秒门限。该参数已提升为 launch 参数，默认仍是 0.5。
4. **`initialpose` 是 `map → odom`，不是 `map → body`。** 代码里 `mat_odom2map_ = mat_initialpose_`。这一轮先用 `map → body` 当初值，配准 fitness 从 0.75 掉到 0.41、低于门槛；换回静止对齐的六个量后恢复到 0.75–0.77。

结论：0.3 m 与 0.6 m 都停在「桥无法持续输出速度」上，与目标距离无关。可选修法两条，都属安全代码改动，需评审后再动：

- 把 SDK 会话 I/O 移出定时器回调（独立线程 + 队列），保持 20 Hz 与 0.3 秒健康窗口不变；
- 或把 tick 降到 3–5 Hz，并把健康窗口放宽到 1 秒，同时接受命令频率下降。

### 桥线程化、网页 QoS 与重复 TF 的修正

按第一条修法完成改动后，同一份 0.6 m 目标的桥输出从 0–1 个样本变成 **297 个**、峰值 0.298 m/s（指令 0.300），使能第 1 次即成功，`scan/odometry/map TF stale` 与 `SDK velocity unconfirmed` 都不再出现；收尾只剩导航中止后正常的 `timed out: command`。机体仍未移动，位移 0.042 m，属噪声量级。

同轮排查另外三个问题：

- **网页点云 QoS 不匹配**：`/cloud_registered_body_1` 由 FAST-LIO 以 RELIABLE 发布，网页按 `qos_profile_sensor_data`（best-effort）订阅，实测 8 秒收到 0 帧，`/api/obstacles` 因此返回 `null`。改为默认可靠 QoS 后连续 12 次采样 485–525 点。
- **重复静态 TF**：反复重启定位 launch 用 `kill`（SIGTERM）而非 SIGINT，子进程不退出，PC2 上同时存在 3 组 `odom → camera_init` 等静态发布器，`map` 时有时无。已清理；并新增 `scripts/env/g1_cleanup_stale.sh`（支持 `--dry-run`，按 `ROS_DOMAIN_ID` 过滤），`walking_env.sh` 里以 `G1_CLEAN_STALE=1` 选择性触发。
- **RViz 参数名**：`g1_walking_localization.launch.py` 原来只声明 `show_rviz`，传 `rviz:=false` 既不报错也不生效，于是多起一个 RViz。统一改名为 `rviz` 并默认 `false`，与 `g1_navigation`、`g1_mujoco` 一致。

**地图 z 的更正**：PC2 所用 `clean.pcd` 的地面在 z ≈ −1.1（p50），点云坐标原点在相机高度，因此 Open3D 输出的 `odom2map` z = −0.06 是**正确**的，不存在此前猜测的 1.37 m 垂直漂移；扫描点里地面点占 53.6%，竖直方向本来就有充分约束。出错的是早前的静止对齐把 `z` 取了 +1.305（等于把地面搬到 0），以及网页按绝对 `map` z 做 0.15–1.8 m 过滤——现在改为相对机体的高度带后与地面位置自洽。

### SDK 最小速度探针

为把「SDK 调用方式」与「ROS/Nav2 链路」分开，绕开 ROS 直接驱动运控：`ChannelFactory::Init(0, "eth0")` → `LocoClient::Init()` → 只读 `GetFsmId` → 连续 20 次 `SetVelocity(0.3, 0, 0, 0.2)`（每 100 ms 一次，共 2 秒）→ `StopMove()`。

实测输出：

```text
fsm_result=0 fsm_id=802 mode=velocity
i=0 rc=0 ... i=19 rc=0
StopMove=0
```

位姿从 `(0.517, 0.024)` 变为 `(0.449, −0.018)`，位移约 **0.08 m**，方向与机头朝向（yaw ≈ −160°）一致。结论：**SDK 速度通路正常，指令被运控接受并执行**。

但 2 秒只走了 8 cm，按 0.3 m/s 应有约 0.6 m，等效速度约 0.04 m/s；这与 Nav2 闭环跑 14 秒只挪 4 cm 是同一量级。因此「机体不动」的原因在物理侧而非软件侧：吊架绷紧、脚掌没有真正承重，机器人只能小幅挪动。下次带动力测试前应放松吊架让脚掌落地，再复跑同一 0.6 m 目标。

探针源码已入库：`scripts/real_robot/g1_loco_velocity_probe.cpp`，支持 `velocity`（默认）、`move`、`fsm` 三种模式，编译命令写在文件头，链接参数与上文 SDK 会话一致；运行前不要 source ROS，否则会加载 Foxy 的 CycloneDDS 0.7 并报 `undefined symbol: ddsi_sertype_v0`。编译产物仍在 `/tmp/g1-loco-build/`，重启即失。
