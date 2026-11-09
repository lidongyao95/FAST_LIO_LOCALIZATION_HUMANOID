# G1 真机 SLAM 文档

当前实机以 PC2 原生 Foxy 为部署基线。现场操作从 [PC2 单机与 B300 指南](g1_real_robot_slam_pc2_b300_guide.md) 开始，实测证据和历史变更见 [调试记录](debug_log/README.md)。[通用部署笔记](g1_real_robot_slam_notes.md) 保留外部 x86/Humble 路线。

## 真机验证状态（2026-11-04，第一台 G1）

下表对应第一台 G1：雷达 MAC `8c:58:23:77:d1:b7`，部署在 `~/FAST_LIO_LOCALIZATION_HUMANOID`。

| 项目 | 结论与范围 |
|---|---|
| SSH、PC2 与构建 | Mac 公钥登录 `unitree@192.168.123.164` 可用。Jetson Orin NX / ARM64 / Ubuntu 20.04.6 / Foxy，已复核 Livox 驱动、FAST-LIO 安装与动态库。 |
| 雷达与 IMU | 实时采集有成功记录。已重读 `~/g1_real_data/bags/static_01`，11.524 秒内点云 115 条（9.98 Hz）、IMU 2306 条（200.1 Hz）。两路消息头在包内各自严格递增。 |
| FAST-LIO 与 PCD | 静止实时里程计有成功记录。已在隔离 ROS 域离线回放静止包，`/map_save` 成功，临时 PCD 文件头为 10,661 点。 |
| 时间 | 先前 PC2 比 NTP 慢约 680 秒，换电重启后显示已同步，本轮偏差约 0.6–0.7 ms。动态传感器对齐与个别时间异常仍需验证。 |
| 硬件与运控 | OTA 包名标识 G1 Edu+，LowState 观测支持 29 DOF。`LocoClient` 只读查询有成功记录。整机/运控固件号、序列号、雷达安装方向及腰部锁定状态待现场确认。 |
| B300 建图 | 尚无正式 A 段录包和地图验收。需先核对安装外参、时间及现场采集条件。 |
| 重定位与导航 | 当前 PC2 overlay 无 `open3d_loc` 和 `nav2_bringup`。独立 B 段重定位、持续负载、TF、速度桥、停止与接管尚未验收。 |

## 真机验证状态（2026-11-05，第二台 G1）

第二台 G1：雷达 MAC `8c:58:23:c0:0c:b6`，雷达序列号 `2021370048`，部署在 `~/ldy/FAST_LIO_LOCALIZATION_HUMANOID`，数据在 `~/ldy/g1_real_data`。与上表不是同一台整机，编号与结论须分开登记。

| 项目 | 结论与范围 |
|---|---|
| SSH、PC2 与构建 | 公钥登录可用，aarch64 / Ubuntu 20.04.6 / Foxy。`livox_ros_driver2`、`fast_lio`、`open3d_loc` 三包构建通过；Open3D 用作者 ARM 预编译包。 |
| 雷达与 IMU | 驱动 1.2.8 启动成功并连上 `192.168.123.120`。`~/ldy/g1_real_data/bags/static_01` 为 13.408 秒，点云 134 条（9.99 Hz）、IMU 2683 条（200.1 Hz）；两路头时间严格递增，`timebase` 与 `header.stamp` 一致。加速度单位为 g，静止模长 1.0025、重力沿 -Z；静止角速度模长约 0.034 rad/s。**动态对齐未测。** |
| FAST-LIO 与 PCD | 已用 `static_01` 在隔离域离线运行：`IMU Initial Done` 后输出 125 条 `/Odometry_loc`（`camera_init → body`）与 125 帧 `/cloud_registered_1`，约 10 Hz；静止 12.4 秒内位移不超过 19.5 mm。`/map_save` 成功，PCD 13,827 点。动态精度未验。 |
| 时间 | `timedatectl` 显示已同步，chrony 源为运控电脑 `192.168.123.161`，偏差约 0.5 ms。 |
| 硬件与运控 | 本轮未接运控：未读 LowState、未核对固件与整机序列号。雷达安装方向、外参、腰部状态待现场确认。 |
| B300 建图 | B300 门口已录制静止基线和吊架移动数据，后者已完成离线候选地图和石膏预览。正式行走建图与实机地图验收仍待完成。 |
| 重定位与导航 | `open3d_loc` 的 ARM 预编译 Open3D 运行时冒烟通过：能加载 PCD 地图、发布 11 个话题、常驻 RSS 约 38.7 MiB，配准流程可执行。修复安装 TF 启动时序后，同一静止录包与地图的回放完成了初始化和稳态配准。**B 段正式重定位仍待验收**。C 段的规划入口 `mode:=plan`（只起 map_server、planner_server 与其内部 global costmap）已在 PC2/Foxy 与 4322/Humble 双端复测：参数读回、三个规划用例与障碍层标记/清除均通过，PC2 仍无 `nav2_bringup` 但该入口不需要它。现场实时验收（正式地图、实测外参、动态数据）未做；TF、速度桥、停止与接管未验。 |

本轮部署、排障和规划验证见 [第二台 G1 调试汇总](debug_log/2026-11-05_g1_pc2_debug_summary.md)。

Ubuntu 本地回放与实时查看见 [部署指南附录 A.1](g1_real_robot_slam_pc2_b300_guide.md#a1-在-ubuntu-本地查看现有录包)。

第一台 G1 的复核细节见 [换电前后对时记录](debug_log/2026-11-04_g1_pc2_slam.md)。

## Ubuntu 离线回放与扫描场景仿真

- [倒装输入补偿与离线回放](inverted_mid360.md)
- [扫描场景的 MuJoCo 与 RViz 导航](scan_mujoco_preview.md)
- [A 段离线地图质检与交付](g1_real_robot_slam_pc2_b300_guide.md#67-a-段离线地图质检与交付)
- [点云过滤、彩虹选参与石膏预览](hang_a02_point_cloud_filters.md)
- [4322 石膏版手动导航](scan_mujoco_preview.md#5-4322-石膏版手动导航)
- [2026-11-04 本地仿真调试记录](debug_log/2026-11-04_scan_navigation.md)

以上为本地离线/仿真流程，不改变上表真机验收状态。

## 已弃用方案

本项目**不使用 G1 官方 SLAM 栈**（整机 PC1 上的 `unitree_slam`、`slam_mapping`/`slam_relocation` 等），建图、定位与导航统一采用本仓库的 FAST-LIO 方案。以下文档仅作为历史调研与排障记录保留，内容不再维护，结论不代表当前方案：

- [G1 官方 SLAM 栈实战：平地建图与绕障导航](deprecated/g1_official_slam_obstacle_navigation.md)

