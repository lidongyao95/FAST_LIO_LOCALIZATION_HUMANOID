# G1 真机 SLAM 文档

当前实机以 PC2 原生 Foxy 为部署基线。现场操作从 [PC2 单机与 B300 指南](g1_real_robot_slam_pc2_b300_guide.md) 开始，实测证据和历史变更见 [调试记录](debug_log/README.md)。[通用部署笔记](g1_real_robot_slam_notes.md) 保留外部 x86/Humble 路线，两份修改计划保留评审背景。

## 真机验证状态（2026-11-04）

| 项目 | 结论与范围 |
|---|---|
| SSH、PC2 与构建 | Mac 公钥登录 `unitree@192.168.123.164` 可用。Jetson Orin NX / ARM64 / Ubuntu 20.04.6 / Foxy，已复核 Livox 驱动、FAST-LIO 安装与动态库。 |
| 雷达与 IMU | 实时采集有成功记录。已重读 `~/g1_real_data/bags/static_01`，11.524 秒内点云 115 条（9.98 Hz）、IMU 2306 条（200.1 Hz）。两路消息头在包内各自严格递增。 |
| FAST-LIO 与 PCD | 静止实时里程计有成功记录。已在隔离 ROS 域离线回放静止包，`/map_save` 成功，临时 PCD 文件头为 10,661 点。 |
| 时间 | 先前 PC2 比 NTP 慢约 680 秒，换电重启后显示已同步，本轮偏差约 0.6–0.7 ms。动态传感器对齐与个别时间异常仍需验证。 |
| 硬件与运控 | OTA 包名标识 G1 Edu+，LowState 观测支持 29 DOF。`LocoClient` 只读查询有成功记录。整机/运控固件号、序列号、雷达安装方向及腰部锁定状态待现场确认。 |
| B300 建图 | 尚无正式 A 段录包和地图验收。需先核对安装外参、时间及现场采集条件。 |
| 重定位与导航 | 当前 PC2 overlay 无 `open3d_loc` 和 `nav2_bringup`。独立 B 段重定位、持续负载、TF、速度桥、停止与接管尚未验收。 |

Ubuntu 本地回放与实时查看见 [部署指南附录 A.1](g1_real_robot_slam_pc2_b300_guide.md#a1-在-ubuntu-本地查看现有录包)。

本轮复核细节见 [换电前后对时记录](debug_log/2026-11-04_g1_pc2_slam.md)。

## Ubuntu 离线回放与扫描场景仿真

- [倒装输入补偿与离线回放](inverted_mid360.md)
- [扫描场景的 MuJoCo 与 RViz 导航](scan_mujoco_preview.md)
- [2026-11-04 本地仿真调试记录](debug_log/2026-11-04_scan_navigation.md)

以上为本地离线/仿真流程，不改变上表真机验收状态。
