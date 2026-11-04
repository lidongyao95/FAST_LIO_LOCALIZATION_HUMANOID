# MID-360 倒装输入补偿

主分支 `main` 的 `livox_ros_driver2/config/MID360_config.json` 使用 roll=180，
`src/comm/pub_handler.cpp` 同时旋转点云与 IMU。本分支使用不同来源的驱动，
已录制的数据也不会重新经过驱动，因此在 FAST-LIO 增加显式输入补偿。

`common.input_roll_180` 默认 false。仅当点云和 IMU 都保留原始倒装传感器坐标时开启。
开启后绕 X 轴旋转 180°：点云、加速度和角速度均 `(x,y,z) -> (x,-y,-z)`，
两类 IMU 协方差同步转换；FAST-LIO 不使用 IMU orientation。
配置中的 LiDAR 到 IMU 外参仍填写原始坐标值，内部自动计算
`T'=C*T`、`R'=C*R*C^T`，其中 `C=diag(1,-1,-1)`。
不要再在驱动、bag 预处理或外参中重复加同一安装旋转。
此功能只定义传感器算法坐标，不代替传感器到机器人机体 TF 标定。

## 离线运行

先结束旧 FAST-LIO 和旧播放器，确保独立域 193 没有其他输入源。
每个终端都在仓库根目录执行 `source scripts/walking_env.sh` 和
`export ROS_DOMAIN_ID=193`。先创建输出目录，再分别运行：

```bash
mkdir -p tmp/g1_real_data/replay/records_inverted
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$PWD/FAST_LIO/config/mid360.yaml" \
  -p common.input_roll_180:=true \
  -p use_sim_time:=true \
  -p map_file_path:="$PWD/tmp/g1_real_data/replay/static_01_inverted.pcd" \
  -p debug_log_dir:="$PWD/tmp/g1_real_data/replay/records_inverted"
```

```bash
rviz2 -d FAST_LIO/rviz/fastlio.rviz --ros-args -p use_sim_time:=true
```

```bash
ros2 bag play tmp/g1_real_data/bags/static_01 --clock --delay 3
```

回放结束、算法仍运行时保存：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

Fixed Frame 使用 `camera_init`，点云为 `/cloud_registered_1`，地图为
`/Laser_map_1`，里程计为 `/Odometry_loc`，轨迹为 `/path_1`。
静止包只能核对翻转方向和静态稳定性；动态去畸变及机体轴向须另行采集验证。

## 2026-11-04 本机验证

- Humble 下重新编译 `fast_lio` 成功，只有已有 Boost bind 弃用提示。
- `static_01` 以 1 倍速、`--clock` 在 ROS_DOMAIN_ID=193 回放完成。
- 原始 IMU 加速度均值约 `[0.0161, 0.000762, -0.9954]`（录包数值）；
  原始数值接近 g 单位，未据此宣称 SI 单位正确，FAST-LIO 初始化会按模长归一化。
- 收到 112 条 `/Odometry_loc`、112 条 `/cloud_registered_1`。
- `/map_save` 返回成功，独立读回 PCD 为 8,414 点，XYZ 全部有限。
- 修改前为 8,415 点；将旧地图按 `C` 旋转后，与新地图最近邻距离中位数
  2.88 mm，95 分位 4.95 mm。该指标用于核对坐标旋转一致性，不代表地图绝对精度。
- X-Z 剖面对比中，原先约 +1.2 m 的主要水平面变为约 -1.2 m，符合地面应在雷达下方的预期。
  平面仍有倾斜，本功能未进行重力调平或安装角标定。
- 静止段相对首个里程计位置最大变化约 5.43 cm；未设置精度验收阈值，不能称为定位精度通过。
- 产物在 `tmp/g1_real_data/replay/`：`static_01_inverted.pcd`、
  `inverted_validation.json`、`inverted_metrics.json`、`inverted_comparison.png` 及日志。
- 未连接真机、未发送运动指令。驱动已补偿的数据必须保持此开关关闭。
