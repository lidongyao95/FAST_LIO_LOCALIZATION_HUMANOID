# 2026-11-24 PC2 无初值全局重定位

## 实现

- `04_robot_ready.sh localize --map MAP.ply` 默认自动初始化，不再交互要求六维初值；显式 `--initial` 仅保留为调试路径。
- `capture_global_scans.py` 从同时间戳 `/cloud_registered_1` 与 `/Odometry_loc` 采集三组各 6 帧、不重叠且新鲜的 camera_init 点云，检查机体静止和时间新鲜度。
- `open3d_loc/global_initialize` 采用全图/重叠分区 FPFH/RANSAC，以及利用重力方向的全图 XY/yaw 相关性提案；全图 ICP 精化与评分。质量门槛为重叠率 ≥0.60、RMSE ≤0.15m、重力方向偏差 <20°。分离位置/朝向（>0.6m 或 >10°）的合格竞争模式评分差 <0.08 时拒绝。
- 后两组扫描各自独立验证，修正不得大于 0.2m / 5°；搜索最长 120 秒，采集最长 20 秒。输出接受的 map←camera_init 变换后才启动持续定位。仅在初始化完成后持续节点发布地图位姿/TF。
- 相关性提案依赖 LIO 与地图重力对齐/地面可辨识。有界多候选搜索不构成任意场景唯一性证明；特征不足、重复结构、失败与超时均退出，不默认零初值、不启用运动。

## 构建与回归

- PC2/Foxy/aarch64 原生构建 `open3d_loc`。首轮链接失败定位到 ARM Open3D 的 lapacke→tmglib 传递依赖；补充本机 `-rpath-link` 后仅重链接受影响包，成功。源码改动均在 Mac 完成后同步，无远端源码漂移。
- 构建流式日志和状态位于 PC2 `~/ldy/g1_real_data/records/builds/auto-global/`：`build.log`（初次失败）、`relink.log`（9.99s 成功）、`correlation-build.log`（17.4s 成功）。
- 回归 `check_global_initialization.py`：真实地图裁剪加未知 11m/-8m/0.35m 平移和 117° 转角及独立噪声，恢复误差门槛 0.15m；重复结构歧义、无关随机点云、多帧不一致均拒绝且不写成功结果。四项 PASS。证据在上述目录 `correlation-regression/`。这属于同源几何变换回归，不等于独立现场精度验收。
- 进程筛选清理回归 8 项通过；新增搜索/采集进程纳入定位预检；01 robot 检查自动初始化可执行文件。

## 实时结果

- 第一轮实时仅 FPFH 搜索未找到合格候选，正确退出；未发布候选位姿。证据：`~/ldy/g1_real_data/records/workflows/robot-ready-20261124-150546-w5mOfh/`。
- 对同一组真实扫描补充全图相关性搜索后，重叠率 0.996745、RMSE 0.0723323m，两组验证亦通过。证据：`auto-global/live-correlation.log` / `.json`。
- 完整重新采集→全图自动搜索→持续定位通过：`~/ldy/g1_real_data/records/workflows/robot-ready-20261124-150931-HTv3Zg/`；初始化重叠率 0.998262、RMSE 约0.07331m，后两组验证通过，持续配准开始更新 `map→body`。启动搜索期间只读 TF 探针返回无地图 TF，接受后 TF 与健康检查通过。
- 全程没有启动 Nav2 控制器、速度桥或 SDK 写入；验证会话复查后停止，网页只读地图检查保留。成功仅说明本次静止几何匹配与跟踪通过，未完成多起点独立参考位姿误差验收。
