# 实机点云石膏场景与导航预演

完整操作流程统一维护在 [WORKFLOWS B3](../docs/WORKFLOWS.md#b3-可选在仿真机预演实机地图)。仿真始终在 Ubuntu 服务器上运行；Mac 负责代码同步和输入传输。

## 场景与输入

原始 PCD 用于拟合地面与坐标对齐。可调整降采样、高度、裁剪范围和近距过滤；DUFOMap、逐帧近距、时间抽样和 IMU 筛帧需要同轮配对输入。参数及输入哈希保存在 `filter_report.json`，过滤后的原地图坐标点云保存在 `filtered.pcd`。

重建输出连续石膏网格 `plaster.obj`。显示和碰撞由同一组顶点、三角面生成：物理碰撞沿各三角面形成 8 mm 薄层，保留房间凹部与通道。虚拟雷达直接对该表面求交，使用仅包含同一网格、地面和动态障碍的查询模型，避免逐个遍历物理碰撞薄片；动态障碍位置与物理场景同步。配套地图也来自该表面。有限平地延伸是仿真假设，重建可能平滑窄缝或小物体，须人工核对。

## 服务器操作入口

在已设置 DISPLAY 的服务器远程桌面终端、仓库根目录执行：

```bash
export G1_SERVER_PCD="$HOME/ldy/g1_real_data/replay/B1_run03.pcd"
./scripts/simulation/03_generate_scene.sh "$G1_SERVER_PCD"
./scripts/simulation/03_preview_scene.sh sim/generated/B1_run03
```

预览入口确认实际使用 NVIDIA GPU。检查完成并关闭预览窗口后执行：

```bash
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1
bash scripts/env/ros_overlay_ns.sh ./scripts/simulation/03_simulation.sh nav sim/generated/B1_run03
```

需要调参时使用 `03_generate_scene.sh --rebuild`；旧石膏结果归档，原始输入保留。场景加载、支撑稳定性、落球碰撞及视觉/碰撞射线一致性有专项检查，但不代替完整导航验收。服务器依赖为本机 `.deps/venv` 的 MuJoCo、NumPy、SciPy、scikit-image、fast-simplification，DUFOMap 按需启用。

`02_mapping.sh export --plaster` 是地图表面预览，不会生成此处可运行的 MuJoCo 场景。实机 B4/B5 使用 B2 已核对地图，不能直接使用仿真配套地图。
