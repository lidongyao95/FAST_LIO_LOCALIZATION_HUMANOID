# 2026-11-24 02 后可选服务器 03 预演集成

> 历史记录：本文旧场景实现及产物已于 2026-11-25 移除，不再作为操作流程。当前统一使用 [WORKFLOWS B3](../../docs/WORKFLOWS.md#b3-可选在仿真机预演实机地图) 的石膏表面生成、预览与导航。

- 02 record 完成后提示导出；export/dufo 完成或遇已有有效地图时，给出网页核对、直接 PC2 04、可选服务器 03 两条分支。
- 复用 03_preview.sh，不再新增重叠的上传实现。play 对 PCD 输入集成 prepare→launch；对已有场景目录直接 launch。prepare/launch 仍可分步执行。
- PC2 到服务器尚未配置已验证的 SSH，默认由操作电脑通过 SSH 中转 PCD，不复制私钥。PC2 IP 从 wlan0 动态读取供提示使用。
- 4622：sjtu-PowerEdge-T640-00，账户 sjtu，工作目录 /home/sjtu/ldy，GPU 0 空闲约7.9 GiB。默认回退4322，显式服务器端口不回退；默认仿真域75，与已运行74和实机223隔离。
- 实测原始 B1_run03 PCD 上传，SHA-256 一致，自动出生点 (1.10,-0.90)，净空0.91m。服务器生成场景/地图通过 MuJoCo 编译。任务 pid/status/generate.log 位于 ~/ldy/g1_real_data/records/preview/，RUNNING→FINISHED 0 跟踪通过。
- 导航启动前检测到 Open3D/PCL 缺失动态库，复用 ~/ldy/logs/g1-nav-20261123-162230/deps 的只读覆盖层，未修改系统。
- 非零出生点暴露隐式 weld 仍保留原点相对偏移的问题；生成器改为显式 relpose=0 0 0 1 0 0 0。测试场景单独修正支撑约束，地图文件未变。
- 完整仿真预演在域75通过：目标 (1.5,-1.6)，goal_status=4/SUCCEEDED，412条非零候选命令、21次路径更新；非硬件命令。测试目标在派生图的连通空闲区域，净空1.25m。
- 最终证据：4622 ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID/sim/generated/workflow_logs/scan-preview-20261124-162203-6S6HA9/nav_goal.json；场景 B1_run03_monitorcheck_161620。测试会话结束后清理，保留原有域74会话。只证明近似扫描场景中的本次仿真导航，不代表实机行走验收。

- 非零出生点回归：`check_g1_scan_scene.py` 按与 WalkingSim 相同的元数据起点初始化。4 秒支撑稳定性无警告、底座最大位移低于0.01m，接触距离约-0.000025m；落球与扫描体素碰撞验证通过。回归输出保存在同测试场景目录 `validation.json` / `collision_validation.json`。
