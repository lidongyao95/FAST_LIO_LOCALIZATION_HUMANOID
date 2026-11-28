# 2026-11-25 石膏场景导航踉跄与雷达求交耗时

## 现象和范围

在 4622（`sjtu-PowerEdge-T640-00`、Ubuntu / ROS 2 Humble、RTX 4090）的 B3 第七步，G1 释放支撑后行走不稳、画面断续。排查使用 `B1_run03` 的石膏场景，测试均为 MuJoCo 仿真；未连接或驱动真实 G1。

先前曾有旧进程加载 `sim/generated/B1_run03_preview/scene.xml`，其中包含 29,420 个扫描显示/碰撞盒。该进程已停止，旧生成器、独立 `scan_preview.sh` 入口和 4622/4322 的旧场景产物已移除。**本次新的踉跄复现加载的是 `sim/generated/B1_run03/scene.xml`，不是旧体素场景。** 实际 XML 使用连续石膏显示网格和从同一表面三角面构造的 8 mm 碰撞薄层。

## 实测与判断

| 检查 | 实测结果 |
| --- | --- |
| GPU 渲染 | 仿真窗口进程出现在 `nvidia-smi` 的 `G` 类进程中，使用 RTX 4090，显存约 252 MiB；桌面 `:11.0` 默认可能是 llvmpipe，需 NVIDIA offload 环境。 |
| 策略推理 | 控制器链接 CPU 版 `onnxruntime-linux-x64-1.22.0`，`OrtRunner` 使用默认 SessionOptions，未注册 CUDA provider。 |
| 障碍接触 | 记录释放支撑及短距离行走时的全部接触，摇晃阶段没有机器人与 `plaster_collision_*` 的接触。 |
| 原雷达路径 | 4320 条射线逐个查询物理碰撞薄片，一次扫描约 160 ms，超过 10 Hz 雷达的 100 ms 周期。 |
| 在完整模型上改查显示表面 | 同样射线约 57 ms，仍需遍历完整物理模型中的几何。 |
| 独立表面查询模型 | 仅保留同一 OBJ、地面和动态障碍，扫描约 9.5–9.7 ms；参考距离误差为 0，动态障碍插入/移除同步通过。 |

以下为同一场景、同一策略、相同短时站立与前行操作的对照。实时倍率为模拟时间增量 / 墙钟时间增量，1 才是实时；不是 FPS。姿态峰值仅描述各次短测，不能当作全路线稳定性保证。

| 对照 | 平均实时倍率 | 最大侧倾 | 最大俯仰 | 石膏障碍接触 |
| --- | ---: | ---: | ---: | --- |
| 修复前完整场景 | 0.320 | 2.81° | 4.08° | 无 |
| 仅关闭石膏物理接触，保留原射线查询 | 0.336 | 1.93° | 5.09° | 无 |
| 去掉场景几何的平地对照 | 0.975 | 1.28° | 3.06° | 无 |
| 修复后完整场景 | 0.785 | 0.86° | 2.06° | 无 |

提交前复测同一射线回归为约 11.6–11.9 ms/帧，参考距离误差仍为 0，说明绝对耗时随运行负载变化；原始对照数值以对应证据文件为准。提交前检查日志在上述证据目录的 `pre-push/tests.log`。

这些对照排除了本次“残留体素碰撞/碰到石膏障碍”的解释，确认原雷达求交是主要性能瓶颈。模拟时间落后，而策略与 ROS 的部分循环依墙钟运行，控制时序不匹配是摇晃的重要解释；没有做独立控制器时钟改造实验，不能把所有晃动都归因于单一因素。

## 修复

- `scripts/simulation/plaster_raycast.py` 复用显示网格、地面和动态障碍，构建仅用于射线查询的模型，不复制数千个物理接触薄片，不改变物理碰撞几何。每帧同步 mocap 障碍位置及姿态。
- `g1_mujoco_sensors.py` 提供射线查询方法，`g1_walking_sim.py` 在石膏场景中调用上述查询器；普通房间路径保留原实现。
- `scripts/checks/check_plaster_raycast.py` 对比相同表面的参考求交，覆盖动态障碍开启、关闭及再次移除。
- 图形导航入口在启动时核对实际 NVIDIA OpenGL renderer，软件渲染时报错。GPU 只加速当前图形渲染；不把原生 MuJoCo 物理、CPU 雷达求交或 CPU ONNX 推理说成 GPU 运算。
- 可选环境变量 `G1_SIM_DIAGNOSTICS` 指定 JSONL 路径，记录实时倍率、姿态、LowCmd 新鲜度、支撑释放及石膏接触；默认不生成诊断数据。

代码在 Mac 修改后同步至 4622。用户入口统一为 `03_simulation.sh nav`；点云降采样和 DUFOMap 内部网格不是旧场景碰撞实现，继续保留。

## 验证和限制

修复后在隔离域 77 启动图形 MuJoCo、控制器、定位、Nav2 和 RViz，短目标 `(1.4, -0.9)` 返回 `SUCCEEDED`（状态 4），27 条非零速度指令、2 次路径更新。完整导航平均实时倍率约 0.699，最大侧倾 1.60°、俯仰 1.43°，未记录石膏障碍接触。测试后已清理本次进程。

目标测试沿用现有到点容差；最终真值约 `(1.233, -0.863)`，距目标约 0.17 m。地图位姿与真值的约 0.0033 m 差异不是到目标误差。该结果证明短目标链路可执行，不代表精确到点、长路线、动态障碍全套验收或完全实时。完整导航仍有约 30% 的模拟时间落后，不应宣称“已完全流畅”。

## 复现与证据

在服务器图形桌面、已核对 DISPLAY 的仓库终端：

```bash
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1
bash scripts/env/ros_overlay_ns.sh ./scripts/simulation/03_simulation.sh nav sim/generated/B1_run03
```

专项射线检查（不启动导航）：

```bash
.deps/venv/bin/python scripts/checks/check_plaster_raycast.py sim/generated/B1_run03
```

4622 证据目录：`/home/sjtu/ldy/g1_real_data/records/gait-diagnosis-20261125/`。

- `full-before-fix/`、`no-contacts/`、`flat/`、`full/`：对应对照日志、接触遥测和 `summary.json`。
- `ray.log`、`ray-fixed.log`：射线耗时及参考距离比较。
- `nav-result.json`、`navigation-summary.json`、`nav-telemetry.jsonl`：完整导航结果及遥测。
- ROS 会话：`sim/generated/workflow_logs/simulation-20261125-112544-rpGFFH/`。
- 旧产物删除清单：`/home/sjtu/ldy/g1_real_data/records/plaster-cleanup-20261125/`。

后续看到踉跄时，先确认实际场景路径、碰撞接触、NVIDIA renderer 和实时倍率；不要仅凭视觉判断为体素残留，也不要因 GPU 有占用便认定整条链路实时。
