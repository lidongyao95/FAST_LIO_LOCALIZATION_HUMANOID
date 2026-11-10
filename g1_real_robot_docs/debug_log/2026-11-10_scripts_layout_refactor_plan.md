# scripts 目录重构计划

目标是把 `scripts/` 从「40 个平铺文件的杂物间」整理成按运行场景分层的目录。本文记录迁移计划与验收判据。

按运行场景拆目录的方向合理。实施前最需要锁定的是共享环境脚本、源码与 ROS 安装副本的对应关系，以及远端现场修正。先迁移运行入口和直接引用，归档与历史入口清理单独评估。

## 1. 现状与证据

`scripts/` 顶层 40 个文件，混了五类完全不同用途的东西：

| 类别 | 文件 |
| --- | --- |
| 真机运行链路（launch 直接调用） | `ensure_g1_nav_lifecycle.py`、`g1_real_velocity_bridge.py`、`g1_nav_tf.py` |
| 真机诊断与采集（现场手册调用） | `probe_topics.py`、`replay_clock.py`、`record_g1_a_segment.sh` |
| SDK 会话 C++ | `g1_loco_zero_session.cpp`、`g1_loco_preflight.cpp`、`g1_loco_sender_session.hpp`、`g1_loco_velocity_sender.hpp` |
| MuJoCo 仿真入口与验证 | 8 个 `start_*/stop_*/run_*.sh`、`walking_env.sh`、`g1_walking_sim.py`、`g1_mujoco_sensors.py`、`g1_command_bridge.py`、`g1_operator.py`、4 个 `validate_*`、`check_g1_localization.py` |
| 构建/地图/一次性工具 | `build_g1_controller.py`、`build_g1_ros2.sh`、`setup_g1_local_deps.sh`、`generate_nav_params.py`、`export_g1_map.py`、`g1_scan_scene.py`、`dufomap_clean.py`、4 个 DUFOMap 脚本、`check_g1_scan_scene.py` |

另有 `__pycache__/` 6 个 `.pyc` 被提交，`validate_nav_goal` 与 `validate_walking` 各有 `.py` + `.sh` 双份入口。

## 2. 三类硬耦合（决定迁移顺序）

1. **launch 里写死 `repo/scripts/<name>`**：`open3d_loc/launch/g1_navigation.launch.py`（109、116、131 行）、`g1_walking_localization.launch.py`（76 行）、`g1_mujoco.launch.py`（48 行）、`ensure_g1_nav_lifecycle.py`（按 basename 匹配速度桥，文件名保持时无需修改）。
2. **shell 里写死字面路径**：`g1_operator.sh:4`、`start_g1_controller.sh:7,12`、`start_walking_sim.sh:6`、`validate_walking.sh:4`、`validate_nav_goal.sh:4`、`run_g1_mujoco.sh:11`，都依赖 `walking_env.sh` 的 `cd "$G1_REPO"`；`record_g1_a_segment.sh:15` 等 9 个脚本 source 了 `walking_env.sh`。
3. **Python 同级 import**：`dufomap_clean`、`export_g1_map`、`g1_scan_scene`、`check_g1_scan_scene`、`g1_mujoco_sensors` 五个模块被兄弟文件 `import`，靠「脚本所在目录自动进 `sys.path`」生效。移动时必须整组同迁，否则改 import。

**额外陷阱**：10 个文件用 `Path(__file__).resolve().parents[1]` 定位仓库根（`build_g1_controller.py`、`check_g1_localization.py`、`check_g1_scan_scene.py`、`g1_command_bridge.py`、`g1_mujoco_sensors.py`、`g1_operator.py`、`g1_scan_scene.py`、`generate_nav_params.py`、`validate_nav_goal.py`、`validate_walking.py`）。下沉一层后 `parents[1]` 会静默指向 `scripts/`，表现为路径找不到或写错位置，必须同步改成 `parents[2]`。

## 3. 目标结构

```text
scripts/
  real/                     真机链路，launch 与现场手册直接调用
    ensure_g1_nav_lifecycle.py
    g1_real_velocity_bridge.py
    g1_nav_tf.py
    probe_topics.py
    replay_clock.py
    record_g1_a_segment.sh
    loco_cpp/               仅用于编译 SDK 会话，不进 ROS 包
      g1_loco_zero_session.cpp
      g1_loco_preflight.cpp
      g1_loco_sender_session.hpp
      g1_loco_velocity_sender.hpp
  walking_env.sh            真机采集与仿真共用的环境入口
  sim/                      MuJoCo 仿真入口与验证
    start_g1_controller.sh  start_g1_mapping.sh  start_g1_localization.sh
    start_g1_navigation.sh  start_walking_sim.sh  stop_g1_mujoco.sh  run_g1_mujoco.sh
    g1_walking_sim.py       g1_mujoco_sensors.py
    g1_command_bridge.py    g1_operator.py  g1_operator.sh
    validate_walking.py     validate_walking.sh
    validate_nav_goal.py    validate_nav_goal.sh
    check_g1_localization.py
  tools/                    构建、地图与一次性实验
    build_g1_controller.py  build_g1_ros2.sh  setup_g1_local_deps.sh
    generate_nav_params.py
    export_g1_map.py  g1_scan_scene.py  dufomap_clean.py  check_g1_scan_scene.py
    check_dufomap_loop.py  check_dufomap_sim.py  dufomap_loop_sim.py
    record_dufomap_scans.py
  g1_nav_web/               保持不变（真机网页与静止对齐）
  hang_a02/                 保留现有实验目录，同步必要的导入路径
```

不做的事：不改 `g1_nav_web/` 内部结构（现场手册已经引用 `scripts/g1_nav_web/server.py` 与 `static_alignment/*`，收益低风险高）；不把 `scripts/` 变成 ROS 包。由 `open3d_loc` 安装 launch，launch 再通过 `repo` 参数访问源码树。

## 4. 迁移映射

| 旧路径 | 新路径 | 风险点 |
| --- | --- | --- |
| `scripts/g1_real_velocity_bridge.py` | `scripts/real/g1_real_velocity_bridge.py` | 更新 launch 路径，保留 basename 和生命周期冲突检查 |
| `scripts/ensure_g1_nav_lifecycle.py` | `scripts/real/` | launch 109、131 行 |
| `scripts/g1_nav_tf.py` | `scripts/real/` | `g1_walking_localization.launch.py:76` |
| `scripts/probe_topics.py` | `scripts/real/` | 现场手册步骤 2 的命令 |
| `scripts/replay_clock.py`、`record_g1_a_segment.sh` | `scripts/real/` | 仅文档引用，需改文档 |
| `scripts/g1_loco_*.cpp/hpp`（4 个） | `scripts/real/loco_cpp/` | 文档里的编译命令，包含 `-I$R/scripts` |
| `scripts/g1_mujoco_sensors.py`、`g1_walking_sim.py` | `scripts/sim/` | 二者必须同目录；`g1_mujoco.launch.py:48` |
| 仿真入口的 8 个 `.sh` | `scripts/sim/` | 同步 source 路径、字面调用及启动与停止脚本的仓库根定位 |
| `scripts/walking_env.sh` | 原路径 | 保留现有 source 入口和环境契约 |
| `scripts/g1_command_bridge.py`、`g1_operator.py` | `scripts/sim/` | 同步调用方及仓库根定位 |
| `scripts/validate_*`、`check_g1_localization.py` | `scripts/sim/` | 文档 |
| `scripts/dufomap_clean.py`、`export_g1_map.py`、`g1_scan_scene.py`、`check_g1_scan_scene.py` | `scripts/tools/` | 四个模块互相 import，且 `hang_a02/plaster-map/mujoco/trial.py:12` 跨目录 import 了 `check_g1_scan_scene` |
| `scripts/dufomap_loop_sim.py` 等 3 个 DUFOMap 脚本 | `scripts/tools/` | 仅文档引用 |
| `scripts/build_*`、`setup_g1_local_deps.sh`、`generate_nav_params.py` | `scripts/tools/` | `generate_nav_params.py:51` 会把路径写进 `navigation/g1_nav2_params.yaml` 的注释 |

## 5. 必须同步修改的引用

- **launch（3 个文件、5 处）**：`open3d_loc/launch/g1_navigation.launch.py`、`g1_walking_localization.launch.py`、`g1_mujoco.launch.py`。
- **Shell**：逐个检查 `BASH_SOURCE`、`dirname`、`source` 和相对路径。移动后的真机采集和仿真入口引用上一级的 `walking_env.sh`。更新 `run_g1_mujoco.sh`、`stop_g1_mujoco.sh`、`build_g1_ros2.sh`、`setup_g1_local_deps.sh` 的仓库根计算。保留环境脚本的 ROS overlay、域、日志及工作目录语义。
- **Python 与配置**：按目标位置逐一检查根路径、同级导入、跨目录导入、动态拼接和命令提示。同步生成器及已提交 YAML 中的入口注释。
- **文档**：`CONFIGURATION.md`、`VALIDATION.md`、`NAVIGATION_MANUAL.md`、`README.md`、`sim/README.md`、`g1_real_robot_docs/`（`g1_pc2_live_nav_web_runbook.md`、`g1_real_robot_slam_pc2_b300_guide.md`、`g1_real_robot_slam_notes.md`、`2026-11-10_g1_autonomous_walk_plan.md`）、`debug_log/` 内引用脚本的几篇、`docs/superpowers/plans/2026-11-05-dufomap-sim.md`、`scripts/hang_a02/README.md`。
- **PC2 与 4322 的部署副本**：迁移后必须重新 rsync，否则现场手册里的命令会指向不存在的路径；迁移当天的实机验证要在同步完成后重跑。

顺带修两处已过期的描述：`debug_log/2026-11-10_pc2_live_nav_web.md` 说 `g1_loco_sender_session.hpp`「只存在于 PC2」，实际仓库里已有；`g1_real_robot_docs/README.md` 对 `notes` 与 `guide` 的关系也需要随目录调整更新。

## 6. 分阶段执行与验证

每批原子迁移入口、调用方、当前手册及生成配置。先在两端对同一个候选构建并运行，再提交该批。前一批通过后进入下一批。

| 阶段 | 内容 | 通过判据 |
| --- | --- | --- |
| 0 | 记录基线并清理跟踪中的 Python 字节码 | 以 `git ls-files` 确认相关字节码不再被跟踪，生成缓存后用 `git check-ignore` 核对规则 |
| 1 | 迁移 `real/` 与 `loco_cpp/`，同步真机 launch、采集入口和 SDK 编译命令 | PC2 Foxy 与 4322 Humble 在独立 ROS 域用既有录包启动 `mode:=plan`，检查定位 TF、代价地图和规划结果与基线一致，确认速度桥及 SDK 运控进程未启动。按文档编译两个 SDK 会话程序，验收只编译 |
| 2 | 迁移 `sim/` 并更新对应 launch、手册与启动停止脚本 | 4322 用独立 ROS 域从正常入口启动，观察时钟、传感器、TF 与仿真节点，走一次现有导航验证，再通过停止入口检查进程退出。PC2 重跑受影响的通用环境和仅规划入口 |
| 3 | 迁移 `tools/`、更新跨目录 import 和文档 | 两端执行受影响的构建入口。用实际 Python 环境验证导入链及 `trial.py` 的模块解析，在临时输出位置运行参数生成器并比较 YAML，允许预期路径注释变化 |

每批执行以下门禁。

1. 建立完整旧路径到新路径清单。按清单逐条搜索当前运行入口、launch、构建脚本、生成配置和手册，再补查 basename、动态路径拼接以及远端自定义启动配置。历史日志与迁移表保留旧路径，记录允许命中。已有 `.sh` 包装入口随批更新，不在本次合并。
2. 对候选中的每个 Shell 文件分别运行 `bash -n`，对 Python 文件做语法检查及实际导入验证。核对入口执行位，并从仓库根和任意工作目录验证入口的路径解析。启动脚本的 `--help` 可能在解析参数前创建锁、生成文件或启动进程，冒烟应使用明确隔离的运行环境。
3. 同一候选分别构建到隔离的 overlay，核对 `ros2 pkg prefix open3d_loc`、实际安装的 launch 文件及传入的 `repo` 都指向候选。以 `--show-args` 检查参数解析，再跑表中的运行验收。远端构建后台写日志并开启 `console_direct+`，每 20–30 秒查看进程、日志增长及终态，失败先定位具体错误再重试。记录候选标识、构建退出码、日志路径、实际入口及验收结果。
4. 发布前先在 PC2 和 4322 做部署差异预览，保留现场配置、数据、旧源码和可用 overlay，按清单处理迁移留下的旧路径。部署后从日常入口重跑对应冒烟。出现路径解析失败、overlay 混用或基线回归时，一并切回上一套源码与 overlay，重跑原入口确认恢复。

## 7. 不建议同批处理

- `.deps/`（752 MB，5 个 submodule，`unitree_mujoco` 还有未提交改动）：属于构建产物与第三方，换机即重建，不参与整理。
- Mac 当前已有 `g1_walking_localization.launch.py`、`g1_real_velocity_bridge.py`、`setup_g1_local_deps.sh` 等未提交改动。先核对工作区 diff 与 stash 内容，再以已确认的基线制作隔离候选，逐项合入需要保留的修改。
- PC2 曾有 Foxy API 兼容修正与临时 SDK 二进制。实施前对部署副本执行反向 `rsync -ain`，对照 Mac diff 核实修正是否已合入。按工作区约定排除构建产物、第三方依赖和数据。

## 8. 其它同样混乱、值得整理的目录

以下仅作后续候选。归档前应核对活跃入口、安装规则、现场调用和实验复现链；`g1_mujoco.launch.py` 目前仍被启动和停止入口依赖。

| 位置 | 证据 | 建议 |
| --- | --- | --- |
| 仓库根四份 md | `CONFIGURATION.md`（约 700 行）、`VALIDATION.md`、`NAVIGATION_MANUAL.md`、`README.md` 在构建/参数/运行/验证上互相重复，README 还是上游 FAST-LIO 正文加本地叠加段 | 上游内容拆成 `UPSTREAM_README.md`；本地收敛为 README（索引）+ CONFIGURATION + VALIDATION 三份 |
| `g1_real_robot_docs/` 两份指南 | `g1_real_robot_slam_pc2_b300_guide.md`（990 行）与 `g1_real_robot_slam_notes.md`（778 行）前 10 章标题几乎逐字相同 | notes 只保留 x86/Humble 章节，其余并入 guide 附录 |
| `g1_real_robot_docs/debug_log/` 索引 | `README.md` 只列 6 篇，漏了 3 篇；`2026-11-10_g1_autonomous_walk_plan.md` 全仓无入链 | 补全索引并把 walk plan 挂到 guide 或本计划 |
| `open3d_loc/launch/` | 6 个 launch 里 `localization_3d_g1`、`open3d_loc_g1` 已过期（guide 自标「历史入口」），`g1_mujoco` 被 `g1_walking_localization` 覆盖；另有 2 个陈旧 `__pycache__/*.pyc` | 删两个历史入口与 pyc，README 改指现行入口 |
| `scripts/hang_a02/` | 三套并行实现：根目录、`combined/`、`plaster-map/` 各有 `compare/render/rviz_review`，没有说明哪套现行 | 保留 `combined/`，其余进 `archive/` |
| `FAST_LIO/Log/` | `dbg.txt`、`imu.txt`、`pos_log.txt`、`mat_out.txt`、`mat_pre.txt` 等 8 个运行日志被提交 | 移出仓库或 ignore，只保留目录占位 |
| `FAST_LIO/config/` | `avia`、`horizon`、`ouster64`、`velodyne` 与本项目无关 | 按需保留 `mid360`、`g1_walking`，其余归档 |
| `docs/superpowers/plans/` | 只有 1 个孤立计划文件，层级过深 | 并入 `g1_real_robot_docs/debug_log/` |
| 根目录 `doc/` | 19 个媒体文件共 256 MB（`loc.gif` 39 MB 等），全部被上游 README 引用 | 与上游 README 一起处理，避免误删引用 |
