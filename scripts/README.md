# 脚本入口与目录职责

日常运行先按[使用流程](../docs/WORKFLOWS.md)选路线。本目录按职责组织，入口编号保留现有命名，不要求跨目录依次执行。

实机从头建图的顺序：Mac 上 `00 check → sync → build`，随后转到 PC2 执行 `01 robot → 02 record/export → 04 localize → 05 Web`。开始采集前独立执行 01；03 仅用于可选仿真。

## 常用入口

| 任务 | 入口 | 运行位置 |
| --- | --- | --- |
| PC2 有线接入、部署检查、源码同步与 PC2 本机构建 | `real_robot/00_pc2_setup.sh check` / `sync` / `build` | Mac / 部署电脑 |
| 查离线、仿真或实机依赖 | `workflows/01_self_check.sh offline\|sim\|robot` | 对应处理机、仿真机或 PC2 |
| 检查解包式 Open3D 本地库的传递依赖（缺库时给修复命令） | `env/open3d_runtime_probe.sh`（`--fix` 免 root 补齐） | PC2 / 任何用 `.deps/arm-open3d` 的主机 |
| 录制实机数据、导出地图 | `workflows/02_mapping.sh record` / `export` | `record` 在 PC2；`export` 可离线处理 |
| DUFOMap 动态点清理并导出地图 | `workflows/02_mapping.sh capture-scans` / `dufo` | 逐帧采集跟随 FAST-LIO；清理在装有 DUFOMap 的处理机执行 |
| 内置房间建图和导航 | `simulation/03_simulation.sh map\|nav` | Ubuntu 仿真机 |
| 实机地图一键上传仿真机并自动选点生成预演场景 | `simulation/03_preview.sh prepare` / `play pc2:/...pcd` | Mac 仓库根发起；仿真在仿真机运行，PCD 取自 PC2（`pc2:/abs/path.pcd`） |
| 启动已有扫描场景做导航预演 | `simulation/03_preview.sh play` | Mac 仓库根发起，自动 SSH 到仿真机执行 MuJoCo + Nav2 |
| 实机点云生成场景后的导航预演 | `simulation/03_simulation.sh nav <场景目录>` | Ubuntu 仿真机，使用虚拟传感器 |
| 查询实时状态、自动全局重定位后持续跟踪 | `real_robot/04_robot_ready.sh status\|localize` | PC2 |
| 定位前网页检查导出地图 | `real_robot/05_web_navigation.sh MAP_DIR --review --port 18082` | PC2；Mac SSH 转发并浏览 |
| 网页仅规划 | `real_robot/05_web_navigation.sh` | PC2；浏览器可在 Mac |

以上均为相对 `scripts/` 的路径，支持 `--help`。自检检查依赖；`04 ... status` 检查已运行的数据链，两者用途不同。

DUFOMap 的[操作步骤](../docs/WORKFLOWS.md#用-dufomap-清理多帧扫描)由统一建图入口调用
[`mapping/record_dufomap_scans.py`](mapping/record_dufomap_scans.py) 和
[`mapping/dufomap_clean.py`](mapping/dufomap_clean.py)。算法库是所选 Python 环境中的
`dufomap==1.1.1` 包，仓库内保存采集、调用与验证脚本。

## 目录如何分工

| 目录 | 职责与边界 |
| --- | --- |
| `workflows/` | 跨场景自检与地图流程；`lib.sh` 管理本轮子进程和日志，`probe.py` 检查 ROS 数据与就绪状态 |
| `env/` | 依赖准备、构建、运行时环境、遗留进程清理；`walking_env.sh` 同时被仿真与实机复用 |
| `mapping/` | 点云读取、DUFOMap、地图导出、石膏预览及扫描场景生成；生成场景不等于启动仿真 |
| `simulation/` | MuJoCo、仿真控制器、仿真运控与 TF；`session.sh` 让内置房间和扫描场景共用启动顺序 |
| `real_robot/` | 实机采集、实时定位入口、独立 SDK 状态查询/速度探针和 `web/`；SDK 探针不属于网页规划链 |
| `navigation/` | 仿真和实机共用的 Nav2 参数生成、生命周期管理；`planning_obstacles.py` 由实机 Nav2 会话管理全局规划障碍输入 |
| `checks/` | 独立诊断和验证程序；行走、目标导航、真值误差检查用于仿真，`probe_topics.py` 用于实机输入检查，`check_planning_obstacles.py` 离线检查规划障碍服务与 Web 客户端，`check_process_preflight.py` 检查进程筛选与清理边界，`check_navigation_architecture.py` 检查两种 Nav2 模式，`check_obstacle_thresholds.py` 检查点云邻域筛选和阈值事务 |
| `experiments/` | `hang_a02` 等特定数据集的历史复现，路径和资产按实验记录准备 |

`03_simulation.sh` 调用 `session.sh`，再编排 `start_*.sh`、控制器、定位与 Nav2。日常运行用统一入口；需要单独观察某个组件时，再按[手动运行手册](../docs/NAVIGATION_MANUAL.md)使用底层入口，避免同时启动两套会话。

`mapping/` 中的 `pcd.py`、`g1_scan_scene.py` 等被其他工具导入；仿真脚本也有同目录导入。再次移动文件时需同时核对 Python 导入、`__file__` 根目录计算、shell 的 `source` 和 launch 中的路径。

## 部署时要一起更新什么

这些脚本通过源码目录调用 ROS 包，不是一个独立安装的 ROS 包。启动所需的不只是复制 `scripts/`：

1. 同步脚本、调用它们的 `open3d_loc/launch/`、配置及相关源码。PC2 现场有改动时先用反向 `rsync -ain` 核对。源码同步排除 `.git/`、`build/`、`install/`、`log/`、`.deps/sdk*`、`.deps/root/`、`vendor/`、`doc/`、`tmp/` 与仿真子模块，现场数据单独管理。
2. 在目标主机更新受影响的 ROS 包，核对实际加载的 `install/`；换架构重新构建依赖和二进制。`build/`、`install/`、`.deps/sdk*`、`.deps/root/`、`vendor/` 不跨机复制作为可用构建。
3. 核对传入的 `repo` 指向这次部署的源码目录，再从日常入口复验。源码迁移完成不等于 PC2 或服务器已部署完成。

远端构建使用 `console_direct+`、后台日志，每 20–30 秒检查进程、日志增长与 `Finished/Failed`，失败先定位原因。首次部署先读[PC2 历史问题汇总](../g1_real_robot_docs/debug_log/2026-11-05_g1_pc2_debug_summary.md)。

`04 ... localize --map MAP.ply` 默认无初值自动重定位；静止采集三组点云、竞争候选与后续扫描验证均通过后才启动持续定位。`--initial` 仅用于显式调试。离线回归入口：`checks/check_global_initialization.py`（运行于本机构建 Open3D 的 Ubuntu 主机）。

02 完成后同时提示：直接 PC2 04，或可选服务器 03。`03_preview.sh play` 对 PCD 输入集成上传、校验、生成与启动；`prepare` 仅生成、`launch` 启动已有场景。默认仿真域 75；全部仿真在服务器运行。
