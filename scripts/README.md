# 脚本入口与目录职责

日常运行先按[使用流程](../docs/WORKFLOWS.md)选路线。本目录按职责组织，入口编号保留现有命名，不要求跨目录依次执行。

## 常用入口

| 任务 | 入口 | 运行位置 |
| --- | --- | --- |
| 查离线、仿真或实机依赖 | `workflows/01_self_check.sh offline\|sim\|robot` | 对应处理机、仿真机或 PC2 |
| 录制实机数据、导出地图 | `workflows/02_mapping.sh record` / `export` | `record` 在 PC2；`export` 可离线处理 |
| DUFOMap 动态点清理并导出地图 | `workflows/02_mapping.sh capture-scans` / `dufo` | 逐帧采集跟随 FAST-LIO；清理在装有 DUFOMap 的处理机执行 |
| 内置房间建图和导航 | `simulation/03_simulation.sh map\|nav` | Ubuntu 仿真机 |
| 实机点云生成场景后的导航预演 | `simulation/scan_preview.sh` | Ubuntu 仿真机，使用虚拟传感器 |
| 查询实时状态、启动持续定位 | `real_robot/04_robot_ready.sh status\|localize` | PC2 |
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

`03_simulation.sh` 和 `scan_preview.sh` 调用 `session.sh`，再编排 `start_*.sh`、控制器、定位与 Nav2。日常运行用统一入口；需要单独观察某个组件时，再按[手动运行手册](../docs/NAVIGATION_MANUAL.md)使用底层入口，避免同时启动两套会话。

`mapping/` 中的 `pcd.py`、`g1_scan_scene.py` 等被其他工具导入；仿真脚本也有同目录导入。再次移动文件时需同时核对 Python 导入、`__file__` 根目录计算、shell 的 `source` 和 launch 中的路径。

## 部署时要一起更新什么

这些脚本通过源码目录调用 ROS 包，不是一个独立安装的 ROS 包。启动所需的不只是复制 `scripts/`：

1. 同步脚本、调用它们的 `open3d_loc/launch/`、配置及相关源码。PC2 现场有改动时先用反向 `rsync -ain` 核对。源码同步排除 `.git/`、`build/`、`install/`、`log/`、`.deps/sdk*`、`.deps/root/`、`vendor/`、`doc/`、`tmp/` 与仿真子模块，现场数据单独管理。
2. 在目标主机更新受影响的 ROS 包，核对实际加载的 `install/`；换架构重新构建依赖和二进制。`build/`、`install/`、`.deps/sdk*`、`.deps/root/`、`vendor/` 不跨机复制作为可用构建。
3. 核对传入的 `repo` 指向这次部署的源码目录，再从日常入口复验。源码迁移完成不等于 PC2 或服务器已部署完成。

远端构建使用 `console_direct+`、后台日志，每 20–30 秒检查进程、日志增长与 `Finished/Failed`，失败先定位原因。首次部署先读[PC2 历史问题汇总](../g1_real_robot_docs/debug_log/2026-11-05_g1_pc2_debug_summary.md)。
