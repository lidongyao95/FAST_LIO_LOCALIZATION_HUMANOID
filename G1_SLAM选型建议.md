# 宇树 G1 仿真与真机导航：完整方案选型建议

> 初次记录：2026-10-22；核查更新：2026-10-23
> 适用对象：Unitree G1，重点关注 MID-360、EDU / PC2 配置；具体系统、传感器和固件以目标机器为准。
> 本文仅做仓库调查与选择，不要求在本知识库目录或当前环境安装、编译、部署。
> 依据为上游公开文档及部分关键代码；本次没有实际编译、运行仿真或测试真机。推荐表示值得优先考察，不表示已由本文作者验证可用。

## 一、先看选择结论

本次需求是：**使用者不熟悉导航，希望获得完整使用流程；同时重视仓库受欢迎程度和工程可信度，允许以较受关注的核心仓库为基础做范围明确的补齐。**

**环境约束：Ubuntu 22.04 + ROS 2 Humble。** 下文主选均围绕该环境；ROS 1 Noetic、ROS 2 Foxy、Jazzy 主线不作为直接采用方案。CPU 架构尚未指定，不能把 Ubuntu 22.04 等同于 x86 或 ARM64。

完整方案最终应覆盖：**建图 → 保存地图 → 重新启动并定位 → 指定目的地 → 规划、绕障与行走 → 停止导航。**

| 优先级                 | 候选路线                                                                                                       | 综合判断                                                                                                                                  |
| ---------------------- | -------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| **真机主选基础** | **DeepGlint `FAST_LIO_LOCALIZATION_HUMANOID` 的 `humble` 分支 + Humble Nav2 + 官方 G1 高层运动接口** | 相对其他 G1 专用候选有明显更高的社区关注度；缺失模块清楚，可形成有边界的补齐项目。但不是现成完整方案，需要具备 ROS 2 导航集成能力的人实施 |
| **完整真机备选** | `unitree-g1-slam-nav-guide`                                                                                  | Humble 中文完整应用候选；仅约 1 Star / 0 Fork，独立复现证据不足，降为对照与有条件备选，不再仅因功能完整排第一                             |
| **仿真候选**     | GrayStars G1 MuJoCo ROS 2 工作区                                                                               | Humble 集成仿真约 30 Star，有保存地图重定位缺口；可作仿真基础，但未发现同等高关注度的完整 G1 Humble 导航仿真方案                          |
| **完整研究备选** | `rl_hnav`                                                                                                    | Humble、仿真与真机模式较齐全，约 20 Star；仍是小规模项目，且引入 RL 步态、双仿真器，不因完整而自动优于主选                                |

**建议从“高关注核心 + 成熟导航框架 + 少量适配”的路线优先评估，把低 Star 完整仓库作为配置与流程参考。** 这是对可维护性与集成工作量的选型判断，不是对组合方案的实测背书。如果完全没有开发支持，DeepGlint 补齐路线也不能承诺直接使用；此时应要求维护者提供与目标环境一致的完整演示、依赖交付或集成支持。

## 二、受欢迎程度与工程可信度

### 2.1 GitHub 数据快照

以下数字于 **2026-10-23** 通过 GitHub REST API 查询；Star / Fork 随时间变化。日期使用所列分支最近提交日期，不把仓库 `pushed_at` 当作代码最后修改时间。数量只反映整个仓库，不能证明 Humble 分支或 G1 实机经过同等规模验证。

| 仓库                                                                                                   |          Star |         Fork | 默认分支最近提交 | Humble 适配与主要信号                                                                                   |
| ------------------------------------------------------------------------------------------------------ | ------------: | -----------: | ---------------- | ------------------------------------------------------------------------------------------------------- |
| [DeepGlint/FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) | **930** | **88** | 2026-01-06       | 有 `humble` 分支，其最近提交为 2025-12-01；团队署名、定位演示、公开问题讨论；非活跃持续维护的完整产品 |
| [grove-g1](https://github.com/Adyansh04/grove-g1)                                                       |            83 |           10 | 2026-10-18       | 近期有功能开发；主线 Jazzy，旧 `humble-unitree` 分支不能视为与主线功能等价                            |
| [GrayStars G1 仿真](https://github.com/GrayStars/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-)                |            30 |            4 | 2026-05-31       | Humble 集成仿真，默认分支仅少量提交，维护广度有限                                                       |
| [rl_hnav](https://github.com/uleroboticsgroup/rl_hnav)                                                  |            20 |            3 | 2026-07-16       | Humble 研究集成方案，社区使用证据仍有限                                                                 |
| [binbin2002/Unitree_G1_Navigation](https://github.com/binbin2002/Unitree_G1_Navigation)                 |            20 |            4 | 2026-03-24       | ROS 1 Noetic，不符合主选环境                                                                            |
| [guruadp/unitree-g1-slam-nav](https://github.com/guruadp/unitree-g1-slam-nav)                           |             8 |            0 | 2026-03-26       | Humble 最小闭环，默认分支仅一次提交                                                                     |
| [unitree-g1-slam-nav-guide](https://github.com/zyli6269-cmyk/unitree-g1-slam-nav-guide)                 |             1 |            0 | 2026-10-22       | Humble、中文完整流程；2026-10-19 才公开，尚缺社区观察时间                                               |
| [Skvayzer/humanoid_navigation](https://github.com/Skvayzer/humanoid_navigation)                         |             0 |            0 | 2026-09-08       | 有干净 ARM64 构建记录，但为 Foxy 实机快照，不符合直接采用条件                                           |

数据来源：[GitHub 仓库 API（DeepGlint）](https://api.github.com/repos/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)、[主线提交](https://api.github.com/repos/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/commits?sha=main&per_page=1)、[humble 提交](https://api.github.com/repos/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/commits?sha=humble&per_page=1)。其他各行按 `https://api.github.com/repos/{owner}/{repo}` 及对应默认分支提交接口查询，仓库链接可查看公开状态。

### 2.2 怎样使用这些信号

- **Star / Fork 是重要筛选项。** 更高关注度通常意味着更有机会找到讨论、分支和他人经验，但 Fork 不等于成功部署，Star 不等于测试人数。
- **低 Star 完整方案有更大的未知复现风险。** 隐藏依赖、作者现场配置、未覆盖固件和无人响应的问题，都可能导致投入难以预测；不能因 README 功能齐全就优先。
- **低 Star 也不等于代码差或问题无解。** 新发布项目尚未积累关注度；干净构建与跨机器复现记录可以补充证据。
- **高 Star 仍要核对具体平台和维护。** DeepGlint 的 930 Star 不证明其 ARM64 Open3D 依赖已解决，也不证明导航集成已完成；最近提交中的文档修改不代表近期算法维护。
- **整体可靠性取决于最薄弱的集成部分。** 即使选用 DeepGlint 和 Nav2，也不能把两者的关注度转移给新写的 TF、地图转换和运动桥代码。

选型顺序调整为：**环境符合 → 核心算法与框架的社区基础 → 目标平台复现和问题处理证据 → 缺失模块及补齐成本 → 使用流程完整度。**

## 三、优先路线：DeepGlint humble 分支补齐完整导航

### 3.1 为什么提升为主选基础

- 仓库：[FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)，使用实际名为小写 [`humble`](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/tree/humble) 的分支。
- 在本次 G1 专用候选中，930 Star / 88 Fork 的关注度明显更高；已有 G1 定位、倒装传感器处理和离线地图工作流。
- 适合作为定位核心保留，通过 [Nav2](https://github.com/ros-navigation/navigation2) 补全规划与导航，不从低关注项目整体接管全部算法和运控逻辑。
- 这是**拟议集成方案**，不是该仓库上游已经提供的完整功能。预计整体难度中高，需要熟悉 ROS 2、TF 和 Nav2 的开发者；不适合要求导航初学者独立拼装。

### 3.2 高 Star 也没有消除的风险

1. `humble` 分支 README 仍保留 ROS 1 的 `catkin_make` / `roslaunch` 命令，而代码中已有 ROS 2 Python launch；需要整理正确的 Humble 使用文档。
2. `Open3D_DIR` 和地图路径等有现场配置；Open3D 二进制、ABI 与 CPU 架构需要固定。README 将 ARM 预编译包标为未充分测试，公开 [Issue #7](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/issues/7) 有 ARM64 部署反馈。
3. 公开 [Issue 列表](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/issues) 仍有 Humble 使用、样例数据失效和 Open3D 包获取问题；[Issue #3](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/issues/3) 也直接讨论缺少导航部分。不能假设维护者会及时解决新问题。
4. GitHub 元数据没有识别出仓库级许可证；这不等于断言所有文件无许可证，但再分发或商业交付前须厘清仓库和依赖各自授权。

因此先评估定位核心在目标架构上的可复现性，再决定是否进入导航补齐。若目标是 x86 服务器，可优先评估文档测试过的 x86 路径；若要求板载 ARM64，不能跳过该项。

### 3.3 最小补齐方案与交付范围

初期范围限定为**室内平地、人工给初始位置、RViz 单目标点导航、沿用原厂步态**；暂不引入楼梯、自动全局重定位、语音、双臂或自训练步态。

| 模块              | 已有基础                              | 需要新增或调整                                                                                                                      | 难度判断           |
| ----------------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- | ------------------ |
| Humble 构建与依赖 | `humble` 分支的 ROS 2 节点与 launch | 固定提交和 Open3D / Livox 依赖，参数化硬编码路径，补齐 Humble 构建与运行说明                                                        | 中；ARM64 可能较高 |
| 建图与地图资产    | FAST-LIO、PCD 地图流程                | 明确建图入口、保存入口；生成与 PCD 同原点和朝向的二维占据栅格（YAML / PGM），保留地图元数据                                         | 中                 |
| 全局定位与里程计  | FAST-LIO 与 `open3d_loc`            | 核对已有 `map→odom`、里程计与机身外参，适配 Nav2 所需连续里程计和导航基座 frame，防止多个节点重复发布 TF                         | 中高               |
| 话题兼容          | 上游点云与定位输出                    | 隔离命名空间或 remap：`open3d_loc` 的 `/map` 是点云，不能与 Nav2 栅格 `/map` 同名；`/scan` 也是点云，不能被误当作 LaserScan | 中                 |
| 规划与避障        | Humble Nav2                           | 接入 map_server、规划器、控制器、代价地图；实时点云经高度与机身过滤后输入障碍层，按需要转换 LaserScan；配置 G1 轮廓和速度约束       | 中高               |
| G1 运动执行       | 官方 SDK2 的 G1 高层 `LocoClient`   | 新增 `/cmd_vel` 到高层速度接口的薄适配层，包含限幅、超时、定位失效阻断、停止和明确使能；使用兼容原厂步态的模式                    | 中高，需实机确认   |
| 完整使用入口      | RViz 与上游初始位姿接口               | 打包建图 / 定位导航 launch、参数、RViz 配置、地图保存和停止流程；统一启动顺序与故障提示                                             | 中                 |

推荐的职责划分：

```text
MID-360 + IMU → FAST-LIO → 连续里程计 / 实时点云
                       └→ open3d_loc + 已有 PCD → 全局定位修正

与 PCD 对齐的二维地图 → Nav2 map_server → Nav2 规划与控制
过滤后的实时点云 ─────────────────────→ 局部障碍代价地图
Nav2 /cmd_vel → G1 高层速度适配 → 官方 LocoClient → 原厂步态
```

`open3d_loc` 的 Humble 源码已经有 `map→odom` 发布逻辑，不应未经检查再写一个重复的定位 TF 发布器。若以 DeepGlint 提供全局定位，则不同时启动 AMCL 争夺 `map→odom`。三维地图定位与二维导航地图必须处在同一地图坐标系；二维平面化也须保留正确的机身到雷达关系。

上述难度为调查判断，不是工期承诺。主要不确定性是 Open3D 架构兼容、TF / 时间语义、地图对齐与固件运控接口；并非简单添加一个 Nav2 launch 就完成。

实现与验收参考：[Humble 定位源码](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/blob/humble/open3d_loc/src/global_localization.cpp)、[构建配置](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/blob/humble/open3d_loc/CMakeLists.txt)、[定位 launch](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/blob/humble/open3d_loc/launch/open3d_loc_g1.launch.py)、[Nav2 坐标系要求](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms.md)、[G1 高层接口示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_loco_client_example.cpp)。Nav2 概念文档可参考新版，但实现和参数须以 Humble 版本为准。

### 3.4 补齐后的验收标准

- 在指定 CPU 架构的干净 Ubuntu 22.04 / Humble 环境可重复构建，公开依赖可获取。
- 保存地图后重启，从不同位置人工初始化，定位能与地图对齐；定位失效时禁止运动。
- Nav2 使用正确栅格和 TF，实时障碍能进入代价地图并触发停车或重规划。
- 单目标点导航、取消、断流和停止均有完整记录；不能只提供 RViz 地图截图。
- 将最终代码、依赖版本、launch、参数和中文说明作为一个集成包交付，使用者无需逐个拼装。

## 四、低关注完整方案：作为对照与有条件备选

### 1. Humble 完整应用备选：unitree-g1-slam-nav-guide

- 仓库：[zyli6269-cmyk/unitree-g1-slam-nav-guide](https://github.com/zyli6269-cmyk/unitree-g1-slam-nav-guide)
- 目标环境：Ubuntu 22.04 ARM64、ROS 2 Humble、G1 PC2、Livox MID-360。
- 已组织的模块：FAST-LIO 建图、静态地图定位、导航规划与避障、G1 C++ 速度桥、RViz 操作面板、航点任务；导览讲解和语音为附加功能。
- 适合初学者的原因：提供建图、导航、停止入口及中文说明，可以围绕完整应用流程理解系统，减少自行集成模块的工作。
- 真机依据：作者声明建图、3D 导航、航点讲解等已在 G1 实机可用，并记录了 PC2 软件快照来源。
- 成熟度限制：部署说明明确没有在另一台机器完整重装；公开仓库不包含现场地图及部分二进制依赖，需要按说明获取。部署说明部分内容仍描述现场压缩包，不能直接当作公开 Git 仓库的完整交付清单。
- 楼梯、多层导航和动态避障保留为作者能力描述，不能仅凭 README 认定通过系统测试。首轮选型重点看平地建图、重新定位和目标点导航。

**判断：符合“完整中文应用”方向，但仅 1 Star / 0 Fork、公开时间很短。降为集成对照与有条件备选，不能凭功能清单压过社区基础更强的核心方案。**

依据：[README](https://github.com/zyli6269-cmyk/unitree-g1-slam-nav-guide/blob/master/README.md)、[部署说明](https://github.com/zyli6269-cmyk/unitree-g1-slam-nav-guide/blob/master/部署说明.md)、[第三方依赖](https://github.com/zyli6269-cmyk/unitree-g1-slam-nav-guide/blob/master/dependencies/README.md)。

### 2. Foxy 参考：Skvayzer/humanoid_navigation（不符合主选环境）

- 仓库：[Skvayzer/humanoid_navigation](https://github.com/Skvayzer/humanoid_navigation)
- 目标环境：Ubuntu 20.04 ARM64、ROS 2 Foxy、G1 / MID-360 及其既有驱动与 DDS 约定。
- 已组织的模块：FAST-LIO 建图与里程计、NDT-OMP 地图定位、地图转换、Nav2、局部障碍代价地图、Unitree 高层速度网关、可视化。
- 使用流程：有安装、地图与坐标系、操作员启动、预览、使能、停止文档；网关包含限速、看门狗与 `killnav`。
- 复现依据：作者提供 2026-09-08 实机部署快照、源码与依赖校验、干净 ARM64 Docker 构建及离线导入验证记录。
- 成熟度限制：公开打包版本没有在第二台机器人上进行物理复测；干净构建通过不等于跨机器自主导航通过。
- 适配限制：依赖目标机器人已有的 `/livox/lidar`（`CustomMsg`）与 `/livox/imu`，不是包含全部硬件驱动的独立镜像；Foxy 已停止维护。

**判断：完整性和验证边界交代较清楚，但 0 Star / 0 Fork，且 Foxy 不符合当前 Humble 要求。只参考其打包与网关设计，不建议为采用它改变当前环境。**

依据：[README](https://github.com/Skvayzer/humanoid_navigation/blob/main/README.md)、[安装说明](https://github.com/Skvayzer/humanoid_navigation/blob/main/docs/INSTALL.md)、[验证记录](https://github.com/Skvayzer/humanoid_navigation/blob/main/docs/VALIDATION.md)。

### 3. ROS 1 参考：Unitree_G1_Navigation（不符合主选环境）

- 仓库：[binbin2002/Unitree_G1_Navigation](https://github.com/binbin2002/Unitree_G1_Navigation)
- 环境：Ubuntu 20.04、ROS 1 Noetic。
- 已组织的模块：FAST-LIO2 三维建图、双重 ICP 重定位、二维栅格导航（A* + TEB）、Unitree SDK 运控，另含语音交互。
- 使用流程：中文说明覆盖建图、保存与编辑地图、启动导航、启动运控以及 RViz 目标点操作。
- 局限：文档存在旧目录名和命令不一致；未在本次调查中验证依赖安装和全新环境复现。选择它意味着接受 ROS 1 技术栈，不应假设能直接按 ROS 2 Humble 使用。

**判断：20 Star / 4 Fork 的完整应用参考，但 ROS 1 Noetic 不符合当前环境；不建议为了采用它增加 ROS 1 / ROS 2 桥接或移植工作。**

依据：[README](https://github.com/binbin2002/Unitree_G1_Navigation/blob/main/README.md)。

## 五、仿真候选与迁移边界

### GrayStars：仿真起步候选

- 仓库：[g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-](https://github.com/GrayStars/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-)
- 主线环境：Ubuntu 22.04 + ROS 2 Humble；另有 Foxy 分支。
- 集成内容：MuJoCo、MJLab G1 步态、模拟 MID-360、ROS 2 桥、slam_toolbox、FAST-LIO 和 Nav2；作者建议采用整包工作区以保留兼容补丁。
- 使用门槛：README 仍要求分别准备和构建 MuJoCo、G1 控制器及相关依赖，并修改部分工作区路径。原文的几行 `rosdep + colcon` 不能代表完整安装流程。
- 关键缺口：本次核查的默认 `nav.launch.py` 启动 FAST-LIO 建图节点，`fastlio_tf_bridge.py` 默认将 `map→odom` 固定为单位变换；这条链路没有加载 PCD 做地图配准，也没有启动 AMCL。仓库中存在其他定位代码，不等于默认导航入口已经接入。
- 因此，应区分“同一初始位置下的仿真导航演示”与“保存地图后重启、换起点仍能重定位导航”。后者需要额外证据。

**判断：可用于体验仿真建图与导航，不宣称是成熟的保存地图重定位或仿真到真机完整交付方案。**

依据：[README](https://github.com/GrayStars/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/blob/main/README.md)、[默认导航入口](https://github.com/GrayStars/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/blob/main/src/mujuco_sim/launch/nav.launch.py)、[TF 桥](https://github.com/GrayStars/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/blob/main/src/mujuco_sim/mujuco_sim/fastlio_tf_bridge.py)。

### 其他仿真与研究候选

| 仓库                                                                   | 内容                                                                                                                      | 当前需求下的定位                                                                         |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| [grove-g1](https://github.com/Adyansh04/grove-g1)                       | Jazzy 综合自主栈，包含 SLAM Toolbox、AMCL、Nav2、FAST-LIO、步态、双臂和任务编排；README 快速开始要求 Docker / NVIDIA 环境 | 功能广，但复杂度高；主线自行运行步态并发送低层 `lowcmd`，不作为初学者真机首选          |
| [uleroboticsgroup/rl_hnav](https://github.com/uleroboticsgroup/rl_hnav) | Humble 下结合 MuJoCo / Gazebo、SLAM、Nav2、RL 步态和真机模式，提供 dry-run 等分阶段模式                                   | 值得研究仿真到真机架构；双仿真器及 Gazebo Classic 增加维护成本，不能据功能描述认定更成熟 |

仿真与真机可以复用算法结构和部分接口，但传感器消息、时间、坐标系、地图定位和运动执行仍可能不同。**本次可接受有开发支持、范围明确的适配，但应输出统一使用入口，不能把集成责任转给导航初学者。**

## 六、其他仓库的定位

| 仓库                                                                                                                         | 保留价值                                     | 当前选择                                                                                              |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| [DeepGlint/FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)                       | G1 地图定位与传感器适配参考，有 Humble 分支  | 提升为主选核心，按第三节补齐；不将原仓库本身称为完整导航方案                                          |
| [guruadp/unitree-g1-slam-nav](https://github.com/guruadp/unitree-g1-slam-nav)                                                 | Humble 下建图与 Nav2 最小集成参考            | 本次所见公开资料不足以支持成熟首选；时间戳覆盖等工程处理有适用边界                                    |
| [yanghc18/G1-Official-SLAM-Indoor-Navigation](https://github.com/yanghc18/G1-Official-SLAM-Indoor-Navigation)                 | 研究 G1 固件 `slam_operate` 服务和官方接口 | README 的已验证流程明确只完成建图、保存、重定位，当时没有测试自主导航；不能据仓库名认定完整导航已验证 |
| [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)                                           | 官方 G1 仿真基础                             | 不含完整 SLAM / 导航应用                                                                              |
| [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar)、[unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) | 步态策略仿真与部署基础                       | 主要解决运动策略，不是完整导航应用                                                                    |

官方固件导航服务可作为另一条选择路径，但须确认目标 G1 是否提供兼容服务及完整操作支持。社区接口示例的验证范围不能替代官方产品能力或目标机器验收。

## 七、原调查中需要纠正的技术表述

这些修正用于判断仓库可信度，不要求在当前环境实施。

1. **地图定位没有“永不漂移”的保证。** DeepGlint 通过已有地图配准约束里程计漂移，仍依赖地图、初值和匹配成功率。
2. **倒装不等于所有机器都要额外旋转 180°。** 先确认驱动是否已转换坐标；重复修正会出错。IMU 加速度在倒装坐标系中的符号变化本身不代表故障。应一致处理点云、IMU、外参与 frame。
3. **时间戳覆盖不是传感器同步。** guruadp 的 `scan_restamper` 用当前 ROS 时间覆盖扫描时间，可能缓解 TF 查询失败，却不恢复采样时间，也不解决 LiDAR–IMU 同步。`use_sim_time` 用于选择 `/clock`，不是实机时钟同步开关。
4. **话题名相同不等于消息兼容。** 本知识库 FAST-LIO 模板将文档中标为 `PointCloud2` 的话题配给 `lidar_type: 1`，而所引用 Ericsii 上游在该模式订阅 `CustomMsg`；这组默认值不能作为已验证接入方案。逐点时间字段也需要匹配。
5. **内置 IMU 不代表 LiDAR–IMU 平移一定为零。** FAST-LIO 上游 MID-360 配置有非零平移；应依据实际驱动输出坐标系采用正确外参。雷达到机身外参是另一层关系。
6. **`/dog_odom` 和 `robot_center` 是特定方案约定，不是所有 G1 的固定接口。** 不应将单个项目配置推广到全部固件。
7. **G1 高层运动接口应以官方 `g1::LocoClient` 为准。** 官方 G1 服务名本身为 `sport`；guruadp 桥使用 G1 速度 API `7105`，不能仅凭文件名 `cmdvel_to_sport` 判断它误用了四足接口。
8. **底层仓库按方案需要选择，不是全部必须安装。** `unitree_slam` 是官方接口与示例，不能当作完整开源 SLAM 引擎；自建方案不必依赖它。

核查来源：[DeepGlint README](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID/blob/main/README.md)、[scan_restamper](https://github.com/guruadp/unitree-g1-slam-nav/blob/master/src/g1_nav_bridge/g1_nav_bridge/scan_restamper.py)、[ROS 时间机制](https://design.ros2.org/articles/clock_and_time.html)、[Ericsii 消息订阅实现](https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/src/laserMapping.cpp)、[FAST-LIO MID-360 外参](https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml)、[G1 官方 API](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_api.hpp)、[unitree_slam](https://github.com/unitreerobotics/unitree_slam)。

## 八、最终选型前只需确认的事项

以下是后续向维护者核实或评估交付的标准，不是本地部署任务：

- [ ] 固定 Ubuntu 22.04 + ROS 2 Humble，进一步确认 x86 / ARM64、G1 自由度及固件。
- [ ] 同时记录 Star / Fork、分支维护、公开问题处理和独立复现，不能只依据功能清单。
- [ ] 若采用 DeepGlint 补齐路线，有负责集成的开发者及明确交付边界，最终提供完整使用流程。
- [ ] 公开仓库及公开依赖能组成完整方案，不依赖作者未发布的私有包或现场压缩包。
- [ ] 有“建图保存 → 重启 → 从不同位置重定位 → 目标点导航 → 障碍处理 → 停止”的连续演示或记录。
- [ ] 了解作者验证、干净构建验证、第二台机器复现之间的区别，记录实际证据等级。
- [ ] 如果要求仿真和真机使用同一套方案，仓库必须分别提供对应入口、配置和验证依据，不能由接口相似推断已经贯通。

**本次建议：在 Ubuntu 22.04 / ROS 2 Humble 下，优先评估 DeepGlint `humble` 分支 + Nav2 + 官方 G1 高层接口的可控补齐路线；低 Star 完整仓库用于对照和有条件备选。GrayStars 保留为仿真候选。若没有集成开发支持，则现有证据不足以承诺任何候选兼具高可靠性和开箱即用，应优先争取维护者或官方的完整交付支持。**
