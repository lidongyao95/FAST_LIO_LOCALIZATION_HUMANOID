# G1 建图、定位与导航

本仓库使用 FAST-LIO、Open3D 和 Nav2，为 Unitree G1 提供建图、定位与导航链路。开始前先选择使用路线：**让虚拟 G1 在 MuJoCo 中走，或在真实 G1 的 PC2 上接入现场数据。**

## 选择使用路线

| 你现在要做什么 | 在哪里运行 | 从哪里开始 |
| --- | --- | --- |
| 没有机器人，先跑通建图、定位和目标导航 | Ubuntu x86_64 / ROS 2 Humble 仿真机 | [纯仿真路线](docs/WORKFLOWS.md#纯仿真路线) |
| 在真实 G1 上采集地图、持续定位和查看网页规划 | PC2 / ARM64 / ROS 2 Foxy；Mac 用 SSH 操作 | [实机部署路线](docs/WORKFLOWS.md#实机部署路线) |
| 已有实机点云，想在近似现场中预演导航 | Ubuntu 仿真机，仍运行虚拟 G1 | [实机地图的仿真预演](g1_real_robot_docs/scan_mujoco_preview.md) |
| 已有实机录包，想复查传感器输入或重新建图 | 与录包兼容的 Ubuntu / ROS 环境 | [离线回放](g1_real_robot_docs/inverted_mid360.md) |
| 清理多帧扫描中的动态点，再导出地图 | 装有 DUFOMap 的离线处理环境，历史验证为 Linux x86_64 | [DUFOMap 动态点清理](docs/WORKFLOWS.md#用-dufomap-清理多帧扫描) |

### 纯仿真与实机地图预演有什么区别

纯仿真从内置虚拟房间开始，用模拟 LiDAR/IMU 建图，再重启虚拟机器人做定位与导航。
实机地图预演从现场采集的点云开始，把点云近似成 MuJoCo 场景，再让虚拟机器人在其中运行。**它的场景素材来自实机，运行时的雷达、IMU、机器人和运动仍是仿真的。**

预演是实机部署中的可选检查，不是接通真实 G1 的最后一步。回到 PC2 后，仍需使用现场地图、实测外参和实时传感器，分别验证定位、规划与实际运动。[完整区别及操作步骤](docs/WORKFLOWS.md)。

## 常用文档

| 内容 | 文档 |
| --- | --- |
| 两条路线的命令、前置条件、完成判据 | [使用流程](docs/WORKFLOWS.md) |
| 脚本目录职责、常用入口与底层工具的关系 | [脚本索引](scripts/README.md) |
| Humble 仿真环境安装、构建与配置参考 | [仿真环境与配置](docs/CONFIGURATION.md) |
| 逐终端调试 MuJoCo、控制器、定位与 Nav2 | [仿真手动运行手册](docs/NAVIGATION_MANUAL.md) |
| PC2 部署资料与各次实机验证记录 | [实机文档入口](g1_real_robot_docs/README.md) |
| 历史仿真结果及其适用范围 | [仿真验证记录](docs/VALIDATION.md) |

当前实机入口只提供感知、持续定位与网页规划，不启动运动控制链。网页的“规划考虑实时障碍”控制实时点云是否参与全局规划；实际自主行走接口需要另行实现与现场验收。历史测试保存在[行走记录](g1_real_robot_docs/2026-11-10_g1_autonomous_walk_plan.md)，其中旧命令不作为当前入口。

## 项目来源

本项目基于 [deepglint/FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)，里程计与定位分别参考 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 和 [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)。依赖版本和本地修改见[配置说明](docs/CONFIGURATION.md#2-仓库与代码来源)。

原项目贡献者：Axiang Sun 孙阿祥、朱峻峰、Yihao Xu 许益豪、Xin Li 李鑫、Zhongxia Zhao 赵仲夏、Gaohao Zhou 周高豪、Dongliang Li 李栋梁、Xiang An 安翔、Ziyong Feng 冯子勇、Huajie Tan 谭桦杰。

学术引用：Yihao Xu, Xin Li, Zhongxia Zhao, Gaohao Zhou, Dongliang Li, Xiang An, Huajie Tan, Ziyong Feng (2025). FAST_LIO_LOCALIZATION_HUMANOID [Source code]. 原项目链接见上。
