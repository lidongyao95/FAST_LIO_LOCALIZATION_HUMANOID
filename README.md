# G1 建图、定位与导航

让宇树 G1 人形机器人在已知地图里自己走到目标点。用 FAST-LIO 建图，Open3D 定位，Nav2 规划和避障。

可以先在 MuJoCo 仿真里跑通，再部署到真机。

## 从哪里开始

| 你想做什么 | 看哪个文档 |
| --- | --- |
| 第一次用，装环境 | [环境配置](docs/setup.md) |
| 没有机器人，在仿真里建图和导航 | [纯仿真](docs/simulation.md) |
| 在真实 G1 上建图和导航 | [真机部署](docs/robot.md) |

## 命令一览

所有操作都通过仓库根目录的 `./g1_slam` 完成：

```bash
./g1_slam setup            # 安装依赖并编译
./g1_slam sim map          # 仿真：建图
./g1_slam sim nav          # 仿真：定位和导航
./g1_slam test             # 运行测试

./g1_slam robot map 地图名  # 真机：建图
./g1_slam robot nav 地图名  # 真机：定位并打开导航网页
```

不带参数运行 `./g1_slam` 查看完整列表。

## 目录

| 目录 | 内容 |
| --- | --- |
| `packages/` | 两个 ROS 包：`fast_lio`（里程计和建图）、`open3d_loc`（地图定位） |
| `shared/` | 仿真和真机共用：环境、编译、地图导出、Nav2 配置 |
| `sim/` | MuJoCo 仿真器、行走控制器、虚拟雷达 |
| `robot/` | 真机上用的建图、定位、导航网页和宇树 SDK 程序 |
| `tests/` | 单元测试和仿真场景测试 |
| `docs/notes/` | 历史调试记录 |

## 来源

本项目基于 [deepglint/FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)，里程计和定位分别来自 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 和 [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)。

原项目贡献者：Axiang Sun 孙阿祥、朱峻峰、Yihao Xu 许益豪、Xin Li 李鑫、Zhongxia Zhao 赵仲夏、Gaohao Zhou 周高豪、Dongliang Li 李栋梁、Xiang An 安翔、Ziyong Feng 冯子勇、Huajie Tan 谭桦杰。

引用：Yihao Xu, Xin Li, Zhongxia Zhao, Gaohao Zhou, Dongliang Li, Xiang An, Huajie Tan, Ziyong Feng (2025). FAST_LIO_LOCALIZATION_HUMANOID [Source code].
