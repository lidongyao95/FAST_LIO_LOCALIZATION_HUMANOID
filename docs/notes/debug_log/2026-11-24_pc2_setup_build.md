# 2026-11-24 PC2 有线部署与本机构建

- 开发机通过 `unitree@192.168.123.164` 验证 PC2：aarch64、Ubuntu 20.04.6、ROS 2 Foxy。
- 当次 `sic-guest` DHCP 地址为 `172.18.14.107/20`，非固定无线地址。
- 标准入口为 Mac 仓库根目录的 `scripts/real_robot/00_pc2_setup.sh check|sync|build`，详见 `docs/WORKFLOWS.md` B0。
- 本轮按用户要求，以本地源码覆盖 PC2 并删除同步范围内远端多余源码，排除构建产物、依赖二进制和运行数据；checksum dry-run 校验通过。因保护缓存/构建文件而保留的旧目录不代表仍保留旧源码。
- 构建前停止本仓库运行会话，构建脚本自身遇域 223 冲突会退出，不自动清理或恢复会话。
- 补齐通用构建入口对 `.deps/arm-open3d/open3d141/lib/cmake/Open3D` 的识别，构建时恢复 Livox ROS 2 包清单。
- 首轮后台增量构建：`livox_ros_driver2`、`open3d_loc`、`fast_lio` 全部 Finished，colcon 总耗时 10.9 秒；随后 `robot` 自检通过，最终退出码 0。
- 首轮证据：PC2 `~/ldy/g1_real_data/records/builds/pc2-20261124-140425-3avGBN/`，其中 `build.log`、`pid`、`status` 保留原始输出。
- 构建成功不表示现场传感器、定位、规划或行走已通过验收；本轮没有重启运行会话或启用硬件控制。

- 将架构/动态库验证加入脚本后的端到端复验：三个包构建完成（8.27 秒）、robot 自检通过，三个程序均为 ARM aarch64，动态库检查全部通过，退出码 0。最终证据目录：`~/ldy/g1_real_data/records/builds/pc2-20261124-140532-ZLN3A6/`。

## 离线依赖与石膏网页修复

- 系统 Python 原有 SciPy 1.3.3、Matplotlib 3.1.2，缺 scikit-image；旧自检合并导入并隐藏错误，误导为三项全缺。改为逐项报告实际错误、解释器、版本及路径。
- 清华 ubuntu-ports apt update 成功，但安装被既存 `chrony`/`systemd-timesyncd` time-daemon 冲突阻断；`apt-get -s --fix-broken install` 显示将移除 chrony，未执行修复或移除服务。
- 使用 `/usr/bin/python3 -m venv --without-pip --system-site-packages .deps/offline-venv`，通过清华 PyPI 二进制轮子安装 NumPy 1.24.4、SciPy 1.10.1、scikit-image 0.21.0、Matplotlib 3.7.5。未修改系统 Python 包。离线自检和 export/dufo 自动选择该环境；sync 保护此本机环境。
- 从 `B1_run03.pcd` 给对应 `B1_run03_map` 目录补充 `plaster.png`、`surface.ply`、`plaster.json`：保留 44,621/110,263 点，生成 207,492 三角面；二维图不变。
- 网页 review 新增石膏图区域和缺失状态。更新服务后通过浏览器确认石膏图可见；只读 HTTP 图像接口验证通过。

## Open3D 链接依赖前移到预检

- 现象：`open3d_loc` 重新链接时报缺 `libtmglib.so.3`。根因不是库缺失，而是 PC2 的 Open3D 运行库解包在 `.deps/arm-open3d/runtime` 里，系统 `ldconfig` 看不到；旧构建脚本只在目录存在时静默追加 `-Wl,-rpath-link`，目录判断失败就到链接期才暴露。
- 新增 `scripts/env/open3d_runtime_probe.sh`：对 `.deps` 本地库逐个 `readelf -d` 取 NEEDED，与本地同名库及系统 `ldconfig` 结果比对；缺失时打印库名、依赖方和对应 apt 包，`--fix` 可免 root 用 `apt-get download` + `dpkg-deb -x` 解包补齐。退出码 0 全部可解析、1 有缺失、2 未发现本地 Open3D。
- 两处踩坑：① 收集库时必须 `-type f -o -type l`，否则 `libtmglib.so.3` 这类符号链接被跳过而误报；② 构建脚本会 `export PATH=/usr/bin:/bin`，而 `ldconfig` 在 `/sbin`，导致系统库整体误判缺失（一度把所有 `libc/libblas/liblapack` 都报成缺）。已改为按 `command -v ldconfig` → `/sbin` → `/usr/sbin` 查找，并在取不到时回退扫描 `/lib/<arch>-linux-gnu` 等目录。
- 接线三处：`build_g1_ros2.sh` 构建 `open3d_loc` 前预检并把 rpath-link 固定传给 CMake（失败即停，不再等链接期）；`01_self_check.sh robot` 在启动定位前检查；`00_pc2_setup.sh check` 在 `ldd` 段之后输出预检报告。
- 复验：限制 `PATH=/usr/bin:/bin` 下预检通过（5 个本地库、传递依赖全部可解析）；负例（临时目录只放 `liblapacke.so.3.9.0`）准确报 `libtmglib.so.3` 缺失、退出码 1；PC2 全量增量构建 3 包 Finished（10.7 秒），`global_initialize` / `global_localization_node` 的 `link.txt` 均含 `-Wl,-rpath-link,<runtime 目录>`，`ldd` 无缺失，`robot` 自检通过。
