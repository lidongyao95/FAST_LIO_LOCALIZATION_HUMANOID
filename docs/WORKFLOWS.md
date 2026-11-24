# G1 使用流程：纯仿真与实机部署

先选路线，再执行该路线中的步骤。脚本名中的 `01`～`05` 是入口编号，**不表示所有用户都要从 1 顺序执行到 5**。纯仿真使用自检和 `03_simulation.sh`；实机部署使用自检、建图、定位和 Web，可按需插入地图预演。

直接跳转：[纯仿真路线](#纯仿真路线) · [实机部署路线](#实机部署路线) · [DUFOMap 动态点清理](#用-dufomap-清理多帧扫描) · [地图处理选项](#地图处理的常用选项) · [域与日志](#ros-域日志与结束会话)

## 先分清四种运行方式

| 方式 | 环境和数据从哪里来 | 谁在运动 | 用于判断什么 |
| --- | --- | --- | --- |
| 纯仿真 | 内置 MuJoCo 房间，运行时生成虚拟 LiDAR/IMU | MuJoCo 中的 G1 | 软件链能否完成建图、重定位、导航、障碍与取消回归 |
| 实机地图的仿真预演 | 实机 PCD 近似成场景，运行时重新生成虚拟 LiDAR/IMU | MuJoCo 中的 G1 | 近似现场中的场景、地图、定位和规划是否配套 |
| 实机录包回放 | 回放已经录下的 LiDAR/IMU，不重新模拟传感器 | 不驱动机器人；数据轨迹固定 | 历史输入上的建图、定位与时间/TF 问题能否复现 |
| PC2 实时运行 | 现场 MID-360 和 IMU，经 FAST-LIO 提供实时点云与里程计 | 默认由操作员在网页显式点“开始实机导航”后经导航 SDK 驱动；`--plan-only` 只提供规划 | 现场输入、定位、地图、代价地图、规划与实机速度链是否可用 |

**根本区别是运行时的输入和控制对象，而不只是地图是不是来自实机。** 回放中给 Nav2 一个新目标不会改变录包轨迹；扫描场景预演可以闭环运动，但运动发生在仿真中。两者均不能代替实机运动验收。

以下命令均从仓库根目录执行。Mac 可以做离线地图处理和 SSH 操作；MuJoCo/ROS 步骤在对应 Ubuntu 主机执行。脚本的 `--help` 列出参数。除录制、导出、自检等有限任务外，会话留在前台，按 Ctrl+C 停止本次启动的进程。

## 启动前的进程预检

每次重启或切换阶段时，先在实际运行 ROS 的 Linux 主机检查旧会话。采集入口检查建图和定位进程，持续定位入口检查旧定位，Web 入口检查旧导航；仿真统一入口检查整套会话。检查发现冲突会列出 PID 并停止本次启动，加载环境本身不清理进程。

以重启实机网页导航为例，先设置 B1 的 ROS 域，再执行：

```bash
# 默认只列出本仓库、当前用户、当前 ROS 域的导航进程。
./scripts/env/g1_cleanup_stale.sh --list --scope navigation
# 优先在旧启动终端 Ctrl+C；终端已丢失时，再显式清理清单中的进程。
./scripts/env/g1_cleanup_stale.sh --stop --scope navigation
./scripts/env/g1_cleanup_stale.sh --check --scope navigation
```

`navigation` 保留正在提供数据的雷达、FAST-LIO 和 Open3D；重启定位用 `localization`，建图用 `mapping`。结束整个仿真会话后，可检查 `--scope all`。支持 `--domain N` 显式指定域；清理只依次发送 SIGINT、SIGTERM，仍未退出会报出 PID，不强制杀死。不属于本仓库的节点还需结合 ROS 图检查，并回到它的原启动终端处理。

## 纯仿真路线

目标：不用连接真实 G1，在内置房间完成“走动建图 → 保存 → 重启定位 → 目标导航”。本路线不需要先执行实机 `02_mapping.sh record`。

### A1. 准备仿真机并自检

运行位置：Ubuntu x86_64 / ROS 2 Humble。首次使用先按[仿真环境与配置](CONFIGURATION.md#首次安装的最短路径)安装和构建。

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
./scripts/workflows/01_self_check.sh sim
```

完成判据：依赖、ROS 包和仿真控制器产物检查通过。这只说明可以继续启动，不代表运行链路已验收。

如果宿主机的 ROS 缺少 `nav2`/`rviz2`、又没有 root 可以安装（例如 4622），用同一批命令加上叠加包装脚本：

```bash
./scripts/env/ros_overlay_ns.sh ./scripts/workflows/01_self_check.sh sim
./scripts/env/ros_overlay_ns.sh ./scripts/simulation/03_simulation.sh map sim/generated/room_run02
```

包装脚本把 `~/ldy/logs/*/deps` 中解包的额外 ROS 包以只读 overlay 挂进一个独立用户命名空间，其余参数和判据都不变；可用 `ROS_DEPS_DIR` 指定别的层目录。A2、A3 的 `03_simulation.sh` 命令在 4622 上同样要加这个前缀。

另外 4622 是 SSH 登录的图形会话，启动前先让子进程继承到显示器，否则 MuJoCo 和 RViz 会起不来：

```bash
# 换成自己的图形会话号（4622 上常用 :11）
export DISPLAY=:11
xdpyinfo >/dev/null && echo display-ok
```

### A2. 在虚拟房间里建图并保存

```bash
# 选择尚不存在的输出目录（已有目录会被拒绝，例如换 room_run02）。
./scripts/simulation/03_simulation.sh map sim/generated/room_run02
```

等待控制器站稳、支架释放、FAST-LIO 收到虚拟传感器数据，随后进入实时建图控制：**按住** `w/a/s/d` 前后左右移动、`q/e` 转向，松手即停（终端按键重复超时 0.45 s 就判定松开），不需要回车，也不需要输入秒数；轻点一下只走约 0.45 s，按空格或 `x` 立即停下，按回车保存地图并结束。需要长时间走就按住不放，或连续敲同一个方向键。走遍起点、目标及连接区域即可。

若把输入用管道喂给脚本（非交互终端，例如自动化脚本），会退回按行模式：写成“方向 + 秒数”如 `w 5`，空行保存。

完成判据：`/map_save` 成功，会话结束，目录内有 `map.pcd`、`map.ply`、`map.pgm`、`map.yaml` 和 `metadata.json`。PCD 来自本轮虚拟 LiDAR/IMU 的 FAST-LIO 建图；PLY 给 Open3D 定位，PGM/YAML 给 Nav2。

### A3. 重启虚拟机器人，用刚保存的地图导航

```bash
./scripts/simulation/03_simulation.sh nav sim/generated/room_run02
```

脚本启动同一个内置房间，加载 A2 的三维地图与二维图，检查定位和 Nav2 后开启仿真运动门禁。在 RViz 中用 **Nav2 Goal** 点选已建图的空闲区域，观察 MuJoCo 中的机器人、实时点云和路径是否一致。

完成判据：定位持续更新，目标被接受，虚拟 G1 执行路径并到达。仅看到地图或规划线，还不能说明闭环导航成功。需要自动记录结果时，先 Ctrl+C 结束上一轮，再运行：

```bash
./scripts/simulation/03_simulation.sh nav sim/generated/room_run02 \
  --goal 3.0 0.0 --obstacle-after 2
```

目标需位于 A2 已覆盖且可达的区域。结果写入本轮日志目录的 `nav_goal.json`；脚本检查目标状态及障碍插入结果。`--headless` 可关闭窗口，`--start X Y --start-yaw RAD` 可选择覆盖区内的新起点。手动逐组件调试见[仿真运行手册](NAVIGATION_MANUAL.md)。

## 实机部署路线

目标：用真实 G1 的传感器生成并核对现场地图，在 PC2 上持续定位，再查看网页并按需规划或由网页显式发起实机导航。地图预演在 Ubuntu 仿真机完成，属于可选支线，结束后回到 PC2 继续。

```text
网线连接 → 部署准备与无线 IP 核对 → PC2 环境与传感器 → 现场采集 → 导出并核对实机地图 → PC2 持续定位 → Web 规划 → 实机导航（FSM 501）
                                  └→ 可选：生成扫描场景 → MuJoCo 预演 ─┘
```

从头建图时，按以下顺序操作：

1. **Mac / 部署电脑：00 部署**，依次执行 `00_pc2_setup.sh check`、`sync`、`build`。
2. **转到 PC2 终端：01 自检**，执行 `01_self_check.sh robot`。即使 00 构建时自动自检通过，开始本轮采集前仍保留独立自检。
3. **同一 PC2 环境：02 建图**，执行 `02_mapping.sh record <本次新名称>`，随后导出并核对地图。
4. 地图核对后再执行 **04 持续定位 → 05 网页（规划或实机导航）**；03 是可选仿真分支。只有明确复用已核对地图时才跳过建图。

00 完成后不再通过 Mac 包装脚本启动运行链。后续脚本直接在 PC2 的本机终端或 SSH 终端执行；Mac 仅作为终端/浏览器使用。

### B0. 先接网线，再完成部署准备

每次现场接入先将 Mac 与机器人 PC2 接到有线网络，通过固定的机器人有线网段登录；不依赖上次的无线地址。以下 Mac 命令在本机终端执行，标为 PC2 的命令在 SSH 会话内执行。

先在 Mac 的仓库根目录运行整合入口，自动完成有线 SSH、PC2 有线地址、ARM64 / Ubuntu 20.04 / Foxy 身份检查，查询无线 DHCP 状态，检查进程/服务、源码与安装产物、ROS/Python/动态库依赖，并生成反向源码差异清单与本地日志：

```bash
./scripts/real_robot/00_pc2_setup.sh check
```

失败时先按输出排查，不进入源码同步或启动阶段。当前入口只读，不停止进程或服务，不同步源码、不构建。报告中的进程/自动重启服务、缺失文件、`not found` 动态库和源码差异需逐项处理；脚本执行成功只表示检查结束，不表示部署已就绪。反向差异使用内容校验，排除构建产物、运行日志和仿真生成文件；仍需在正向同步前单独预览仅存在于 Mac 的文件。后面的源码同步与构建仍按对应步骤执行。下列手动命令用于首次配置网卡或排障。

**1. 有线登录并确认设备。** Mac 有线网卡需配置一个未占用的 `192.168.123.x/24` 地址（例如确认空闲后使用 `192.168.123.100`），避开 PC2 `.164`、运控 `.161` 和雷达 `.120`；此直连网卡无需配置默认网关或 DNS。

```bash
# Mac
ping -c 2 192.168.123.164
ssh unitree@192.168.123.164
# PC2：核对有线地址、架构、系统、磁盘与当前无线连接
hostname -I
ip -4 addr show eth0
uname -m
cat /etc/os-release
df -h "$HOME"
nmcli -f NAME,TYPE,DEVICE connection show --active
```

应确认连接的是 PC2（`eth0=192.168.123.164`、aarch64、Ubuntu 20.04 / Foxy）。SSH 不通先核对网线、Mac 网卡地址和路由，保持有线连接完成下列步骤。

**2. 核对并同步仓库。** PC2 已有可用部署且本轮无源码更新时可跳过复制；首次部署先创建目录。Mac 切到本地仓库根目录执行：

```bash
# Mac
cd /Users/lidongyao/Desktop/code/SLAM/FAST_LIO_LOCALIZATION_HUMANOID
export PC2_WIRED=unitree@192.168.123.164
ssh "$PC2_WIRED" 'mkdir -p ~/ldy/FAST_LIO_LOCALIZATION_HUMANOID ~/ldy/g1_real_data/{bags,maps,records,config}'
# 后续三个 rsync 命令使用同一组排除项；不复制架构相关产物与仿真子模块。
G1_SYNC_EXCLUDES=(
  --exclude='.git/' --exclude='build/' --exclude='install/' --exclude='log/'
  --exclude='.deps/sdk*' --exclude='.deps/root/' --exclude='vendor/'
  --exclude='doc/' --exclude='tmp/'
  --exclude='.deps/src/unitree_mujoco/' --exclude='.deps/src/unitree_rl_mjlab/'
)
# 已有部署先反向 dry-run：列出 PC2 与 Mac 的差异，不直接覆盖任何一端。
rsync -ain "${G1_SYNC_EXCLUDES[@]}" "$PC2_WIRED:ldy/FAST_LIO_LOCALIZATION_HUMANOID/" ./
```

逐项确认差异来源；需要保留的 PC2 源码/配置先按清单拉回 Mac，合并后再向 PC2 同步。首次空目录可直接继续。记录本地 `git rev-parse HEAD` 和 `git status --short`，再预览并执行：

```bash
# Mac：先预览，再复制；不使用 --delete。现场地图、录包和记录单独管理。
rsync -ain "${G1_SYNC_EXCLUDES[@]}" ./ "$PC2_WIRED:ldy/FAST_LIO_LOCALIZATION_HUMANOID/"
rsync -ai "${G1_SYNC_EXCLUDES[@]}" ./ "$PC2_WIRED:ldy/FAST_LIO_LOCALIZATION_HUMANOID/"
```

若明确以本地为唯一基准、无需保留远端源码改动，可用整合脚本替代上面的手动同步：

```bash
# Mac 仓库根目录：有线核对后直接同步，不要求先审查反向差异。
./scripts/real_robot/00_pc2_setup.sh sync
```

`sync` 覆盖远端同名源码，并删除同步范围内仅远端存在的旧文件；排除项保护 `.git`、构建产物、依赖二进制、现场运行日志和仿真子模块，`~/ldy/g1_real_data` 不在同步目录内。完成后自动以 checksum dry-run 校验。它不会停止进程、构建或重启程序，后续需更新受影响的本机安装。只有 `check` 模式保持全程只读。

**3. 在 PC2 完成环境 setup。** 首次部署、换机或更新涉及编译的源码时，先读[PC2 历史问题汇总](../g1_real_robot_docs/debug_log/2026-11-05_g1_pc2_debug_summary.md)，再按[PC2 部署指南第 4 章](../g1_real_robot_docs/g1_real_robot_slam_pc2_b300_guide.md#4-建立可复现的软件环境)核对 Foxy、ARM64 本机依赖与构建配置。源码 rsync 副本不带 Git 元数据，需核对 Livox SDK / 驱动源码及版本是否齐全，不能假定能直接运行 submodule 初始化。已有部署优先补齐缺失依赖并只构建受影响包；Mac 的 `build/`、`install/`、SDK 和预编译库不能直接用于 PC2。

远端构建必须使用 `--event-handlers console_direct+`，后台运行并保存日志；每 20–30 秒检查构建进程、日志增长及 `Finished` / `Failed` 终态，失败定位后再修复。不要原样套用 x86/Humble 的依赖路径。部署完成后执行 B1 自检，再启动感知与定位。

源码同步后，在 Mac 用同一入口执行 PC2 本机构建：

```bash
./scripts/real_robot/00_pc2_setup.sh build
```

`build` 只构建已经同步到 PC2 的源码，固定 ROS 2 Foxy、ARM64 本机依赖和两路编译，依次增量构建 `livox_ros_driver2`、`fast_lio`、`open3d_loc`；恢复 Livox 的 ROS 2 `package.xml`，识别 PC2 的 `.deps/arm-open3d/open3d141`。它先检查域 223 的冲突进程，发现冲突则退出；在原启动终端结束会话后再执行，不自动清理或重启。构建锁避免同时运行两轮构建。

远端 `pc2_build.sh` 使用 nohup 后台任务，日志、PID 和最终退出码保存在 `~/ldy/g1_real_data/records/builds/pc2-*/`；构建保持 `console_direct+`。每秒检查完成/进程退出，每 25 秒输出进程存活、日志大小/增长和包终态；完成立即报告，失败提取错误线索，不自动重试。SSH 断开后构建继续，可在有线恢复后读取最新目录的 `build.log`、`pid`、`status`，确认终态后再决定操作，不重复启动。

三个包完成后自动执行 `01_self_check.sh robot`，核对三个可执行文件为 ARM64、`ldd` 无缺失动态库；构建或验证失败都会返回非零。通过只证明依赖和安装检查完成；仍需后续定位/传感器现场验证。本步骤不安装缺失系统依赖、不启动行走或恢复旧会话。

PC2 的 Open3D 是解包到 `.deps/arm-open3d` 的 ARM 运行库，`liblapacke` / `libtmglib` 只在本地目录可见，系统 `ldconfig` 查不到；缺了它们链接 `open3d_loc` 才会报 `cannot find -ltmglib` 一类错误。因此构建入口现在先跑 `scripts/env/open3d_runtime_probe.sh`：逐个 `readelf -d` 核对本地库的传递依赖，全部可解析才把 `-Wl,-rpath-link,<runtime 目录>` 固定传给 CMake；不完整时直接打印缺哪个库、对应 apt 包和 `--fix` 命令并停止，不再等到链接期才失败。同一预检也接在 `00_pc2_setup.sh check`（构建后 `ldd` 段之后）和 `01_self_check.sh robot`（启动定位前），所以部署检查、自检、构建三处都会提前暴露这个问题。

**4. 通过有线会话查询当次无线 IP。** 无线用于后续远程访问时，先确认 PC2 已连接 `sic-guest`；未连接则先在 PC2 配置该 Wi-Fi，再查询：

```bash
# PC2
nmcli -f GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY device show wlan0
ip -4 addr show wlan0
nmcli -f ipv4.method,ipv4.addresses connection show sic-guest
```

`sic-guest` 应为 `ipv4.method=auto`，未设置静态地址。无线使用 DHCP，`172.18.14.107` 只是 2026-11-24 的实测租约，不能写成固定连接目标。记录本次 `wlan0` 地址，在 Mac **另开终端**执行 `ssh unitree@<当次无线IP>`，核对 `hostname -I` 确认仍是同一台 PC2。首次使用新地址时核对已有可信主机公钥。只有无线 SSH 验证成功且后续操作确需无线时才拔网线；也可全程保留有线。无线不通时继续用有线排查，不修改机器人/雷达有线地址。换电、重启、重连 Wi-Fi 后重新查询。

完成判据：有线 SSH 可用，源码与本机安装匹配、B1 自检通过；若要切无线，还需当次 DHCP 地址和无线 SSH 验证通过。

### B1. 在 PC2 设置环境并执行 01 自检

运行位置：PC2 / ARM64 / Ubuntu 20.04 / ROS 2 Foxy。Mac 用 SSH 登录，后续 PC2 命令在 SSH 会话内执行。首次部署见[PC2 部署指南](../g1_real_robot_docs/g1_real_robot_slam_pc2_b300_guide.md)；历史硬件记录按[实机文档入口](../g1_real_robot_docs/README.md)中的日期和设备区分。

每个参与本轮实机运行的 PC2 终端都先执行：

```bash
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export G1_DATA="$HOME/ldy/g1_real_data"
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
```

```bash
./scripts/workflows/01_self_check.sh robot
```

自检通过后会查询 `wlan0` 当次 IPv4 并打印 `ssh unitree@<当次无线IP>`。在操作电脑另开终端验证无线登录成功后，断开外部电脑连接 PC2 的有线网线，从无线会话继续 B2 建图；不要断开机器人内部或雷达网线。若无无线地址或连接失败，保留有线连接排查，重跑 01 获取新地址。 自检末尾也会打印新无线会话的环境命令；SSH 新会话不会继承上一个终端的 export，登录后执行：

```bash
cd "$HOME/ldy/FAST_LIO_LOCALIZATION_HUMANOID"
export G1_DATA="$HOME/ldy/g1_real_data"
export ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1
```


完成判据：所需 ROS 包和本机部署环境检查通过。此自检不查询雷达流或运控模式。换机/换架构应重新构建；源码同步与 `install/` 更新要求见[脚本索引](../scripts/README.md#部署时要一起更新什么)。

### B2. 采集并导出现场地图

PC2 执行；已有可用 PCD 时可跳过录制，直接导出：

```bash
./scripts/workflows/02_mapping.sh record B1_run01
```

入口复用活跃的 Livox；没有雷达端点时启动已配置的驱动，并启动 FAST-LIO 和录包。按现场采集安排覆盖要导航的区域，回到终端按 Enter 保存。本入口负责采集，机器人移动按现场已有操作方式完成。

按 **Enter** 才执行地图保存与结果校验；**Ctrl+C 是取消采集，不保证生成 PCD**。中断后保留的 bag/records 不要覆盖或删除，下一轮使用新名称。收到停止提示后等待清理结束，避免连续 Ctrl+C 中断收尾。

完成判据：`$G1_DATA/maps/B1_run01.pcd` 保存成功，`$G1_DATA/bags/B1_run01/metadata.yaml` 中雷达和 IMU 消息数均非零。原始日志在 `$G1_DATA/records/B1_run01/`。这说明数据已落盘，地图质量还需核对。

若需用 DUFOMap 去除动态点，在采集期间另开终端运行[逐帧扫描采集](#用-dufomap-清理多帧扫描)，或之后回放录包再采集。`record` 保存的单张 PCD 不包含 DUFOMap 所需的逐帧射线起点，不能直接作为它的输入；普通 `export` 也不会自动运行 DUFOMap。

导出可以在 PC2 或已装 NumPy/OpenCV 的地图处理机执行。跨机时先复制 PCD；以下两条路径均指执行机器上的文件：

```bash
./scripts/workflows/01_self_check.sh offline
./scripts/workflows/02_mapping.sh export /path/to/B1_run01.pcd /path/to/B1_map01 --plaster
```

`/path/to/...` 需替换为实际路径，输出目录须尚不存在。完成后检查墙、门口、起点、目标及未知区。地图起点处于占用格时，先查遮挡、坐标和地图覆盖，不能仅凭导出成功继续。

| 导出文件 | 用途 |
| --- | --- |
| `map.ply` | 现场点云降采样后的三维定位图，保留输入坐标 |
| `map.yaml`、`map.pgm` | 同坐标系二维规划图，未观测区域保留为未知 |
| `metadata.json` | 来源、分辨率、地面拟合及栅格统计 |
| `plaster.png`、`surface.ply`、`plaster.json` | 加 `--plaster` 才生成的可视预览与地面参数 |

输出目录已存在且为有效地图时，`02 export/dufo` 保留文件、不重新导出或应用参数，直接给出已有地图预览方式（退出码 3 表示未执行导出）；非有效地图目录仍报错。若网页服务及隧道已在运行，可直接打开链接。

核对完成后 `02` 会同时打印两个分支，二者选一即可：**(A)** 在 PC2 执行 `./scripts/real_robot/04_robot_ready.sh localize --map <地图目录>/map.ply` 直接进入 B4；**(B)** 可选支线：先在仿真服务器上预演导航、确认路线可通过再上实机。仿真（MuJoCo、定位、Nav2）跑在 4622 服务器上（默认 `sjtu@202.121.181.124`，换机器用 `G1_SIM_HOST`/`G1_SIM_PORT` 指定）；Mac 只负责发起，在 Mac 仓库根执行 `./scripts/simulation/03_preview.sh prepare pc2:<采集 PCD> --name <名称>_preview --pc2 unitree@<当次无线IP>`，通过 SSH 把 PCD 传到服务器并生成预演场景（见 B3），Mac 本身不跑仿真。

`02 export` / `dufo` 成功结束后会自动打印本次输出目录对应的网页启动命令、当次无线 IP 对应的 SSH 转发命令和浏览器地址；无需手工替换地图目录。无无线地址时会提示填写实际主机。

命令行连接 PC2 时，在定位前用网页检查导出地图：

```bash
# PC2：无需先启动定位或 Nav2。仅使用系统 Python 的 OpenCV、PyYAML。
./scripts/real_robot/05_web_navigation.sh "$G1_DATA/maps/B1_run03_map" --review --port 18082
# Mac 另开终端：地址使用 01 输出的当次无线 IP。
ssh -N -L 18082:127.0.0.1:18082 unitree@<当次无线IP>
```

浏览器打开 `http://127.0.0.1:18082/`。支持缩放、平移、点击查询地图坐标和空闲/占用/未知状态。核对墙、重影、门口和起点；点击不设置定位初值，不发送规划或运动请求。检查结束在 PC2 按 Ctrl+C 关闭。`--review` 不占用 ROS 域，正常 B5 网页导航保持原行为。

PC2 离线自检逐项报告 SciPy、scikit-image、Matplotlib 的导入结果和版本，不打印安装路径。缺失或导入失败时，末尾给出通过清华 PyPI 镜像安装独立 `.deps/offline-venv` 的命令；使用 ARM64/Python 3.8 二进制轮子，避免更改 ROS 系统 Python。该环境存在时，offline 自检及 `02 export/dufo` 默认使用它（兼容此前 `G1_PYTHON=/usr/bin/python3` 设置）；自定义的其他 `G1_PYTHON` 仍优先。虚拟环境是 PC2 本机构建产物，源码同步排除它。

普通导出与网页检查不要求先装石膏预览依赖。要生成石膏图，在 `02 export` 时加 `--plaster`；网页在二维图下加载同目录 `surface.ply` 三维网格，支持左键旋转、右键平移、滚轮缩放、俯视和重置；WebGL 不可用或无网格时回退到静态 `plaster.png`，并明确提示。为已有导出补充石膏图时，确认目录尚无 `surface.ply` 等同名产物，然后在 PC2 使用：

```bash
.deps/offline-venv/bin/python scripts/mapping/plaster_surface.py \
  "$G1_DATA/maps/B1_run03.pcd" "$G1_DATA/maps/B1_run03_map"
```

替换为实际来源 PCD 和对应导出目录，不要混用不同采集的数据。该命令不修改二维图；生成后重启 `--review` 服务并刷新网页。石膏图是经过地面拟合/调平、裁剪和平滑的静态预览，不用于点击设置定位坐标。

2026-11-24 PC2 曾发现 apt 中 `chrony` 与 `systemd-timesyncd` 冲突，导致 apt 安装预览依赖失败；未执行会移除 chrony 的自动修复。当前离线预览使用独立 Python 环境。

**`--plaster` 不生成 `scene.xml`，也不会启动 MuJoCo。** 只需实时定位和网页导航时，直接进入 B4。

### B3. 可选：在仿真机预演实机地图

02 采集、导出并核对地图后，可直接进入 B4 的 04 自动定位，也可选择本节 03 预演。PC2 不运行 MuJoCo；在 Mac 仓库根发起，PCD 经 SSH 中转到 Ubuntu 仿真服务器，MuJoCo、定位与 Nav2 全部在服务器运行。

一条命令完成“读取 PC2 PCD → 上传与 SHA-256 校验 → 自动选安全起点 → 生成扫描场景和配套地图 → 启动预演”：

```bash
./scripts/simulation/03_preview.sh play pc2:/home/unitree/ldy/g1_real_data/maps/B1_run03.pcd \
  --name B1_run03_preview_new --pc2 unitree@<当次无线IP>
```

名称使用未创建的新名称（02 会自动打印带时间戳的命令）。`play` 的输入为 PCD 时先准备再启动；输入已有场景目录时直接启动。只想先生成而不启动，使用 `prepare`；生成后脚本打印包含本次服务器和端口的 `launch` 命令：

```bash
./scripts/simulation/03_preview.sh prepare pc2:/home/unitree/ldy/g1_real_data/maps/B1_run03.pcd \
  --name B1_run03_preview_new --pc2 unitree@<当次无线IP>
./scripts/simulation/03_preview.sh launch sim/generated/B1_run03_preview_new
```

无图形界面时，在 `play` 或 `launch` 命令末尾添加 `-- --headless --goal X Y`，目标在生成的仿真地图中选择。GUI 模式自动探测服务器显示会话，也可通过 `G1_SIM_DISPLAY=:N` 指定。起点从 `metadata.json` 自动传给仿真，不再退回 `(0,0)`；用 `--start X Y` 可覆盖生成起点，导航阶段覆盖需放在 `--` 后。

默认 `sjtu@202.121.181.124:4622`，不可达或空闲显存不足 2 GiB 时尝试 4322；显式 `G1_SIM_HOST`/`G1_SIM_PORT` 指定时不擅自切换。默认仿真域 75，与实机域 223、常用仿真域 74 分离；可用 `G1_SIM_DOMAIN` 指定独立域，不能用 223。只读检查并选择空闲 GPU，不结束已有服务器任务。

生成时选择离扫描原点最近、至少有 0.75m 净空的起点，保持独立新目录。场景生成和 MuJoCo 编译在服务器后台记录 `pid/status/generate.log`（`~/ldy/g1_real_data/records/preview/`），每 25 秒报告进程与日志增长；SSH 中断后先查看状态，不重复启动。仿真运行在前台，Ctrl+C 停止本次预演。

服务器须已完成 A1 仿真环境准备；启动前检查定位程序动态库及 Nav2 包，若本机缺依赖但已有 `ros_overlay_ns.sh` 覆盖层可用，会自动在只读覆盖层中运行，不更改系统依赖；覆盖层不可用则明确报错。不会自动安装或构建。Mac/PC2 不需复制私钥；PC2 尚未配置服务器 SSH 时使用上述 Mac 中转方式即可。预演结束回到 PC2 的实机地图，执行 04；可全程跳过本节。

服务器内部也可直接复用现有入口做逐步排障：

```bash
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1
.deps/venv/bin/python scripts/mapping/g1_scan_scene.py \
  --pcd /path/to/B1_run01.pcd --out sim/generated/B1_preview01 --auto-start
./scripts/simulation/03_simulation.sh nav sim/generated/B1_preview01 --start X Y
```

最后一条的起点取生成时打印的值；`03 nav` 遇到包含 `scene.xml` 的扫描场景会自动转入扫描预演实现。

这里重新生成 `scene.xml` 和配套的 `map.ply/map.yaml/map.pgm`；定位图取自近似碰撞几何，地面经过调平和人工延伸。运行链是“虚拟 LiDAR/IMU → FAST-LIO → Open3D → Nav2 → 虚拟 G1”，没有接入 PC2 的实时雷达或硬件速度输出。

完成判据：场景能加载，虚拟点云与配套地图对齐，预演目标及障碍用例符合预期。它有助于发现场景或地图配套问题；真实标定、传感器延迟、行走振动和运控响应仍由现场验证。

**B4/B5 使用 B2 导出并核对的实机地图，不直接使用 B3 的仿真地图。** 两个目录虽然都叫 `map.ply`、`map.yaml`，几何、坐标与空闲区假设可能不同。生成原理、初始高度及石膏版历史场景见[扫描场景预演手册](../g1_real_robot_docs/scan_mujoco_preview.md)。

### B4. 回到 PC2，启动持续定位并检查状态

将 B2 的整组地图放到 PC2，例如 `$G1_DATA/maps/B1_map01/`。在完成 B1 环境设置的终端执行：

```bash
./scripts/real_robot/04_robot_ready.sh localize \
  --map "$G1_DATA/maps/B1_map01/map.ply"
```

默认自动检测机器人在地图中的位置与朝向，无需输入配准初值。启动期间保持机器人静止：入口采集三组不重叠的新鲜同步点云，在整张地图及重叠分区用 FPFH/RANSAC 与重力对齐的全图平移/朝向相关性搜索、ICP 复核；重叠率不足、误差过大、存在得分接近的不同位置、后两组扫描不一致或超时均退出，不回退到零初值、不启动持续地图定位。通过后才将 `map ← camera_init` 结果交给 Open3D 持续跟踪；持续节点初始化完成前不发布候选地图 TF/位姿。

搜索最长 120 秒，扫描采集最长 20 秒。证据保存在本轮日志目录的 `global-scans/`、`global-capture.log`、`global-search.log` 和 `global-initialization.json`。多候选检查只是有界几何搜索，不能保证所有重复场景都可区分；没有足够独特结构时换到更可辨识的位置，再重新启动本轮会话。成功仍需结合网页与现场核对，不表示运动许可。

`--initial X Y Z ROLL PITCH YAW` 仅保留为显式调试选项（米、度；`map ← camera_init`，旋转顺序 Rx·Ry·Rz），标准流程不要求使用。不要把机体在地图上的位置直接照填，也不要照抄历史初值。

入口保留已有 FAST-LIO，或启动驱动与 FAST-LIO，再启动 Open3D 持续配准。已有旧 Open3D 或 `map → body` 对齐时会报错，先确认并结束冲突会话。保持本终端运行，另开一个完成 B1 环境设置的 PC2 终端检查：

```bash
./scripts/real_robot/04_robot_ready.sh status
# 可选：另外查询运控模式，NIC 换成机器人通信网卡名。
./scripts/real_robot/04_robot_ready.sh status --interface NIC
```

完成判据：雷达/IMU、FAST-LIO 机体云和里程计有新数据，配准健康且 `map → body` 更新。还需结合地图与现场核对位置和朝向。`status` 是只读检查，不负责启动定位，也不表示已经获得实机行走许可。SDK 查询工具见[构建记录](../g1_real_robot_docs/debug_log/2026-11-10_pc2_live_nav_web.md#sdk-会话二进制的构建)。

### B5. 启动 Web：规划与实机导航

04 定位成功后会打印完整的下一步命令，并保存到本轮 `next-web.txt`：在操作电脑另开 SSH 终端（同时转发 18081 端口），登录后执行 `cd`、本轮 `G1_DATA`/ROS 域的 `export`，再用定位图同目录的 `map.yaml` 启动 05。最后给出浏览器地址。保持 04 终端运行；若没有配套 `map.yaml`，提示补齐地图，不替你选择其他地图。

#### B5.0 实机导航前置：让机器人进入 FSM 501

05 默认以实机导航启动，而导航 SDK 只接受 **FSM 501（Regular mode）**：其它模式会打印 `REJECTED FSM_NOT_501` 并退出，页面显示启动失败原因。FSM 切换由现场操作员负责；PC2 侧先用工具只读确认，必要时再尝试 `set`：

```bash
# PC2：不要 source ROS，Foxy 的 DDS 会与 SDK 自带的 DDS 冲突
env -u LD_LIBRARY_PATH "$G1_DATA/bin/g1_fsm_tool" eth0 get      # 期望 fsm_id=501
env -u LD_LIBRARY_PATH "$G1_DATA/bin/g1_fsm_tool" eth0 set 501  # before/读回都会打印
```

`set` 只有在读回确认目标模式后才返回 0，不要只看服务端应答。按公开文档，501 需从 Lock stand（FSM 4）进入：遥控器 `L2+B` 进阻尼 → `L2+Up` 进锁定站立，再在机器人板载机执行 `/home/unitree/carry mode regular`。2026-11-24 实测：机器人处于 802 时 `SetFsmId(501)` 被接受但读回仍是 802，PC2 侧不能仅靠 SDK 从 802 直接切；工具源码为 `scripts/real_robot/g1_fsm_tool.cpp`，完整过程见[切换 501 命令记录](../g1_real_robot_docs/debug_log/2026-11-24_fsm_501_switch.md)。只用 `--plan-only` 规划时不需要 501。

#### B5.1 启动入口

保持 B4 定位终端运行，在另一个完成 B1 环境设置的 PC2 终端执行：

```bash
./scripts/real_robot/05_web_navigation.sh "$G1_DATA/maps/B1_map01" --port 18081
```

三种入口的区别：

- 默认 `real`：启动 Nav2 的 `map_server`、`planner_server`、`controller_server`、`bt_navigator` 和实时障碍输入节点，网页可“开始实机导航”。
- `--plan-only`：只启动 `map_server`、`planner_server`，网页仅请求 `ComputePathToPose` 路径，不启动控制器、行为树与 SDK。
- `--review`：只检查导出地图，不启动 ROS、定位或 Nav2。

启动前入口逐项检查：`/Odometry_loc` 与 `/cloud_registered_body_1` 是否有发布端（没有会直接提示先运行 04，而不是等 12 秒超时）、`lio`/`localized` 就绪、地图 YAML 与图像、本地端口、旧 Nav2 节点。`real` 模式还要求 `$G1_DATA/bin/g1_navigation_sdk` 已构建（`./scripts/real_robot/build_navigation_sdk.sh`）、机器人网口存在（`G1_ROBOT_INTERFACE`，默认 `eth0`）以及 `metadata.json` 中的 `floor_z`。

在 Mac 上转发网页端口；无线连接可换成现场确认的 PC2 地址：

```bash
ssh -N -L 18081:127.0.0.1:18081 unitree@192.168.123.164
```

05 就绪后会打印访问电脑上执行的完整 SSH 转发命令，自动使用当次无线地址（无无线时尝试有线地址），并给出浏览器地址。若已按 04 提示通过 `ssh -L` 建立同一端口转发，直接复用，无需再执行 `ssh -N`。

打开 [Web 页面](http://127.0.0.1:18081/)。先核对位姿箭头、实时雷达点和地图是否对齐，再点选目标查看路径。

#### B5.2 实机导航如何执行与停止

`real` 模式下网页先规划、再明确点击“开始实机导航”才动作：

- 规划阶段只调用 `ComputePathToPose`，不发送速度。
- 开始后网页向 Nav2 发送 `NavigateToPose` 目标，同时启动 `g1_navigation_sdk` 子进程；SDK 校验 FSM 501 后 `ARM`，网页把控制器输出的 `/g1/navigation/cmd_vel` 以 20 Hz 转发为 `V x y z`。行为树为 `PipelineSequence`：1 Hz `ComputePathToPose` + `FollowPath`。
- 以下任一条件都会取消目标并让 SDK 发 `STOP` 停车：点击“取消并停车”、到达目标、导航结束、页面心跳中断（2 秒）、`/localization_3d_registration_ok` 失效或 `map → body` TF 超过 1 秒、控制器速度超过 0.35 秒未更新、启动后 2 秒仍无速度输出。取消后若零速度确认失败，页面提示人工接管。
- 同一时刻只允许一个实机导航任务，任务由发起页面持有；刷新或关闭浏览器会因心跳丢失而停止。网页断连、定位失效都会停止任务。
- 现场必须有人值守并保证急停可用；`--plan-only` 与实机导航互斥，切换需要重启 05。

网页的“规划考虑实时障碍”默认开启，决定实时雷达观测是否进入代价地图：关闭后使用静态地图障碍，实时雷达点仍可显示。规划模式下它只影响全局规划；`real` 模式局部代价地图读取同一组滤波点云，因此开关同时影响全局与局部障碍层。两个“显示”选项只控制画面。切换后旧路径失效，需要重新规划；状态未知或切换失败会显示原因，详见[开关说明](../scripts/real_robot/web/README.md#实时障碍参与规划)。

网页还提供“点群判定半径”“最少支持点数”“障碍膨胀距离”三个滑块。拖动预览，松手后应用并使旧路径失效；点群阈值只影响实时障碍，膨胀距离同时影响静态和实时障碍。Foxy 修改膨胀距离时会短暂停止规划并重载代价地图，页面等待确认后才恢复规划。

实机导航与规划共用同一 Nav2 会话：`real` 模式额外启动 `controller_server`、`bt_navigator`，全局与局部代价地图的实时障碍层都按 `floor_z + 0.15 ~ floor_z + 1.8` 过滤高度（局部层为静态层 + 实时障碍层 + 膨胀层），`--plan-only` 不启动这些节点，网页只请求 `ComputePathToPose` 路径。

完成判据：地图、位姿、代价地图、实时雷达图层和规划器就绪；现场可通行目标能规划，阻断目标符合预期。用同一目标分别检查开关两种状态下的路径和代价地图。实机导航还必须在 FSM 501、现场有人值守且急停可用时验证启动、抵达、取消停车和断连停车；只看到路径或 SDK `ARM` 成功，不能说明机器人会安全行走。

常见失败：`未检测到 FAST-LIO 输出` 表示 04 会话已停或未启动，重开 04 再启动 05；`REJECTED FSM_NOT_501` 表示机器人不在 501，按 B5.0 处理；端口 `Errno 98` 多是上一轮 05 刚退出、端口尚未释放，等待数秒重试；`Web 数据未就绪` 查本轮 `web.log`、`nav2.log` 和定位数据。

本路线到此完成实时感知、定位、规划与实机导航。默认入口仍要求操作员显式点击“开始实机导航”，`--plan-only` 是没有运动的安全模式；停止、超时、断链和接管行为见 B5.2，现场验收仍需逐项复测。旧版本的测试保存在[历史行走记录](../g1_real_robot_docs/2026-11-10_g1_autonomous_walk_plan.md)，其中已删除的命令不属于当前操作流程。

## 地图处理的常用选项

首次运行可沿用默认值；需要调图时再查本节。

- 录制可加 `--seconds 60` 自动结束，或用 `--point-filter 3 --surface-voxel 0.5 --mapping-voxel 0.5` 调整 FAST-LIO。雷达话题自动识别 `/livox/lidar` 或 `/livox/custom_msg`，也可指定 `--lidar-topic`。
- 导出预设 `detail`、`standard`、`quick` 的三维体素 / 二维分辨率分别为 0.03/0.05、0.06/0.08、0.12/0.12 m，用 `--preset` 选择，或分别用 `--voxel`、`--resolution` 覆盖。二维投影使用三维降采样前的点。
- 二维图支持 `--max-range`、`--floor-band`、`--obstacle-min`、`--obstacle-max`。默认拟合地面分类；已整平点云可显式选 `--floor-model percentile`。地面拟合失败时先检查地面回波覆盖。
- `--plaster` 默认剔除离地 0.16 m 以下的回波，在 15 m 半径、离地 2.8 m 内以 0.08 m 网格生成预览。用 `--plaster-ground-cut`、`--plaster-radius`、`--plaster-height`、`--plaster-grid` 调整，只影响预览文件。
- 离线 Python 可用 `G1_PYTHON=/path/to/venv/bin/python` 指定。导出需要 NumPy/OpenCV；石膏预览另需 SciPy、scikit-image、Matplotlib。

## 用 DUFOMap 清理多帧扫描

这是导出地图前的可选清理步骤。代码入口是 `scripts/workflows/02_mapping.sh dufo`，
底层调用 `scripts/mapping/dufomap_clean.py`；算法本体来自 Python 环境中的
`dufomap==1.1.1` 包，因此仓库没有单独的 DUFOMap 算法源码目录。

DUFOMap 需要多帧点云、精确配对时间戳和每帧雷达原点；单张 PCD 可直接导出，无需此步骤。实际采集或录包回放时，保持 FAST-LIO 运行，在同一 ROS 域的另一个终端执行：

```bash
./scripts/workflows/02_mapping.sh capture-scans /path/to/scans \
  --lidar-in-body X Y Z --frames 300
G1_PYTHON=/path/to/dufomap-venv/bin/python \
  ./scripts/workflows/02_mapping.sh dufo /path/to/scans /path/to/B1_clean01 --d-p 1 --plaster
```

`X Y Z` 是当前挂载与 FAST-LIO 配置下 LiDAR 在 `body` 中的实测平移（米）。采集可用 `--idle-timeout` 代替固定帧数。后处理使用 `dufomap==1.1.1`，该版本没有 ARM64 安装包，PC2 采集扫描后复制到 Linux x86_64 处理机处理；offline 自检末尾会给出隔离环境、清华 PyPI 镜像安装及导入验证命令，并在 PC2 明确提示不要原地执行 x86_64 安装。默认 `--d-s 0.2` m、`--d-p 2` 体素；`--d-p 1` 是示例候选，需核对误删情况。独立清空复扫可用 `--revisit-after STAMP_NS` 指定起始纳秒时间戳，见[验证记录](superpowers/plans/2026-11-05-dufomap-sim.md)。

## ROS 域、日志与结束会话

| 入口 | 未设置 `ROS_DOMAIN_ID` 时的默认域 | 默认工作流日志 |
| --- | --- | --- |
| `03_simulation.sh`、`scan_preview.sh`、自检 `sim` | 74 | `sim/generated/workflow_logs/` |
| PC2 采集、定位、Web、自检 `robot` | 223 | `$G1_DATA/records/workflows/` |
| 手动 `start_*.sh` 与仿真检查脚本 | 72 | 见各脚本输出 |
| 支架传感器演示 `run_g1_mujoco.sh` | 71 | `sim/generated/logs/` |

已导出的域会覆盖默认值。同一会话的所有终端必须一致，尤其是在域 74 运行统一入口后，另开终端手动执行 `ros2` 或检查脚本时：

```bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source scripts/env/walking_env.sh
```

PC2 实时入口改用域 223。仿真、录包回放与现场实时链分开运行；实机入口会拒绝域内的 `/clock` 或仿真真值发布者。实时 PC2 使用系统时间，MuJoCo 行走链使用仿真时钟；Foxy 录包回放的时钟处理见[离线回放说明](../g1_real_robot_docs/inverted_mid360.md)。

入口启动时打印本轮日志目录，可用 `G1_WORKFLOW_LOGS` 改变位置。失败时先查对应组件日志和 `probe-*.log`；进程冲突按[启动前的进程预检](#启动前的进程预检)处理。

统一入口按 Ctrl+C 清理本次启动的进程。实机 Web 和定位分属两个终端，先停 Web，再停定位；复用的外部驱动或 FAST-LIO 仍由原终端管理。纯仿真会话里导航门禁一直保持开启，带或不带 `--goal` 都可以在同一轮里连续点选多个目标；`--goal` 只多跑一次自动验证，验证结束不会关门禁。要换地图或换起点时再 Ctrl+C 重开一轮。
