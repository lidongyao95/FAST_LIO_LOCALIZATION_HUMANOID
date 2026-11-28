# PC2 仅规划打通：TF 假死、地图污染与副本图（2026-11-24）

本轮目标是把 PC2 上的 G1 网页端从"地图能看、点选目标必然失败"推进到"稳定生成可行路径"，
为后续实机行走（速度桥 + 运控接管）准备一个可信的起点。结论是两件事叠在一起：
旧会话的 `map → body` 变换早已停发，而地图本身在机器人站位上被建图时的机身/人员污染。
两个问题都已修复并验证，但**副本图只是绕过，不是修正**，建图阶段还要做根因处理。

## 环境

- PC2：`ssh unitree@172.18.14.107`（sic-guest，wlan0 172.18.14.107/20），eth0 192.168.123.164，
  雷达 192.168.123.120；aarch64 / Ubuntu 20.04.6 / ROS 2 Foxy。
- 仓库：`/home/unitree/ldy/FAST_LIO_LOCALIZATION_HUMANOID`，
  `ROS_DOMAIN_ID=223`、`ROS_LOCALHOST_ONLY=1`。
- 地图：`/home/unitree/g1_real_data/maps/b1_2026-11-23`（`clean.pcd` + `map.pgm/yaml/metadata.json`，
  origin `[-3.2, -6.0]`，分辨率 0.1 m，187×130，`floor_z = -1.2487`）。
- 远端浏览：`ssh -N -L 18081:127.0.0.1:18081 unitree@172.18.14.107`，然后打开
  <http://127.0.0.1:18081>。

## 症状与两次误判

1. 网页能显示地图、位姿、实时雷达点，但点选目标一律返回
   `未返回可行路径（Nav2 状态 6）`。
2. 用副本图重启 Nav2 后，planner_server 反复报
   `Timed out waiting for transform from body to map ... Invalid frame ID "map" ... frame does not exist`，
   随后 `ensure_g1_nav_lifecycle.py` 超时，整个 launch 自己 SIGINT 退出。

第一次误判是"以为规划器配置有问题"，第二次误判是"以为新地图 YAML 写坏了"。实际都不是：
`/api/map` 里机器人位姿所在栅格是 `100`（occupied），最近占据格距位姿 0.058 m，
NavFn 的起点不可通行；而 `map` 帧消失是因为**上一轮留下的 `global_localization_node` 进程还活着、
配准日志还在刷（fitness 0.73），但它那条 `map → body` TF 已经不再发布**。
进程活着 + 日志在动 ≠ 链路健康，这是本轮最值得记住的一条。

## 根因一：旧定位会话的 TF 假死

- 现象：`ros2 run tf2_ros tf2_echo map body` 报 `Invalid frame ID "map"`，
  但 `global_localization_node` 进程在、配准输出在。
- 处理：`scripts/env/g1_cleanup_stale.sh --stop --scope all` 清干净后按规范链重启，
  不要在半死会话上叠新 launch。
- 规范链（不要手搓 ros2 launch）：
  1. `04_robot_ready.sh localize --map .../clean.pcd --initial -0.26781 0.03669 -0.00383 -0.60679 -0.93801 -45.0886`
     → 成功标志 `Open3D 已配准，map → body 持续更新`；
  2. `05_web_navigation.sh <MAP_DIR> --port 18081`
     → 成功标志 `地图、位姿、代价地图、规划器和实时雷达图层就绪（N 个点）`。
- 两个坑：
  - `ros2 launch` 必须先 `cd $REPO`，否则报 `Package 'open3d_loc/launch/...' not found`；
  - `05` 的端口预检用不带 `SO_REUSEADDR` 的 bind，旧 server 刚退出时会
    `[Errno 98] Address already in use`，`ss` 看不到监听时等几秒重跑即可。

## 根因二：建图时把机身/人员扫进了地图

- 证据链：
  - 位姿格 `100`，最近占据格 0.058 m，成本地图该处 254，±0.8 m 窗口全是 253/254；
  - 把 `/api/obstacles` 的 490 个地图系点与占据格做 Chamfer 最近距离：
    as-is 中位数 0.041 m，网格搜索 (dx,dy,dθ) 最优只到 0.040 m →
    **定位没有错位，地图也没有翻转**，就是地图里确实有东西；
  - 以 `floor_z=-1.2487` 为基准的高度直方图：位姿 0.5 m 内 950 点，其中 0.8–1.4 m
    高度 443 点（正是机身高度），0–0.2 m 344 点 → 建图当天机器人和人员就站在那里；
  - 地图上因此出现一条 2.3 m × 0.3 m 的水平"障碍带"，机器人正落在带子中间。
- 曾试过用 `--obstacle-min/--obstacle-max` 重建窄带图，但 `--floor-model plane` 拟合出
  `floor≈-1.2281`，比百分位的 p5 高 2 cm，大量地面点掉进障碍带，
  occupied 从 6647 涨到 9706，机器人周围几乎全黑 → 该图作废。**高度带过滤必须在正确的
  地面基准上做，否则越滤越脏。**

## 修法：地图副本 + 可复现脚本

原始地图不动，复制一份并把站位半径内的占据格改成自由：

```bash
cd /home/unitree/ldy/FAST_LIO_LOCALIZATION_HUMANOID
python3 scripts/mapping/patch_g1_map.py /home/unitree/g1_real_data/maps/b1_2026-11-23 \
    --pose -0.2509 0.0364 --radius 0.8 --out /home/unitree/g1_real_data/maps/b1_2026-11-23_local
```

输出：

```
位姿格 (29, 69) 值 0 -> 254
半径 0.8 m 内清掉占据格 139 个，剩余 0 个
```

同一个脚本在 Mac 上对同一份原始地图重跑，产出与 PC2 上的副本 **逐字节一致**，
所以这份地图可以随时重建，不需要手工修图。

**关键实现细节（踩过的坑）**：PGM 首行是地图顶部，y 必须按
`Y = oy + (height - row - 0.5) * resolution` 换算。第一版按 `oy + (row + 0.5) * resolution`
清，清到了关于 y≈0.5 镜像的位置——地图上看是"清干净了"，位姿格仍是 `100`，
`/api/plan` 全部 422。判断清除是否生效，只看位姿格的值，不要看图像观感。

PNG 图例：`0`=占据、`254`=自由、`205`=unknown（205 和 254 都 >200，渲染时别混）。
`/api/map` 的 `data` 是扁平数组，索引 `row*width+col`，`row0` 对应 `y=origin_y`（底部）。

## 验证

- 机器人周围 16 个方向/距离采样：**15 成功 1 失败**；唯一失败点 (x≈-0.24, y≈3.54)
  距占据格 0.39 m，落在 0.6 m 膨胀半径内，属合法拒绝（flood fill 显示该格本身 free
  且连通 12147 个格，说明是膨胀层拒绝而非地图断裂）。
- 网页端实测：点选目标 → 仅规划 → `规划成功 · 11.15 m · 222 个路径点`，
  状态行 `PC2 已连接 · 地图就绪 · 定位 TF 可用 · 规划器在线`，实时雷达 467 点。

## 遗留与下一步

- 副本图在机器人脚下"抹掉"了 0.8 m 内的障碍，**不能拿去跑长距离自主导航而不复核**；
  建图阶段应加入机身/人员高度带过滤（在正确地面基准上）来根治。
- 网页目前只到"生成路径"：`mode:=plan` 只起 `map_server` + `planner_server`，
  没有控制器与速度桥。实机行走需要 `mode:=nav`（全栈 + 实机参数）与
  `g1_real_velocity_bridge.py` 的 dry-run → 硬件输出两阶段推进。
- 速度桥与 SDK 会话源码原先只存在于 PC2（旧扁平布局），本轮已拉回
  `scripts/real_robot/walking/`；PC2 上的运行路径仍是 `scripts/` 下的旧位置，两边布局尚未统一。
