# 05 Web 导航「未达到就绪条件：lio」定位过程（2026-11-24 晚间）

16:59 在 PC2 运行 `05_web_navigation.sh` 时在 `wait_ready lio --seconds 12` 处失败：

```
失败：未达到就绪条件：lio --seconds 12，查看 probe-lio.log
日志目录：/home/unitree/ldy/g1_real_data/records/workflows/web-navigation-20261124-165932-yslxAY
```

`probe-lio.log` 全文只有 `lio 未就绪，已收到：无`（29 字节），即 12 s 内一条 LIO 消息都没收到。

## 根因：05 不启动定位，依赖 04 常驻的 FAST-LIO

- 设计上 05 只做导航侧（Nav2 + Web），`/Odometry_loc`、`/cloud_registered_body_1` 由 04 的
  FAST-LIO 发布；05 的 usage 也写明「先让 `04_robot_ready.sh localize` 保持运行」。
- 当天 16:33:39 启动的 04 会话（`robot-ready-20261124-163339-b5kyUK`）本身是成功的
  （probe-lio 就绪、自动全局重定位 accepted fitness 0.9976），但 16:49:46 收到 SIGINT 被停掉
  （fast-lio / livox / open3d 三份日志都是 `signal_value=2`）。
- 16:49 之后没有任何新的 `robot-ready-*` 目录；16:59 跑 05 时 `ps` 里没有任何 ROS 进程，
  lio/web 锁空闲、18081 空闲。所以失败点是「定位链不存在」，不是雷达、FAST-LIO 或地图故障。
- 16:59 有两次尝试：`web-navigation-20261124-165921-tWzOTB`（ros/ 为空，probe 之前就失败）
  和 `-165932-yslxAY`（ros/ 三个 python3 日志时间 ≈16:59:33.3/37.8/40.5，与
  preflight → free probe → 包检查 → 地图检查 → nav_graph_empty → lio probe 的时序吻合）。

也就是说，16:51–16:57 构建导航 SDK、改 05/live_env 期间停掉的 04 没有再拉起，直接启动 05 就撞上了这个报错。

## 恢复

按规范链重启，两个会话都保持运行：

```bash
# 终端 A
G1_DATA=/home/unitree/ldy/g1_real_data ./scripts/real_robot/04_robot_ready.sh localize \
  --map /home/unitree/ldy/g1_real_data/maps/B1_run01_map/map.ply

# 终端 B（04 打印的 next-web.txt 里有同样的命令）
G1_DATA=/home/unitree/ldy/g1_real_data ./scripts/real_robot/05_web_navigation.sh \
  /home/unitree/ldy/g1_real_data/maps/B1_run01_map/map.yaml --port 18081
```

本轮（`robot-ready-20261124-170445-sFjWt0` + `web-navigation-20261124-170648-Id6qfp`）结果：

- 04：probe-lio 就绪；自动全局重定位 accepted fitness 0.99569、rmse 0.0745，
  initial `[-0.1844, 0.0110, 0.0149, 2.4554, -15.6209, -174.83]`；probe-localized
  `map → body: -0.178, -0.009, 0.035 m`。
- 05：probe-lio 就绪、probe-localized `map → body: -0.182, -0.012, 0.036 m`、
  probe-nav `Nav2 生命周期 active，已收到 /map`；`map_server/planner_server/controller_server/bt_navigator`
  全部 active，global/local costmap 已激活；Web 监听 `127.0.0.1:18081`，
  `/api/state` 返回 `navigation.mode=real, phase=idle, available=true`，
  `/api/obstacles` 有 2880 个实时障碍点。
- 当时机器人未动：网页需要人工点「开始实机导航」，且要求操作员已把机器人置 FSM 501。

## 改进：把「未达到就绪条件」换成可操作的诊断

`05_web_navigation.sh` 在 `nav_graph_empty` 与 `wait_ready lio` 之间加了一段 4 行前置检查
（第 87–89 行），用 `probe.py free --topic` 判断 LIO 话题有没有发布端：

```bash
if probe free --topic /Odometry_loc --topic /cloud_registered_body_1 2>/dev/null; then
    fail "未检测到 FAST-LIO 输出（/Odometry_loc、/cloud_registered_body_1 无发布端）；先运行 04_robot_ready.sh localize 并保持该会话运行"
fi
```

- 语义核对：LIO 在跑时该命令 exit 1（有发布端）→ 跳过 fail，继续原来的 12 s 等待；
  话题无发布端时 exit 0 → 直接给出「去启动 04」的提示，不再让人对着 `probe-lio.log` 猜。
- 已在 PC2 用现有链路验证两个分支（有 LIO：跳过；伪话题：触发），并 `bash -n` 通过；
  线上脚本已更新（md5 `a398069e8231ad90861f2368b301f120`，旧版备份在
  `/tmp/05_web_navigation.sh.bak.20261124-170809`）。改动只影响下一次启动。

## 待确认 / 提醒

- `docs/WORKFLOWS.md` 的 B5 仍写「Web 仅规划」（mode:=plan，只有 map_server + planner_server），
  而 05 默认已切到 real（会起 controller_server/bt_navigator，并通过 SDK 下发实机导航）。
  文档与代码不一致，实测前先确认到底要走哪条链路。
- 本轮是 real 模式首次跑通就绪，但只是「等待开始」状态，真机运动与停止/断链保护尚未验证；
  运动前需要现场人员、FSM 501 与急停手段。
- 05 的端口预检不带 SO_REUSEADDR，上一轮刚退出时可能 Errno 98，等几秒或换端口重试。
