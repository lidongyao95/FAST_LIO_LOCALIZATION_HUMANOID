#!/usr/bin/env bash
# Sourced after workflows/lib.sh by live G1 entry points.

require_real_graph() {
    probe free --topic /clock --topic /g1_sim/ground_truth ||
        fail "实机入口需要独立的 ROS 域，检查仿真时钟和仿真位姿真值发布者"
}

ensure_livox() {
    local selected=${1:-} launch
    if lidar_topic=$(probe lidar --seconds 4 --lidar-topic "$selected"); then
        say "复用活跃雷达与 IMU：$lidar_topic"
        return
    else
        local status=$?
        (( status == 3 )) || fail "发现雷达或 IMU 端点，但流异常；检查重复进程、话题和驱动日志"
    fi
    ros_packages livox_ros_driver2
    local share config
    share="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2"
    config="$share/config/MID360_config.json"
    # The upstream config ships 192.168.1.5/192.168.1.12; with those addresses
    # the SDK cannot bind the receive sockets and the driver dies with
    # "bind failed". Reinstalling the driver source restores them, so put the
    # robot addresses back before every launch.
    if ! grep -q 192.168.123.164 "$config" 2>/dev/null; then
        cp "$repo/scripts/real_robot/config/MID360_config.robot.json" "$config"
        say "已恢复雷达接收地址配置：$config"
    fi
    launch="$share/launch_ROS2/msg_MID360_launch.py"
    check_file "$launch"
    start_bg livox ros2 launch "$launch"
    lidar_topic=$(probe lidar --seconds 18 --lidar-topic "$selected") || fail "MID-360 雷达或 IMU 无数据；检查 PC2 驱动配置和网口"
    assert_running
}

pc2_access_ip() {
    local nic address
    if command -v ip >/dev/null 2>&1; then
        for nic in wlan0 eth0; do
            address=$(ip -4 -o addr show dev "$nic" scope global 2>/dev/null |
                awk 'NR == 1 { split($4, parts, "/"); print parts[1] }') || address=
            if [[ -n "$address" ]]; then printf '%s\n' "$address"; return; fi
        done
    fi
}

print_web_access() {
    local web_port=$1 address
    address=$(pc2_access_ip)
    say '在 Mac 等访问电脑另开终端执行以下命令（不是在 PC2 里执行）：'
    if [[ -n "$address" ]]; then
        printf 'ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:%s:127.0.0.1:%s unitree@%s\n' "$web_port" "$web_port" "$address"
    else
        say '未检测到可用地址，请将下方 PC2_IP 替换为当次实际地址。'
        printf 'ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:%s:127.0.0.1:%s unitree@PC2_IP\n' "$web_port" "$web_port"
    fi
    say '保持转发终端运行；如果已通过 04 提示建立了同一端口转发，无需重复执行。'
    say "在访问电脑浏览器打开：http://127.0.0.1:${web_port}/"
}

print_web_next_steps() {
    local localization_map=$1 web_map="${1%/*}/map.yaml" pc2_ip=
    pc2_ip=$(pc2_access_ip)
    say '保持当前 04 终端运行，不要按 Ctrl+C。下一步启动 05 网页导航。'
    say '1. 在操作电脑另开终端，登录 PC2 并同时建立网页转发：'
    if [[ -n "$pc2_ip" ]]; then
        printf 'ssh -o ExitOnForwardFailure=yes -L 127.0.0.1:18081:127.0.0.1:18081 unitree@%s\n' "$pc2_ip"
    else
        printf 'ssh -o ExitOnForwardFailure=yes -L 127.0.0.1:18081:127.0.0.1:18081 unitree@<当次PC2_IP>\n'
        say '未取得 PC2 地址，请替换上面的 IP。'
    fi
    say '2. 登录后在新 PC2 终端依次执行（新 SSH 会话不会继承原终端的 export）：'
    printf 'cd %q\n' "$repo"
    printf 'export G1_DATA=%q\n' "${G1_DATA:-$HOME/ldy/g1_real_data}"
    printf 'export ROS_DOMAIN_ID=%q ROS_LOCALHOST_ONLY=%q\n' "${ROS_DOMAIN_ID:-223}" "${ROS_LOCALHOST_ONLY:-1}"
    if [[ -s "$web_map" ]]; then
        printf './scripts/real_robot/05_web_navigation.sh %q --port 18081\n' "$web_map"
    else
        say "定位图同目录未找到 map.yaml：${localization_map%/*}；请先准备与这张三维图配套的二维地图。"
        printf './scripts/real_robot/05_web_navigation.sh <配套地图目录/map.yaml> --port 18081\n'
    fi
    say '3. 等待 Web 就绪，在操作电脑浏览器打开：http://127.0.0.1:18081/'
    say '核对机器人位姿、实时点云和地图，点选目标后先规划路径；点击“开始实机导航”执行，点击“取消并停车”终止。'
    say '上述 SSH 已包含端口转发，无需再开转发终端；若已有同一 PC2 的转发，可复用它并直接打开新 PC2 终端。'
}
