#!/usr/bin/env bash
# Helpers shared by the real-robot entry scripts. Source after shared/lib.sh.

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
    local host_ip
    host_ip=$(sed -n 's/.*"cmd_data_ip" *: *"\([^"]*\)".*/\1/p' "$repo/robot/config/MID360_config.robot.json" | head -1)
    if ! grep -q -- "$host_ip" "$config" 2>/dev/null; then
        cp "$repo/robot/config/MID360_config.robot.json" "$config"
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
    say '保持这个终端运行。'
    say "在访问电脑浏览器打开：http://127.0.0.1:${web_port}/"
}
