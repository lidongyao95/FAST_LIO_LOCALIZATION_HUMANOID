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
    launch="$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
    check_file "$launch"
    start_bg livox ros2 launch "$launch"
    lidar_topic=$(probe lidar --seconds 18 --lidar-topic "$selected") || fail "MID-360 雷达或 IMU 无数据；检查 PC2 驱动配置和网口"
    assert_running
}
