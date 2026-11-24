#!/usr/bin/env bash
# 在没有 root 的远端主机上，把解包到 logs/*/deps 的额外 ROS 包叠加进命名空间后执行命令。
# 仅用于宿主 /opt/ros/<distro> 缺少 nav2/rviz2 一类包、又无法 apt 安装的机器（例如 4622）。
# 用法：scripts/env/ros_overlay_ns.sh <命令> [参数...]
# 依赖层位置可用 ROS_DEPS_DIR 指定；默认取 ~/ldy/logs 下最新的含 opt/ros/humble 的 deps 目录。
set -e
deps=${ROS_DEPS_DIR:-}
if [[ -z "$deps" ]]; then
    for candidate in $(ls -td "${HOME}"/ldy/logs/*/deps 2>/dev/null); do
        if [[ -d "$candidate/opt/ros/humble" ]]; then deps=$candidate; break; fi
    done
fi
[[ -n "$deps" && -d "$deps/opt/ros/humble" ]] ||
    { echo "找不到 ROS 覆盖层（ROS_DEPS_DIR 或 ~/ldy/logs/*/deps）" >&2; exit 1; }
(( $# )) || { echo "用法：$0 <命令> [参数...]" >&2; exit 2; }
exec unshare --user --map-root-user --mount -- bash -c '
deps=$1; shift
mount -t overlay overlay -o "lowerdir=$deps/usr:/usr" /usr
mount -t overlay overlay -o "lowerdir=$deps/opt/ros/humble:/opt/ros/humble" /opt/ros/humble
export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu
exec "$@"
' _ "$deps" "$@"
