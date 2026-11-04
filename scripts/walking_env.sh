#!/usr/bin/env bash
G1_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 distro: honour ROS_DISTRO, otherwise prefer Humble and fall back to Foxy.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    G1_ROS_DISTRO="$ROS_DISTRO"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    G1_ROS_DISTRO=humble
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    G1_ROS_DISTRO=foxy
else
    echo "No ROS 2 Humble/Foxy installation found under /opt/ros." >&2
    return 1 2>/dev/null || exit 1
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/bin:/bin
source "/opt/ros/${G1_ROS_DISTRO}/setup.bash"
if [[ -f "$G1_REPO/install/local_setup.bash" ]]; then
    source "$G1_REPO/install/local_setup.bash"
fi
export ROS_DISTRO="$G1_ROS_DISTRO"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${ROS_LOG_DIR:-$G1_REPO/sim/generated/ros_logs}"

# Architecture-specific library paths, added only when they exist.
g1_lib_paths=()
arch="$(uname -m)"
unitree_lib="$G1_REPO/.deps/src/unitree_sdk2/thirdparty/lib/$arch"
open3d_lib="$G1_REPO/.deps/root/usr/lib/${arch}-linux-gnu"
for path in "$unitree_lib" "$open3d_lib" "$G1_REPO/.deps/sdk/lib" /usr/local/lib; do
    [[ -d "$path" ]] && g1_lib_paths+=("$path")
done
if ((${#g1_lib_paths[@]})); then
    export LD_LIBRARY_PATH="$(IFS=:; echo "${g1_lib_paths[*]}")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
if [[ "${DISPLAY:-}" == :1 && ! -S /tmp/.X11-unix/X1 && -S /tmp/.X11-unix/X0 ]]; then export DISPLAY=:0; fi
if [[ -n "${DISPLAY:-}" ]]; then
    export MUJOCO_GL="${MUJOCO_GL:-glfw}"
    export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-glx}"
    if [[ "${G1_MUJOCO_GPU:-1}" != "0" ]] && command -v nvidia-smi >/dev/null 2>&1; then
        export __NV_PRIME_RENDER_OFFLOAD=1
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
    fi
else
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
fi
cd "$G1_REPO"
mkdir -p "$ROS_LOG_DIR"
