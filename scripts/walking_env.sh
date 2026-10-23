#!/usr/bin/env bash
G1_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/bin:/bin
source /opt/ros/humble/setup.bash
source "$G1_REPO/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${ROS_LOG_DIR:-$G1_REPO/sim/generated/ros_logs}"
export LD_LIBRARY_PATH="$G1_REPO/.deps/src/unitree_sdk2/thirdparty/lib/x86_64:$G1_REPO/.deps/root/usr/lib/x86_64-linux-gnu:$G1_REPO/.deps/sdk/lib:$LD_LIBRARY_PATH"
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
