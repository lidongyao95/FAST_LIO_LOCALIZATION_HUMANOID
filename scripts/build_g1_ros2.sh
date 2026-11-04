#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ROS 2 distro: honour ROS_DISTRO, otherwise prefer Humble and fall back to Foxy.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    export ROS_DISTRO=humble
    ROS_SETUP=/opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    export ROS_DISTRO=foxy
    ROS_SETUP=/opt/ros/foxy/setup.bash
else
    echo "No ROS 2 Humble/Foxy installation found under /opt/ros." >&2
    exit 1
fi

ARCH="$(uname -m)"
# Default keeps the historical x86 behaviour; on the robot only driver + FAST-LIO
# are needed until Open3D is available for aarch64.
G1_BUILD_PACKAGES="${G1_BUILD_PACKAGES:-livox_ros_driver2 fast_lio open3d_loc}"

required_paths=(
    "$ROS_SETUP"
    .deps/src/livox_ros_driver2/package.xml
    FAST_LIO/package.xml
)
missing=()
for path in "${required_paths[@]}"; do
    [[ -e "$path" ]] || missing+=("$path")
done
for command_name in colcon /usr/bin/python3; do
    command -v "$command_name" >/dev/null 2>&1 || missing+=("$command_name")
done

if ((${#missing[@]})); then
    printf 'Missing ROS build prerequisites:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    echo "Run scripts/setup_g1_local_deps.sh and install ROS 2 first." >&2
    exit 1
fi

# Livox SDK: prefer the local build, otherwise use a system-wide installation.
LIVOX_LIB="$REPO_ROOT/.deps/sdk/lib/liblivox_lidar_sdk_shared.so"
LIVOX_INC="$REPO_ROOT/.deps/sdk/include"
if [[ ! -f "$LIVOX_LIB" ]]; then
    for candidate in /usr/local/lib/liblivox_lidar_sdk_shared.so /usr/lib/liblivox_lidar_sdk_shared.so; do
        if [[ -f "$candidate" ]]; then
            LIVOX_LIB="$candidate"
            LIVOX_INC="/usr/local/include"
            break
        fi
    done
fi
if [[ ! -f "$LIVOX_LIB" ]]; then
    echo "Livox SDK2 shared library not found (checked .deps, /usr/local, /usr)." >&2
    exit 1
fi

# Open3D is only required by open3d_loc.
OPEN3D_DIR=""
if [[ " $G1_BUILD_PACKAGES " == *" open3d_loc "* ]]; then
    for candidate in \
        "$REPO_ROOT/.deps/root/usr/lib/${ARCH}-linux-gnu/cmake/Open3D" \
        /usr/local/lib/cmake/Open3D \
        /usr/lib/${ARCH}-linux-gnu/cmake/Open3D; do
        if [[ -d "$candidate" ]]; then
            OPEN3D_DIR="$candidate"
            break
        fi
    done
    if [[ -z "$OPEN3D_DIR" ]]; then
        echo "open3d_loc requested but Open3D CMake config was not found for ${ARCH}." >&2
        echo "Set G1_BUILD_PACKAGES='livox_ros_driver2 fast_lio' to build without localization." >&2
        exit 1
    fi
fi

# Build against system ROS only. The runtime overlay is intentionally excluded.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export PATH=/usr/bin:/bin
set +u
source "$ROS_SETUP"
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${G1_BUILD_JOBS:-3}}"

cmake_args=(
    -DROS_EDITION=ROS2
    -DDISTRO_ROS="$ROS_DISTRO"
    -DLIVOX_LIDAR_SDK_LIBRARY="$LIVOX_LIB"
    -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$LIVOX_INC"
    -DPYTHON_EXECUTABLE=/usr/bin/python3
    -DPython3_EXECUTABLE=/usr/bin/python3
    -DBUILD_TESTING=OFF
)
if [[ -n "$OPEN3D_DIR" ]]; then
    cmake_args+=(-DOpen3D_DIR="$OPEN3D_DIR")
fi

echo "ROS distro: $ROS_DISTRO | arch: $ARCH | packages: $G1_BUILD_PACKAGES"
colcon build \
    --base-paths .deps/src/livox_ros_driver2 FAST_LIO open3d_loc \
    --packages-select $G1_BUILD_PACKAGES \
    --executor sequential \
    --cmake-args "${cmake_args[@]}"

expected_outputs=()
if [[ " $G1_BUILD_PACKAGES " == *" livox_ros_driver2 "* ]]; then
    expected_outputs+=(install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node)
fi
if [[ " $G1_BUILD_PACKAGES " == *" fast_lio "* ]]; then
    expected_outputs+=(install/fast_lio/lib/fast_lio/fastlio_mapping)
fi
if [[ " $G1_BUILD_PACKAGES " == *" open3d_loc "* ]]; then
    expected_outputs+=(install/open3d_loc/lib/open3d_loc/global_localization_node)
fi
for output in "${expected_outputs[@]}"; do
    if [[ ! -e "$output" ]]; then
        echo "ROS build finished but expected output is missing: $output" >&2
        exit 1
    fi
done

echo "ROS 2 G1 workspace is ready."
echo "In each runtime terminal, run: source scripts/walking_env.sh"
