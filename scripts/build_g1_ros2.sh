#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

required_paths=(
    /opt/ros/humble/setup.bash
    .deps/sdk/lib/liblivox_lidar_sdk_shared.so
    .deps/sdk/include
    .deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D/Open3DConfig.cmake
    .deps/src/livox_ros_driver2/package.xml
    FAST_LIO/package.xml
    open3d_loc/package.xml
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
    echo "Run scripts/setup_g1_local_deps.sh and install ROS 2 Humble first." >&2
    exit 1
fi

# Build against system ROS only. The runtime overlay is intentionally excluded.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
export PATH=/usr/bin:/bin
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${G1_BUILD_JOBS:-3}}"

colcon build \
    --base-paths .deps/src/livox_ros_driver2 FAST_LIO open3d_loc \
    --packages-select livox_ros_driver2 fast_lio open3d_loc \
    --executor sequential \
    --cmake-args \
        -DROS_EDITION=ROS2 \
        -DDISTRO_ROS=humble \
        -DLIVOX_LIDAR_SDK_LIBRARY="$REPO_ROOT/.deps/sdk/lib/liblivox_lidar_sdk_shared.so" \
        -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$REPO_ROOT/.deps/sdk/include" \
        -DOpen3D_DIR="$REPO_ROOT/.deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D" \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
        -DBUILD_TESTING=OFF

expected_outputs=(
    install/local_setup.bash
    install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node
    install/fast_lio/lib/fast_lio/fastlio_mapping
    install/open3d_loc/lib/open3d_loc/global_localization_node
)
for output in "${expected_outputs[@]}"; do
    if [[ ! -e "$output" ]]; then
        echo "ROS build finished but expected output is missing: $output" >&2
        exit 1
    fi
done

echo "ROS 2 G1 workspace is ready."
echo "In each runtime terminal, run: source scripts/walking_env.sh"
