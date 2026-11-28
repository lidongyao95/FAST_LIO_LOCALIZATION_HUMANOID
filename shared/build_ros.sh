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

# rsync source-only deployment removes the generated ROS-edition manifest.
livox_driver="$REPO_ROOT/.deps/src/livox_ros_driver2"
if [[ -f "$livox_driver/package_ROS2.xml" ]]; then
    cp -f "$livox_driver/package_ROS2.xml" "$livox_driver/package.xml"
fi

required_paths=(
    "$ROS_SETUP"
    .deps/src/livox_ros_driver2/package.xml
    packages/fast_lio/package.xml
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
    echo "Run ./g1 setup and install ROS 2 first." >&2
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
        "$REPO_ROOT/.deps/arm-open3d/open3d141/lib/cmake/Open3D" \
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
# 解包式 Open3D 的传递依赖（lapacke -> tmglib 等）只存在于 .deps 本地目录，
# 链接器和运行时都看不到。构建前先确认全部可解析，避免编到一半才报
# "cannot find -ltmglib"；缺失时给出可执行的补齐命令。
if [[ " $G1_BUILD_PACKAGES " == *" open3d_loc "* ]]; then
    open3d_probe="$REPO_ROOT/shared/open3d_probe.sh"
    if [[ -f "$open3d_probe" ]]; then
        if probe_report="$(bash "$open3d_probe" 2>&1)"; then
            eval "$(bash "$open3d_probe" --env)"
            if [[ -n "${OPEN3D_RUNTIME_LIB:-}" ]]; then
                cmake_args+=("-DCMAKE_EXE_LINKER_FLAGS=-Wl,-rpath-link,$OPEN3D_RUNTIME_LIB")
                echo "Open3D 链接搜索路径：$OPEN3D_RUNTIME_LIB"
            fi
        else
            probe_status=$?
            printf '%s\n' "$probe_report" >&2
            if ((probe_status == 1)); then
                echo "Open3D 传递依赖不完整：链接会在上述库处中断；先按提示补齐再重新构建。" >&2
            else
                echo "未找到可用的本地 Open3D 运行库（.deps/arm-open3d/runtime 或 .deps/root）。" >&2
            fi
            exit 1
        fi
    fi
fi


echo "ROS distro: $ROS_DISTRO | arch: $ARCH | packages: $G1_BUILD_PACKAGES"
colcon build \
    --base-paths .deps/src/livox_ros_driver2 packages/fast_lio packages/open3d_loc \
    --packages-select $G1_BUILD_PACKAGES \
    --executor sequential \
    --event-handlers console_direct+ \
    --cmake-args "${cmake_args[@]}"

expected_outputs=()
if [[ " $G1_BUILD_PACKAGES " == *" livox_ros_driver2 "* ]]; then
    expected_outputs+=(install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node)
fi
if [[ " $G1_BUILD_PACKAGES " == *" fast_lio "* ]]; then
    expected_outputs+=(install/fast_lio/lib/fast_lio/fastlio_mapping)
fi
if [[ " $G1_BUILD_PACKAGES " == *" open3d_loc "* ]]; then
    expected_outputs+=(install/open3d_loc/lib/open3d_loc/global_localization_node
                       install/open3d_loc/lib/open3d_loc/global_initialize)
fi
for output in "${expected_outputs[@]}"; do
    if [[ ! -e "$output" ]]; then
        echo "ROS build finished but expected output is missing: $output" >&2
        exit 1
    fi
done

echo "ROS 2 G1 workspace is ready."
