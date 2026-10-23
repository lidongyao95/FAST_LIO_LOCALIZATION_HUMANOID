#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="$REPO_ROOT/.deps"
VENV="$DEPS/venv"
OPEN3D_LIB="$DEPS/root/usr/lib/x86_64-linux-gnu/libOpen3D.so.0.14.1"
OPEN3D_CONFIG="$DEPS/root/usr/lib/x86_64-linux-gnu/cmake/Open3D/Open3DConfig.cmake"
LIVOX_LIB="$DEPS/sdk/lib/liblivox_lidar_sdk_shared.so"

cd "$REPO_ROOT"

if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "Only x86_64 is currently validated; found $(uname -m)" >&2
    exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble is missing; install it before configuring local dependencies" >&2
    exit 1
fi

missing=()
for command_name in apt-get git cmake g++ make dpkg-deb ldd; do
    command -v "$command_name" >/dev/null 2>&1 || missing+=("$command_name")
done
if ((${#missing[@]})); then
    echo "Missing system commands: ${missing[*]}" >&2
    echo "Install apt, build-essential, cmake, git, dpkg and binutils before rerunning." >&2
    exit 1
fi

python_version="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "$python_version" != "3.10" ]]; then
    echo "Expected /usr/bin/python3 3.10 for ROS 2 Humble; found $python_version" >&2
    exit 1
fi

echo "[1/4] Initialize source submodules"
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive

livox_driver="$DEPS/src/livox_ros_driver2"
cp -f "$livox_driver/package_ROS2.xml" "$livox_driver/package.xml"
mkdir -p "$livox_driver/launch"
cp -f "$livox_driver"/launch_ROS2/* "$livox_driver/launch/"

echo "[2/4] Configure MuJoCo Python environment"
python_packages_ok() {
    "$VENV/bin/python" - <<'PY'
import mujoco
import numpy
assert mujoco.__version__ == "3.5.0", mujoco.__version__
assert numpy.__version__ == "1.26.4", numpy.__version__
PY
}

if [[ ! -x "$VENV/bin/python" ]]; then
    if /usr/bin/python3 -c 'import ensurepip' >/dev/null 2>&1; then
        /usr/bin/python3 -m venv "$VENV"
    elif command -v uv >/dev/null 2>&1; then
        uv venv --python /usr/bin/python3 "$VENV"
    else
        echo "Install python3-venv or uv before creating the Python environment" >&2
        exit 1
    fi
fi

if ! python_packages_ok >/dev/null 2>&1; then
    if "$VENV/bin/python" -m pip --version >/dev/null 2>&1; then
        "$VENV/bin/python" -m pip install --upgrade pip
        "$VENV/bin/python" -m pip install 'mujoco==3.5.0' 'numpy==1.26.4'
    elif command -v uv >/dev/null 2>&1; then
        uv pip install --python "$VENV/bin/python" 'mujoco==3.5.0' 'numpy==1.26.4'
    else
        echo "pip and uv are unavailable; install python3-venv or uv" >&2
        exit 1
    fi
fi
python_packages_ok

echo "[3/4] Configure Open3D C++ library"
open3d_runtime_ok() {
    [[ -f "$OPEN3D_LIB" && -f "$OPEN3D_CONFIG" ]] || return 1
    local unresolved
    unresolved="$(
        LD_LIBRARY_PATH="$DEPS/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}" \
            ldd "$OPEN3D_LIB" 2>/dev/null | grep 'not found' || true
    )"
    [[ -z "$unresolved" ]]
}

if ! open3d_runtime_ok; then
    mkdir -p "$DEPS/debs" "$DEPS/root"
    download_dir="$(mktemp -d "$DEPS/debs/open3d.XXXXXX")"
    trap 'rm -rf -- "$download_dir"' EXIT
    (
        cd "$download_dir"
        apt-get download \
            'libopen3d-dev=0.14.1+dfsg-7build3' \
            'libopen3d0.14=0.14.1+dfsg-7build3' \
            'libfilament1.9=1.9.25+dfsg2-4build1' \
            'liblapacke=3.10.0-2ubuntu1' \
            'liblzf1=3.6-3' \
            'libstb0=0.0~git20210910.af1a5bc+ds-1' \
            'libtinyobjloader1=2.0.0~rc5+dfsg1-5build1' \
            'libtmglib3=3.10.0-2ubuntu1'
        for package in ./*.deb; do
            dpkg-deb -x "$package" "$DEPS/root"
        done
    )
    rm -rf -- "$download_dir"
    trap - EXIT
fi

if ! open3d_runtime_ok; then
    echo "Open3D has unresolved shared-library dependencies:" >&2
    LD_LIBRARY_PATH="$DEPS/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}" \
        ldd "$OPEN3D_LIB" | grep 'not found' >&2
    exit 1
fi

echo "[4/4] Build and install Livox SDK2"
cmake -S .deps/src/Livox-SDK2 -B .deps/sdk-build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$DEPS/sdk"
cmake --build .deps/sdk-build --parallel "${G1_BUILD_JOBS:-3}"
cmake --install .deps/sdk-build

if [[ ! -f "$LIVOX_LIB" ]]; then
    echo "Livox SDK installation is incomplete: $LIVOX_LIB is missing" >&2
    exit 1
fi

echo "Local G1 dependencies are ready."
