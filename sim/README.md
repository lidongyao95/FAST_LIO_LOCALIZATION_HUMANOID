# G1 MuJoCo localization and navigation demos (ROS 2 Humble)

Run from the repository root:

```bash
./scripts/run_g1_mujoco.sh
# No windows:
./scripts/run_g1_mujoco.sh headless:=true rviz:=false
```

Stop with Ctrl+C in the launch terminal, or `./scripts/stop_g1_mujoco.sh`. The demo uses ROS_DOMAIN_ID=71 by
 default; export ROS_DOMAIN_ID before launching to change it.

This file documents the original supported-base sensor/localization demo. The
free-walking controller, map-save/restart flow, Nav2, dynamic obstacle service
and staged startup are documented in
[`NAVIGATION_MANUAL.md`](../NAVIGATION_MANUAL.md). Those walking demos use
`ROS_DOMAIN_ID=72` by default.

Both entry points default to NVIDIA PRIME rendering when `DISPLAY` is set.
The environment is defined in `scripts/walking_env.sh`:

```text
MUJOCO_GL=glfw
__NV_PRIME_RENDER_OFFLOAD=1
__GLX_VENDOR_LIBRARY_NAME=nvidia
```

Set `G1_MUJOCO_GPU=0` before launch to retain software rendering explicitly.

## Scope

The `run_g1_mujoco.sh` entry is a sensor/localization test rig rather than a
walking demo. The
unmodified official G1 29-DoF MJCF is included in a generated scene. A MuJoCo
mocap weld supports the pelvis. After 12 seconds it follows a slow scripted
translation. Joint PD control holds the zero pose. No external G1 controller,
private simulation repository, or ground-truth odometry is used by FAST-LIO.

MuJoCo ray casting generates 10 Hz XYZI PointCloud2 on `/livox/lidar`.
The model's pelvis IMU publishes at 500 Hz on `/livox/imu`.
Both use epoch-aligned MuJoCo acquisition timestamps, paced to wall time;
use_sim_time is false. Rendering delays do not change IMU integration intervals. Rays within each cloud
are simultaneous (no rolling scan distortion). The LiDAR is 0.45 m above the
pelvis IMU with identity rotation. This is an explicit synthetic sensor mount,
not the hardware MID360 calibration. Room geometries use visible group 0, and
the LiDAR ray mask also uses group 0, so the floor, walls and obstacles are
visible while retaining collision and ray-cast behavior. Self returns are
excluded. The original upstream model and its mesh assets are not edited.

The generated room map and scene come from the same geometry definition.
They are written to `sim/generated/`, preserving `data/map.ply`. The supplied
map is not assumed to match this synthetic room. `/g1_sim/ground_truth` is
published only for evaluation and is not subscribed to by the LIO/localizer.

The pipeline is:

```
G1 MuJoCo sensors -> FAST-LIO -> Open3D map registration -> /localization_3d
```

RViz shows the room map, point cloud and estimated path; the separate MuJoCo
window shows G1. This entry starts no navigation or walking policy. Use
`NAVIGATION_MANUAL.md` for the complete free-walking workflow.

## Dependencies and provenance

All dependencies are local to `.deps/`, except the system ROS Humble, PCL,
OpenCV, Eigen and their normal Ubuntu dependencies. The launch script clears
workspace overlays and loads only `/opt/ros/humble` plus this repository.

Official source snapshots fetched for this setup:

- https://github.com/unitreerobotics/unitree_mujoco
  `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`
- https://github.com/Livox-SDK/livox_ros_driver2
  `21445540f0d100dc86a7e6df312dd70bbdb4afdf`
- https://github.com/Livox-SDK/Livox-SDK2
  `c0796f04c143143899c87a773d9f6b7136453c0b`

These are pinned official upstream snapshots, not a claim of a tagged stable
release. MuJoCo Python is 3.5.0, NumPy 1.26.4 in a Python 3.10 venv with system
site packages (for ROS). Open3D is Ubuntu Jammy's 0.14.1+dfsg-7build3, unpacked
under `.deps/root` with its missing runtime libraries; no sudo install was
performed. Livox SDK is built under `.deps/sdk`; the driver is built against
that SDK, not `/usr/local` or another workspace.

## Rebuild on this configured machine

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL=3
colcon build --base-paths .deps/src/livox_ros_driver2 FAST_LIO open3d_loc \
  --packages-select livox_ros_driver2 fast_lio open3d_loc --executor sequential \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_LIBRARY="$PWD/.deps/sdk/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$PWD/.deps/sdk/include" \
  -DOpen3D_DIR="$PWD/.deps/root/usr/lib/x86_64-linux-gnu/cmake/Open3D" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DBUILD_TESTING=OFF
```

This demo has separate config and launch files; the original real-robot
configuration remains available. `Open3D_DIR` is now supplied through CMake
rather than hardcoding an author's home directory.

Validate the running demo in another terminal:

```bash
export ROS_DOMAIN_ID=71
source scripts/walking_env.sh
/usr/bin/python3 scripts/check_g1_localization.py --seconds 30
```

The timestamp-paired measurements are saved to
`sim/generated/validation.json`. Debug logs go to `sim/generated/logs/`.
