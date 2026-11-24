# G1 MuJoCo localization and navigation demos (ROS 2 Humble)

For the complete pure-simulation walking and navigation workflow, start with
[`03_simulation.sh`](../scripts/simulation/03_simulation.sh). The `map`
stage drives through the built-in room and records a FAST-LIO map using simulated
LiDAR and IMU. The `nav` stage restarts the simulator and loads that map.
See [the two-stage workflow](../docs/WORKFLOWS.md#纯仿真路线). That unified entry
defaults to ROS domain 74. For a scene generated from a real scan, use
[`scan_preview.sh`](../scripts/simulation/scan_preview.sh) and the
[scan-preview guide](../g1_real_robot_docs/scan_mujoco_preview.md): its runtime
sensors and robot are still simulated. Live PC2 operation follows the separate
[real-robot workflow](../docs/WORKFLOWS.md#实机部署路线).

## Supported-base sensor demo

The commands below start the smaller sensor/localization rig, without the
walking controller or Nav2. Use the workflow above for goal navigation.

Run from the repository root:

```bash
./scripts/simulation/run_g1_mujoco.sh
# No windows:
./scripts/simulation/run_g1_mujoco.sh headless:=true rviz:=false
```

Stop with Ctrl+C in the launch terminal, or `./scripts/simulation/stop_g1_mujoco.sh`. The demo uses ROS_DOMAIN_ID=71 by
 default; export ROS_DOMAIN_ID before launching to change it.

This file documents the original supported-base sensor/localization demo. The
free-walking controller, map-save/restart flow, Nav2, dynamic obstacle service
and staged startup are documented in
[`NAVIGATION_MANUAL.md`](../docs/NAVIGATION_MANUAL.md). Those walking demos use
`ROS_DOMAIN_ID=72` by default.

Both entry points default to NVIDIA PRIME rendering when `DISPLAY` is set.
The environment is defined in `scripts/env/walking_env.sh`:

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

The official model/SDK source revisions, Python environment, Open3D and Livox
build details are maintained in [the configuration reference](../docs/CONFIGURATION.md).
This sensor rig shares those dependencies with the walking demo, but does not
require starting its controller.

## Rebuild on this configured machine

```bash
./scripts/env/setup_g1_local_deps.sh
./scripts/env/build_g1_ros2.sh
```

This demo has separate config and launch files; the original real-robot
configuration remains available. `Open3D_DIR` is now supplied through CMake
rather than hardcoding an author's home directory.

Validate the running demo in another terminal:

```bash
export ROS_DOMAIN_ID=71
source scripts/env/walking_env.sh
/usr/bin/python3 scripts/checks/check_g1_localization.py --seconds 30
```

The timestamp-paired measurements are saved to
`sim/generated/validation.json`. Debug logs go to `sim/generated/logs/`.
