# Hang A 02 offline mapping and MuJoCo trial

These experiment scripts use the recorded 4322 assets under
`~/ldy/dufomap-mask-review`, `~/ldy/g1_a_replay/hang_A_02`, and
`/tmp/hang-a02-clean`. Run scripts from their respective output directories.
The sweep and combined sweep produce timestamped evidence grids and comparison
plots. The RViz publishers use transient local topics and an isolated ROS domain.

Run the original sweep before `plaster-map/generate.py`, which uses the parent
`grids.npz` for the matching body poses and IMU scores. Use the existing DUFOMap
Python dependency path. Run `render.py` with SciPy, scikit-image, Matplotlib,
OpenCV and Pillow to export the mesh and preview. The navigation image is
exported separately from scan evidence and retains unknown cells.

In `plaster-map/mujoco`, run `trial.py` with the walking simulation environment
and `MUJOCO_GL=egl` to generate the scene, OBJ, screenshots and probe results.
Run `navigation_assets.py` with NumPy and OpenCV to reproject the occupancy map
and localization point cloud into the scene's world frame.

See [the filter and visualization guide](../../g1_real_robot_docs/hang_a02_point_cloud_filters.md)
for tuning parameters, GPU RViz reproduction, and aligned map rerun comparisons.
