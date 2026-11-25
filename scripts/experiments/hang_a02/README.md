# Hang A 02 offline mapping and MuJoCo trial

These experiment scripts use the recorded 4322 assets under
`~/ldy/dufomap-mask-review`, `~/ldy/g1_a_replay/hang_A_02`, and
`/tmp/hang-a02-clean`. Run scripts from their respective output directories.
`compare.py --mode single` (the default) sweeps individual filters;
`compare.py --mode combined` evaluates the eight range/time/IMU combinations.
Use separate output directories for the two modes. Both write `grids.npz` and
`cloud-*.npy`; the same `render.py`, `rviz_review.py` and `review.rviz` handle
either result. The RViz publisher uses transient local topics and an isolated ROS domain.

Run the original sweep before `plaster-map/generate.py`, which uses the parent
`grids.npz` for the matching body poses and IMU scores. Use the existing DUFOMap
Python dependency path. Run `render.py` with SciPy, scikit-image, Matplotlib,
OpenCV and Pillow to export the mesh and preview. The navigation image is
exported separately from scan evidence and retains unknown cells.

Generate and navigate plaster scenes with the maintained server-side
`03_generate_scene.sh`, `03_preview_scene.sh`, and `03_simulation.sh nav`
workflow in [WORKFLOWS B3](../../../docs/WORKFLOWS.md#b3-可选在仿真机预演实机地图).
The same reconstructed surface supplies display, collision, LiDAR and paired maps.

See [the filter and visualization guide](../../../g1_real_robot_docs/hang_a02_point_cloud_filters.md)
for tuning parameters, GPU RViz reproduction, and aligned map rerun comparisons.
