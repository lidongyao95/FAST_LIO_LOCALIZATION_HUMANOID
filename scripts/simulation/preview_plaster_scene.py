"""Inspect a generated plaster scene without running physics, ROS or locomotion."""
import json
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer


def main():
    directory = Path(sys.argv[1]).resolve()
    meta = json.loads((directory / 'metadata.json').read_text())
    if meta.get('scene_representation') != 'continuous plaster mesh':
        raise SystemExit('[失败] 这不是石膏网格场景，请先重新生成。')
    model = mujoco.MjModel.from_xml_path(str(directory / 'scene.xml'))
    spawn = meta['spawn_xy_m']
    model.qpos0[:2] = spawn
    model.body('support').pos[:2] = spawn
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(f'[预览] 石膏网格 {meta["surface_triangles"]} 个三角面；出生点 {spawn}', flush=True)
    print('鼠标旋转、平移、缩放；关闭窗口结束。静态预览，不运行物理或导航。', flush=True)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.geomgroup[3] = 0
        viewer.cam.lookat[:] = [spawn[0], spawn[1], .8]
        viewer.cam.distance = min(12., meta['crop_radius_m'] * 1.5)
        viewer.cam.azimuth, viewer.cam.elevation = 135., -35.
        while viewer.is_running():
            viewer.sync()
            time.sleep(.02)


if __name__ == '__main__':
    main()
