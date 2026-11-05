#!/usr/bin/env python3
"""Official G1 model with a supported base and ray-cast LiDAR for LIO testing.

This is a localization sensor test rig, not a walking controller. The original
upstream model is included unchanged. No ground-truth pose enters FAST-LIO.
"""
import argparse
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sim/generated'
# center and half-size; asymmetric room and obstacles give observable geometry.
BOXES = [((0, -4, 1.5), (6, .1, 1.5)),
         ((0, 4, 1.5), (6, .1, 1.5)),
         ((-6, 0, 1.5), (.1, 4, 1.5)),
         ((6, 0, 1.5), (.1, 4, 1.5)),
         ((3, 1.5, .8), (.5, .6, .8)),
         ((-2, -2, 1), (.4, .7, 1)),
         ((-3, 2, .6), (.8, .4, .6))]


def generate():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'logs').mkdir(exist_ok=True)
    model = ROOT / '.deps/src/unitree_mujoco/unitree_robots/g1/g1_29dof.xml'
    scene = ET.Element('mujoco', model='G1 localization sensor rig')
    ET.SubElement(scene, 'include', file=str(model))
    ET.SubElement(scene, 'compiler', angle='radian', meshdir=str(model.parent / 'meshes'))
    ET.SubElement(scene, 'option', timestep='0.002', gravity='0 0 -9.81')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'headlight', ambient='.45 .45 .45',
                  diffuse='.65 .65 .65', specular='.15 .15 .15')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='0 0 5', dir='0 0 -1',
                  ambient='.35 .35 .35', diffuse='.8 .8 .8')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='10 10 .1',
                  rgba='.38 .44 .5 1', group='0')
    for i, (pos, size) in enumerate(BOXES):
        ET.SubElement(world, 'geom', name=f'room_{i}', type='box',
                      pos=' '.join(map(str, pos)), size=' '.join(map(str, size)),
                      rgba='.78 .82 .88 1', group='0')
    dynamic_obstacle = ET.SubElement(
        world, 'body', name='nav_dynamic_obstacle', mocap='true', pos='0 0 -5')
    ET.SubElement(
        dynamic_obstacle, 'geom', name='nav_dynamic_obstacle_geom', type='box',
        size='.35 .35 .75', rgba='.9 .35 .2 1', group='0')
    ET.SubElement(world, 'body', name='support', mocap='true', pos='0 0 .793')
    eq = ET.SubElement(scene, 'equality')
    ET.SubElement(eq, 'weld', body1='support', body2='pelvis', solref='.004 1')
    ET.ElementTree(scene).write(OUT / 'scene.xml')
    points = []
    # Sample exactly the same world geometry as the ray caster, in world frame.
    for pos, half in BOXES:
        for axis in range(3):
            a, b = [i for i in range(3) if i != axis]
            u, v = np.meshgrid(np.arange(-half[a], half[a]+.04, .08),
                               np.arange(-half[b], half[b]+.04, .08))
            for sign in (-1, 1):
                p = np.tile(pos, (u.size, 1)).astype(float)
                p[:, axis] += sign * half[axis]
                p[:, a] += u.ravel()
                p[:, b] += v.ravel()
                points.append(p)
    x, y = np.meshgrid(np.arange(-6, 6, .08), np.arange(-4, 4, .08))
    points.append(np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)]))
    points = np.concatenate(points)
    with (OUT / 'room.ply').open('w') as f:
        f.write(f'ply\nformat ascii 1.0\nelement vertex {len(points)}\n'
                'property float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f, points, fmt='%.4f')
    # Nav2 uses a separate 2-D occupancy grid with the same world origin.
    resolution = .05
    ox, oy = -6.4, -4.4
    width, height = 256, 176
    xs = ox + (np.arange(width) + .5)*resolution
    ys = oy + (np.arange(height) + .5)*resolution
    X, Y = np.meshgrid(xs, ys)
    image = np.full((height, width), 205, dtype=np.uint8)
    image[(abs(X) < 6.) & (abs(Y) < 4.)] = 254
    for (cx, cy, _), (hx, hy, _) in BOXES:
        image[(abs(X-cx) <= hx) & (abs(Y-cy) <= hy)] = 0
    with (OUT/'room.pgm').open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(image[::-1].tobytes())
    (OUT/'room.yaml').write_text(
        f'image: room.pgm\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


class Sensors(Node):
    def __init__(self, move=False, model_path=None, transient_obstacle=False):
        super().__init__('g1_mujoco_sensors')
        self.model = mujoco.MjModel.from_xml_path(str(model_path or OUT / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.move = move
        self.obstacle_mocap = (
            self.model.body("nav_dynamic_obstacle").mocapid[0] if transient_obstacle else None
        )
        self.imu = self.create_publisher(Imu, '/livox/imu', 50)
        self.cloud = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.truth = self.create_publisher(PoseStamped, '/g1_sim/ground_truth', 10)
        self.imu_site = self.model.site('imu').id
        self.base = self.model.body('pelvis').id
        az, el = np.meshgrid(np.linspace(-np.pi, np.pi, 180, endpoint=False),
                              np.deg2rad(np.linspace(-35, 35, 24)))
        self.rays = np.ascontiguousarray(np.column_stack([
            (np.cos(el)*np.cos(az)).ravel(), (np.cos(el)*np.sin(az)).ravel(),
            np.sin(el).ravel()]))
        self.groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.next_scan = 0.
        self.epoch_ns = self.get_clock().now().nanoseconds
        mujoco.mj_forward(self.model, self.data)
        self.get_logger().info('Official G1 sensor rig: IMU 500 Hz, LiDAR 10 Hz; supported base')

    def apply_control(self):
        # Delayed, smooth scripted base translation; no locomotion policy.
        t = max(0., self.data.time - 12.)
        if self.move:
            self.data.mocap_pos[0, 0] = .5 * (1 - np.cos(.08*t))
        if self.obstacle_mocap is not None:
            self.data.mocap_pos[self.obstacle_mocap] = (
                [2., 0., .95] if self.data.time < 8. else [0., 0., -5.]
            )
        self.data.ctrl[:] = -80*self.data.qpos[7:] - 3*self.data.qvel[6:]

    def step(self):
        self.apply_control()
        # Update the mocap target and sample the IMU at EVERY physics step.
        # Decimating a stepped constraint acceleration aliases its impulses.
        mujoco.mj_step(self.model, self.data)
        # Rendering can stall wall-clock publication. Keep IMU integration and
        # LiDAR acquisition stamps tied to the same MuJoCo time, epoch aligned.
        stamp = rclpy.time.Time(nanoseconds=self.epoch_ns + round(self.data.time*1e9)).to_msg()
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, 'imu_link'
        q = self.data.sensor('imu_quat').data
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, q)
        for field, sensor in [('angular_velocity', 'imu_gyro'), ('linear_acceleration', 'imu_acc')]:
            v = getattr(msg, field)
            v.x, v.y, v.z = map(float, self.data.sensor(sensor).data)
        self.imu.publish(msg)
        if self.data.time >= self.next_scan:
            self.next_scan = self.data.time + .1
            rotation = self.data.site_xmat[self.imu_site].reshape(3, 3)
            origin = self.data.site_xpos[self.imu_site] + rotation @ np.array([0., 0., .45])
            directions = np.ascontiguousarray(self.rays @ rotation.T)
            distances = np.empty(len(directions))
            geomids = np.empty(len(directions), dtype=np.int32)
            mujoco.mj_multiRay(self.model, self.data, origin, directions.ravel(),
                              self.groups, 1, self.base, geomids, distances, None,
                              len(directions), 30.)
            valid = (distances > .5) & (distances < 30.)
            xyz = self.rays[valid] * distances[valid, None]
            packed = np.column_stack([xyz, np.ones(len(xyz))]).astype('<f4')
            pc = PointCloud2()
            pc.header.stamp, pc.header.frame_id = stamp, 'lidar_link'
            pc.height, pc.width = 1, len(packed)
            pc.fields = [PointField(name=name, offset=4*i, datatype=7, count=1)
                         for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
            pc.point_step, pc.row_step, pc.is_dense = 16, 16*len(packed), True
            pc.data = packed.tobytes()
            self.cloud.publish(pc)
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, self.data.site_xpos[self.imu_site])
            pose.pose.orientation = msg.orientation
            self.truth.publish(pose)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--move', action='store_true')
    parser.add_argument('--generate-only', action='store_true')
    parser.add_argument('--transient-obstacle', action='store_true',
                        help='Place a box in front of the sensor for the first 8 simulated seconds')
    args = parser.parse_args()
    generate()
    if args.generate_only:
        return
    rclpy.init()
    node = Sensors(args.move, transient_obstacle=args.transient_obstacle)
    viewer = None
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(node.model, node.data)
        viewer.cam.distance = 9
        viewer.cam.lookat[:] = [0, 0, 1]
    start = time.monotonic()
    next_view = start
    try:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            node.step()
            rclpy.spin_once(node, timeout_sec=0.)
            if viewer is not None and time.monotonic() >= next_view:
                viewer.sync()
                next_view = time.monotonic() + 1./30.
            time.sleep(max(0., start + node.data.time - time.monotonic()))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if viewer:
            viewer.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
