#!/usr/bin/env python3
"""Move a supported G1 sensor rig through two closed loops with random boxes."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy

from g1_mujoco_sensors import OUT, Sensors, generate


PERIOD = 30.0
WARMUP = 3.0
BOX_HALF_SIZE = [.23, .23, .65]


class LoopSensors(Sensors):
    def __init__(self, scene, centers):
        super().__init__(model_path=scene)
        self.centers = centers
        self.support = int(self.model.body("support").mocapid[0])
        self.boxes = [int(self.model.body(f"transient_{i}").mocapid[0])
                      for i in range(len(centers))]
        self.marked = False

    def apply_control(self):
        super().apply_control()
        elapsed = max(0., self.data.time - WARMUP)
        theta = 2 * np.pi * elapsed / PERIOD
        self.data.mocap_pos[self.support, :2] = [1.15 * (1 - np.cos(theta)), 1.15 * np.sin(theta)]
        for i, (mocap, center) in enumerate(zip(self.boxes, self.centers)):
            active = elapsed < PERIOD and i * 4 <= elapsed < 19 + i * 3
            self.data.mocap_pos[mocap] = [*center, BOX_HALF_SIZE[2] if active else -5.]
        if not self.marked and elapsed >= PERIOD:
            print(f"CLEAR_PASS_NS={self.epoch_ns + round(self.data.time * 1e9)}", flush=True)
            self.marked = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=4322)
    parser.add_argument("--speed", type=float, default=1.)
    parser.add_argument("truth", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    rng = np.random.default_rng(args.seed)
    centers = np.array([[-1.25, 1.1], [1.1, 2.25], [2.7, -1.9], [-.95, -1.4]])
    centers += rng.uniform(-.2, .2, centers.shape)
    generate()
    scene = ET.parse(OUT / "scene.xml")
    world = scene.getroot().find("worldbody")
    for index, (x, y) in enumerate(centers):
        body = ET.SubElement(world, "body", name=f"transient_{index}", mocap="true",
                             pos=f"{x} {y} -5")
        ET.SubElement(body, "geom", type="box", size=" ".join(map(str, BOX_HALF_SIZE)),
                      group="0", rgba="1 .2 .1 1")
    path = OUT / "dufomap_loop.xml"
    scene.write(path)
    args.truth.write_text(json.dumps({"seed": args.seed, "centers": centers.tolist(),
                                     "half_size": BOX_HALF_SIZE}))
    rclpy.init()
    node = LoopSensors(path, centers)
    try:
        started = time.monotonic()
        while rclpy.ok() and node.data.time < WARMUP + 2 * PERIOD + 2:
            node.step()
            rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(max(0., started + node.data.time / args.speed - time.monotonic()))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
