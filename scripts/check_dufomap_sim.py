#!/usr/bin/env python3
"""Deterministic ray-cast check of transient obstacle removal and wall retention."""
import numpy as np

from dufomap_clean import clean


def simulate(mode):
    for index in range(36):
        moved = index >= 12 and mode != "turn_only"
        origin = np.array([0., (index - 12) * 0.025 if moved else 0., 1.2])
        yaw = 0.25 if index >= 12 else 0.0
        wide = moved and mode == "wide_revisit"
        azimuth = np.linspace(-.9, .9, 321) if wide else np.linspace(-.55, .55, 201)
        elevation = np.linspace(-.55, .55, 201) if wide else np.linspace(-.23, .25, 91)
        az, el = np.meshgrid(azimuth + yaw, elevation)
        directions = np.column_stack([np.ones(az.size), np.tan(az.ravel()), np.tan(el.ravel())])
        distance = np.full(len(directions), 5.)
        hit = np.zeros(len(directions), dtype=bool)
        if not moved:
            # Intersect a box in front of the wall, including rays entering its sides.
            low = np.array([1.8, -.35, .35]) - origin
            high = np.array([2.2, .35, 1.95]) - origin
            with np.errstate(divide="ignore"):
                near = np.minimum(low / directions, high / directions).max(axis=1)
                far = np.maximum(low / directions, high / directions).min(axis=1)
            hit = (near <= far) & (far > 0)
            distance[hit] = near[hit]
        yield origin + distance[:, None] * directions, np.r_[origin, 1., 0., 0., 0.], hit


def main():
    for mode in ("turn_only", "revisit", "wide_revisit"):
        frames = list(simulate(mode))
        wall_kept = wall_total = obstacle_kept = obstacle_total = 0
        for (_, mask), (_, _, hit) in zip(clean((p, pose) for p, pose, _ in frames), frames):
            wall_kept += np.count_nonzero(mask & ~hit)
            wall_total += np.count_nonzero(~hit)
            obstacle_kept += np.count_nonzero(mask & hit)
            obstacle_total += np.count_nonzero(hit)
        retained_wall = wall_kept / wall_total
        removed_obstacle = 1 - obstacle_kept / obstacle_total
        print(f"{mode}: wall retained {retained_wall:.2%}, obstacle removed {removed_obstacle:.2%}")
        assert retained_wall >= .98, "Static wall eroded"
        if mode == "wide_revisit":
            assert removed_obstacle >= .85, "Transient obstacle remained after wide revisit"
        elif mode == "revisit":
            assert removed_obstacle >= .25, "Narrow revisit did not clear the observed interior"
        else:
            assert removed_obstacle <= .05, "Continuously occupied obstacle was removed"


if __name__ == "__main__":
    main()
