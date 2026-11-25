#!/usr/bin/env python3
"""Regression checks for filtering semantics and paired-scan provenance."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mapping'))
from scene_filters import add_arguments, validate, downsample, filter_points, prepare
from dufomap_clean import write_pcd


def options(**values):
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    args = parser.parse_args([])
    args.radius = 8.
    args.pcd = Path('source.pcd')
    args.out = Path('out')
    args.start = [0., 0.]
    args.auto_start = True
    for key, value in values.items():
        setattr(args, key, value)
    return args


class Filtering(unittest.TestCase):
    def test_robot_relative_near_is_not_map_origin(self):
        points = np.array([[5.1, 0., 1.], [.1, 0., 1.], [5.1, 0., .01]])
        args = options(near_radius=1., scans=Path('scans'))
        result = filter_points(points, np.array([5., 0., 0.]), np.eye(3), np.zeros(3), args)
        np.testing.assert_allclose(result, points[[1, 2]])
        args = options(origin_clear_radius=1.)
        result = filter_points(points, None, np.eye(3), np.zeros(3), args)
        np.testing.assert_allclose(result, points[[0, 2]])

    def test_near_ground_exemption_and_height(self):
        points = np.array([[.1, 0., .01], [.1, 0., 1.], [2., 0., 6.]])
        args = options(origin_clear_radius=1.)
        np.testing.assert_allclose(filter_points(points, None, np.eye(3), np.zeros(3), args), points[:1])
        args.keep_near_ground = False
        self.assertEqual(len(filter_points(points, None, np.eye(3), np.zeros(3), args)), 0)

    def test_downsample_is_centroid_and_handles_negative_coordinates(self):
        result = downsample(np.array([[-.09, 0., 0.], [-.01, 0., 0.], [.02, 0., 0.]]), .1)
        np.testing.assert_allclose(result[:, 0], [-.05, .02])

    def test_rejects_frame_operations_on_merged_pcd(self):
        for override in ({'dufo': True}, {'near_radius': 1.}, {'sample_period': 2.},
                         {'imu_max_accel': .6}, {'voxel': float('nan')}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                validate(options(**override))

    def test_time_buckets_then_imu_no_replacement_and_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scans = root / 'scans'
            scans.mkdir()
            xyz = np.column_stack([np.linspace(2., 3., 200), np.zeros(200), np.ones(200)])
            source = root / 'source.pcd'
            write_pcd(source, xyz)
            for i in range(4):
                np.savez(scans / f'{1000000000+i*100000000}.npz', points=xyz,
                         pose=[0., 0., 0., 1., 0., 0., 0.], body=[0., 0., 0.])
            evidence = root / 'evidence.npz'
            np.savez(evidence, t=[0., .1, .2, .3], body=np.zeros((4, 3)), scores=[1., .1, .1, .1])
            out = root / 'out'
            out.mkdir()
            args = options(pcd=source, scans=scans, evidence=evidence,
                           sample_period=.2, imu_max_accel=.6)
            _, report = prepare(xyz, np.eye(4), args, out)
            self.assertEqual(report['frames_total'], 4)
            self.assertEqual(report['frames_kept'], 1)
            self.assertEqual([f['selected'] for f in report['frame_sources']], [False, False, True, False])
            self.assertTrue((out / 'filtered.pcd').is_file())
            self.assertTrue(json.loads((out / 'filter_report.json').read_text())['source_sha256'])

    def test_missing_body_is_never_silently_treated_as_lidar(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scans = root / 'scans'
            scans.mkdir()
            xyz = np.tile([[2., 0., 1.]], (150, 1))
            source = root / 'source.pcd'
            write_pcd(source, xyz)
            for stamp in (1, 2):
                np.savez(scans / f'{stamp}.npz', points=xyz, pose=[0., 0., 0., 1., 0., 0., 0.])
            out = root / 'out'
            out.mkdir()
            args = options(pcd=source, scans=scans, near_radius=1.)
            with self.assertRaisesRegex(ValueError, 'body'):
                prepare(xyz, np.eye(4), args, out)

    def test_real_dufomap_pipeline(self):
        try:
            from dufomap import dufomap
        except ImportError:
            self.skipTest('DUFOMap not installed')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scans = root / 'scans'
            scans.mkdir()
            y, z = np.meshgrid(np.linspace(-1, 1, 25), np.linspace(.2, 2., 25))
            xyz = np.column_stack([np.full(y.size, 3.), y.ravel(), z.ravel()])
            source = root / 'source.pcd'
            write_pcd(source, xyz)
            for i in range(4):
                np.savez(scans / f'{100+i}.npz', points=xyz,
                         pose=[0., .02*i, 1., 1., 0., 0., 0.], body=[0., .02*i, 0.])
            out = root / 'out'
            out.mkdir()
            args = options(pcd=source, scans=scans, dufo=True, voxel=.06, near_radius=.5)
            points, report = prepare(xyz, np.eye(4), args, out)
            self.assertTrue(report['dufomap_used'])
            self.assertEqual(report['frames_total'], 4)
            self.assertGreaterEqual(len(points), 100)
            self.assertTrue(np.allclose(points[:, 0], 3.))


if __name__ == '__main__':
    unittest.main(verbosity=2)
