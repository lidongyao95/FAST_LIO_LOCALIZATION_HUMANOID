"""Reusable filtering for the server-local plaster pipeline.

PCD-only and exact paired-scan inputs deliberately have different capabilities.
"""
import hashlib
from importlib import metadata
import json
from pathlib import Path

import numpy as np
from dufomap_clean import clean, read_scans, scan_paths, write_pcd


def add_arguments(parser):
    parser.add_argument('--voxel', type=float, default=0., help='输入点云降采样间距（米）；0 关闭，不生成体素显示')
    parser.add_argument('--origin-clear-radius', type=float, default=0., help='忽略地图原点周围非地面点（米），0 关闭；不是沿轨迹的机器人近距过滤')
    parser.add_argument('--near-radius', type=float, default=0., help='逐帧机器人周围非地面点剔除半径（米），需要 --scans 和逐帧位置')
    parser.add_argument('--near-reference', choices=('body', 'lidar-origin'), default='body', help='近距参考点；默认机器人 body，雷达射线原点需显式选择')
    parser.add_argument('--keep-near-ground', action='store_true', dest='keep_near_ground', default=True)
    parser.add_argument('--no-keep-near-ground', action='store_false', dest='keep_near_ground')
    parser.add_argument('--floor-band', type=float, default=.15, help='近距地面豁免带宽（米）')
    parser.add_argument('--resolution', type=float, default=.05, help='二维规划栅格分辨率（米）')
    parser.add_argument('--height-min', type=float, default=-.15, help='相对拟合地面的最低保留高度（米）')
    parser.add_argument('--height-max', type=float, default=4.5, help='相对拟合地面的最高保留高度（米）')
    parser.add_argument('--scans', type=Path, help='同轮逐帧世界点云和 pose 的 NPZ 目录')
    parser.add_argument('--sample-period', type=float, default=0., help='逐帧时间抽样间隔（秒），0 保留全部帧；需 --scans')
    parser.add_argument('--evidence', type=Path, help='与扫描精确配对的 t/body/scores NPZ，用于机器人近距及 IMU 筛帧')
    parser.add_argument('--imu-max-accel', type=float, help='IMU 去重力加速度 P95 上限（米/秒²），需 --evidence')
    parser.add_argument('--no-imu-filter', action='store_true', help='关闭 IMU 筛帧')
    parser.add_argument('--dufo', action='store_true', help='启用 DUFOMap，必须提供同轮 --scans')
    parser.add_argument('--dufo-d-s', '--d-s', type=float, default=.2)
    parser.add_argument('--dufo-d-p', '--d-p', type=int, default=1)
    parser.add_argument('--plaster-grid', type=float, default=.10, help='石膏重建采样间距（米），不是显示体素')
    parser.add_argument('--plaster-ground-cut', type=float, default=.16)
    parser.add_argument('--plaster-height', type=float, default=2.5)
    parser.add_argument('--max-faces', type=int, default=4000, help='网格简化目标面数；拓扑约束可能使实际值稍高')


def validate(args):
    numeric = ['voxel','origin_clear_radius','near_radius','height_min','height_max',
               'sample_period','dufo_d_s','dufo_d_p','plaster_grid','plaster_ground_cut',
               'plaster_height','max_faces','floor_band','resolution']
    if args.imu_max_accel is not None:
        numeric.append('imu_max_accel')
    if not all(np.isfinite(getattr(args, key)) for key in numeric):
        raise ValueError('所有过滤和重建参数必须是有限数')
    if any(getattr(args, key) < 0 for key in ('voxel','origin_clear_radius','near_radius',
                                             'sample_period','dufo_d_s','dufo_d_p')):
        raise ValueError('半径、间距、时间和 DUFOMap 裕量不能为负数')
    if args.floor_band <= 0 or not .01 <= args.resolution <= .5:
        raise ValueError('--floor-band 须为正数，--resolution 允许 0.01–0.5 m')
    if args.height_min >= args.height_max:
        raise ValueError('--height-min 必须小于 --height-max')
    if not .02 <= args.plaster_grid <= .5 or not .05 <= args.plaster_ground_cut <= .6:
        raise ValueError('--plaster-grid 允许 0.02–0.5 m，--plaster-ground-cut 允许 0.05–0.6 m')
    if not args.plaster_ground_cut < args.plaster_height <= 10 or args.max_faces < 100:
        raise ValueError('石膏最高高度须大于地面剔除高度且不超过 10 m，目标面数至少 100')
    if args.imu_max_accel is not None and args.imu_max_accel < 0:
        raise ValueError('--imu-max-accel 不能为负数')
    if not args.scans and (args.dufo or args.near_radius > 0 or args.sample_period > 0 or args.evidence):
        raise ValueError('DUFOMap、逐帧近距、时间抽样和配对证据需要 --scans；合并 PCD 的原点过滤请用 --origin-clear-radius')
    if args.imu_max_accel is not None and not args.no_imu_filter and not args.evidence:
        raise ValueError('IMU 筛帧需要同轮 --evidence，不能从合并 PCD 推断')


def frame_context(args, paths):
    stamps = np.array([int(p.stem) for p in paths], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('扫描时间戳必须严格递增')
    times = (stamps - stamps[0]) * 1e-9
    keep = np.ones(len(paths), dtype=bool)
    if args.sample_period:
        _, first = np.unique(np.floor(times / args.sample_period), return_index=True)
        keep[:] = False
        keep[first] = True
    positions = None
    if args.evidence:
        with np.load(args.evidence, allow_pickle=False) as evidence:
            evidence_times = evidence['t']
            positions, scores = evidence['body'], evidence['scores']
        if evidence_times.shape != times.shape or not np.allclose(times, evidence_times, rtol=0, atol=1e-9):
            raise ValueError('证据相对时间与本轮扫描不匹配，不能复用其他录包的证据')
        if positions.shape != (len(paths), 3) or not np.isfinite(positions).all() or scores.shape != times.shape:
            raise ValueError('配对证据的 body/scores 维度或机器人位置无效')
        if args.imu_max_accel is not None and not args.no_imu_filter:
            keep &= np.isfinite(scores) & (scores <= args.imu_max_accel)
    if not keep.any():
        raise ValueError('没有扫描通过时间/IMU 筛选')
    return stamps, keep, positions


def check_sources(args):
    if not args.pcd.is_file():
        raise ValueError('输入 PCD 不存在')
    if args.evidence and not args.evidence.is_file():
        raise ValueError('--evidence 文件不存在')
    if args.scans:
        paths = scan_paths(args.scans)
        if not args.scans.is_dir() or len(paths) < 2:
            raise ValueError('--scans 至少需要两个以纳秒时间戳命名的配对 NPZ 文件')
        stamps, selected, positions = frame_context(args, paths)
        for i, path in enumerate(paths):
            with np.load(path, allow_pickle=False) as frame:
                if 'points' not in frame:
                    raise ValueError(f'扫描缺少 points：{path}')
                pose = frame['pose']
                if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isclose(np.linalg.norm(pose[3:]), 1.):
                    raise ValueError(f'扫描 pose 无效：{path}')
                if 'stamp_ns' in frame and int(frame['stamp_ns']) != int(stamps[i]):
                    raise ValueError(f'扫描文件名与内部时间戳不一致：{path}')
                if args.near_radius and args.near_reference == 'body' and positions is None and selected[i]:
                    if 'body' not in frame or frame['body'].shape != (3,) or not np.isfinite(frame['body']).all():
                        raise ValueError('逐帧近距过滤缺少机器人 body 位置；提供同轮 --evidence、重新采集带 body 的扫描，或显式 --near-reference lidar-origin')
    if args.dufo:
        try:
            from dufomap import dufomap
        except ImportError as error:
            raise ValueError('服务器缺少 DUFOMap；在 .deps/venv 安装 dufomap==1.1.1 后再试，不会自动跳过清理') from error


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


def downsample(points, size):
    if not size:
        return points
    _, index, counts = np.unique(np.floor(points / size).astype(np.int64), axis=0,
                                 return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(index, weights=points[:, axis]) / counts
                            for axis in range(3)])


def prepare(raw, transform, args, out):
    """Return floor-aligned filtered points and a reproducible provenance report."""
    validate(args)
    check_sources(args)
    rotation, translation = transform[:3, :3], transform[:3, 3]
    params = {key: (str(value.resolve()) if isinstance(value, Path) else value)
              for key, value in vars(args).items() if key not in ('out', 'pcd', 'start', 'auto_start')}
    report = {'source_pcd': str(args.pcd.resolve()), 'source_sha256': digest(args.pcd),
              'parameters': params, 'input_points': len(raw), 'frames_total': None,
              'frames_kept': None, 'dufomap_used': args.dufo, 'frame_sources': []}
    report['software_versions'] = {}
    for package in ('numpy', 'scipy', 'scikit-image', 'fast-simplification', 'dufomap'):
        try:
            report['software_versions'][package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            pass
    report['world_from_camera_init'] = transform.tolist()
    paths = []
    positions = None
    if args.scans:
        paths = scan_paths(args.scans)
        if len(paths) < 2:
            raise ValueError('--scans 至少需要两个时间戳命名的配对 NPZ 文件')
        stamps, keep_frames, positions = frame_context(args, paths)
        if args.evidence:
            report['evidence_sha256'] = digest(args.evidence)
            report['evidence_timing_check'] = 'relative time offsets; operator must provide same-run evidence'
        if args.dufo:
            print(f'[DUFOMap] 全量 {len(paths)} 帧建立动态判别；随后应用输出帧与点过滤', flush=True)
            iterator = clean(read_scans(paths), d_s=args.dufo_d_s, d_p=args.dufo_d_p)
        else:
            iterator = ((p, np.ones(len(p), bool)) for p, _ in read_scans(paths))
        report['frames_total'] = len(paths)
        report['frame_sources'] = [{'stamp_ns': int(stamp), 'sha256': digest(path),
                                    'selected': bool(selected)}
                                   for stamp, path, selected in zip(stamps, paths, keep_frames)]
        chunks = []
        frame_counts = []
        report['scan_input_points'] = 0
        report['static_points_all_frames'] = 0
        for i, (points, static) in enumerate(iterator):
            report['scan_input_points'] += len(points)
            report['static_points_all_frames'] += int(np.count_nonzero(static))
            if not keep_frames[i]:
                continue
            points = points[static]
            reference = None
            if args.near_radius:
                with np.load(paths[i], allow_pickle=False) as frame:
                    if args.near_reference == 'lidar-origin':
                        reference = frame['pose'][:3]
                    elif positions is not None:
                        reference = positions[i]
                    elif 'body' in frame:
                        reference = frame['body']
                    else:
                        raise ValueError('逐帧近距过滤缺少机器人 body 位置；提供 --evidence、重新采集带 body 的扫描，或显式 --near-reference lidar-origin')
                if np.asarray(reference).shape != (3,) or not np.isfinite(reference).all():
                    raise ValueError('逐帧近距参考位置无效')
            filtered = filter_points(points, reference, rotation, translation, args)
            chunks.append(filtered)
            frame_counts.append(len(filtered))
        if len(chunks) != int(keep_frames.sum()):
            raise ValueError('DUFOMap 返回帧数不符合输入')
        points = np.concatenate(chunks)
        report['frames_kept'] = len(chunks)
        report['frame_output_points'] = frame_counts
    else:
        points = filter_points(raw, None, rotation, translation, args)
    report['points_before_downsample'] = len(points)
    points = downsample(points, args.voxel)
    report['output_points'] = len(points)
    if len(points) < 100:
        raise ValueError('过滤后点数不足 100；减小近距/高度过滤或降采样间距，或检查原始采集')
    # Preserve a useful filtered PCD in original camera_init coordinates.
    write_pcd(out / 'filtered.pcd', (points - translation) @ rotation)
    (out / 'filter_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f'[过滤] {report["input_points"]} 原始 PCD 点；筛选后 {report["points_before_downsample"]}，降采样后 {len(points)}', flush=True)
    return points, report


def filter_points(points, reference, rotation, translation, args):
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('扫描必须包含有限的 Nx3 世界坐标点')
    aligned = points @ rotation.T + translation
    ground = np.abs(aligned[:, 2]) < args.floor_band
    mask = ((aligned[:, 2] >= args.height_min) & (aligned[:, 2] <= args.height_max)
            & (np.linalg.norm(aligned[:, :2], axis=1) < args.radius))
    if args.origin_clear_radius:
        far = np.linalg.norm(aligned[:, :2], axis=1) > args.origin_clear_radius
        mask &= far | (ground & args.keep_near_ground)
    if args.near_radius:
        far = np.linalg.norm(points[:, :2] - np.asarray(reference)[:2], axis=1) > args.near_radius
        mask &= far | (ground & args.keep_near_ground)
    return aligned[mask]
