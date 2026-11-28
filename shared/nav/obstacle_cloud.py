"""Single-frame 3-D radius consensus, retaining PointCloud2 records and ray data."""
import copy

import numpy as np
from scipy.spatial import cKDTree


def records_and_xyz(cloud):
    fields = {field.name: field for field in cloud.fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('点云需要单值 FLOAT32/FLOAT64 的 x、y、z 字段')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > cloud.point_step:
            raise ValueError('点云字段超出 point_step')
        formats.append(('>' if cloud.is_bigendian else '<') + 'f' + str(size))
        offsets.append(field.offset)
    if cloud.height < 1 or cloud.width < 0 or cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('点云行布局无效')
    required = ((cloud.height - 1) * cloud.row_step + cloud.width * cloud.point_step) if cloud.width else 0
    if len(cloud.data) < required:
        raise ValueError('点云数据不完整')
    # Opaque records preserve fields and padding bytes not used for XYZ.
    # Copying a structured XYZ dtype may leave its unnamed gaps uninitialized.
    records = np.ndarray((cloud.height, cloud.width), dtype='V' + str(cloud.point_step), buffer=cloud.data,
                         strides=(cloud.row_step, cloud.point_step)).reshape(-1)
    xyz = np.column_stack([np.ndarray((cloud.height, cloud.width), dtype=kind,
        buffer=cloud.data, offset=offset, strides=(cloud.row_step, cloud.point_step)).reshape(-1)
        for kind, offset in zip(formats, offsets)]).astype(float)
    return records, xyz


def consensus_mask(xyz, radius, minimum):
    """Count the point itself; no accumulation across frames or cluster chaining."""
    if minimum == 1:
        return np.ones(len(xyz), dtype=bool)
    if len(xyz) < minimum:
        return np.zeros(len(xyz), dtype=bool)
    # Request only the kth distance: bounded output memory even in dense scans.
    distances, _ = cKDTree(xyz).query(xyz, k=[minimum], distance_upper_bound=radius + 1e-12)
    return distances[:, 0] <= radius + 1e-12


def select(cloud, records, mask):
    output = copy.copy(cloud)
    output.height, output.width = 1, int(np.count_nonzero(mask))
    output.row_step = output.width * cloud.point_step
    output.data = records[mask].tobytes()
    output.is_dense = True
    return output


def filter_cloud(cloud, settings, transform, min_height, max_height, max_range):
    records, xyz = records_and_xyz(cloud)
    finite = np.isfinite(xyz).all(axis=1)
    valid = xyz[finite]
    translation, quaternion = transform
    axis = np.asarray(quaternion[:3], dtype=float)
    cross = np.cross(axis, valid)
    world = valid + 2 * (quaternion[3] * cross + np.cross(axis, cross)) + translation
    eligible = ((world[:, 2] >= min_height) & (world[:, 2] <= max_height)
                & (np.linalg.norm(valid, axis=1) <= max_range))
    indices = np.flatnonzero(finite)[eligible]
    mask = np.zeros(len(xyz), dtype=bool)
    mask[indices] = consensus_mask(valid[eligible], settings['neighbor_radius'], settings['min_points'])
    # Raytracing retains finite raw returns, including rejected isolated points.
    return (select(cloud, records, mask), select(cloud, records, finite),
            dict(input_points=len(xyz), candidate_points=len(indices), obstacle_points=int(mask.sum())))
