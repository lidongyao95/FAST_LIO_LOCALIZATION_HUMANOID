"""Continuous scan surface and triangle-preserving collision geometry (no box voxels)."""
import numpy as np


def reconstruct(points, radius, pitch=0.10, max_faces=4000, ground_cut=.16, height=2.5):
    from scipy.ndimage import gaussian_filter
    from skimage.measure import marching_cubes
    import fast_simplification

    above = points[(points[:, 2] >= ground_cut) & (points[:, 2] <= height)
                   & (np.linalg.norm(points[:, :2], axis=1) < radius)]
    if len(above) < 100:
        raise ValueError("Insufficient above-floor points for plaster reconstruction")
    cells = np.floor(above / pitch).astype(np.int64)
    lower = cells.min(axis=0) - 4
    cells -= lower
    shape = cells.max(axis=0) + 5
    if np.prod(shape, dtype=np.float64) > 24_000_000:
        raise ValueError("Plaster reconstruction volume too large")
    field = np.zeros(shape, np.float32)
    field[tuple(cells.T)] = 1
    field = gaussian_filter(field, .85)
    if field.max() <= .14:
        raise ValueError("Scan too sparse for a supported plaster surface")
    vertices, faces, _, _ = marching_cubes(field, .14, spacing=(pitch,) * 3,
                                          allow_degenerate=False)
    vertices += (lower + .5) * pitch
    faces = faces[:, ::-1].copy()
    if len(faces) > max_faces:
        vertices, faces = fast_simplification.simplify(
            vertices.astype(np.float64), faces, target_count=max_faces,
            agg=4., verbose=False)
    triangles = vertices[faces]
    area2 = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1)
    faces = faces[area2 > 1e-8]
    if not len(faces):
        raise ValueError("Reconstruction contains no valid surface triangles")
    return vertices, faces


def write_obj(path, vertices, faces):
    with path.open('w') as stream:
        for p in vertices:
            stream.write('v ' + ' '.join(f'{x:.7f}' for x in p) + '\n')
        for face in faces:
            stream.write('f ' + ' '.join(str(int(i) + 1) for i in face) + '\n')


def sample_surface(vertices, faces, spacing=.07):
    chunks = []
    for tri in vertices[faces]:
        length = max(np.linalg.norm(tri[a] - tri[b]) for a, b in ((0, 1), (1, 2), (2, 0)))
        n = max(1, int(np.ceil(length / spacing)))
        u, v = np.meshgrid(np.arange(n + 1) / n, np.arange(n + 1) / n)
        mask = u + v <= 1 + 1e-9
        chunks.append(tri[0] + u[mask, None] * (tri[1] - tri[0])
                      + v[mask, None] * (tri[2] - tri[0]))
    return np.concatenate(chunks)
