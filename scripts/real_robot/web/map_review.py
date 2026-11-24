#!/usr/bin/env python3
"""Read-only exported-map inspection, without ROS or localization."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import yaml


def load_map(path):
    path = Path(path).expanduser().resolve()
    if path.is_dir():
        path /= 'map.yaml'
    config = yaml.safe_load(path.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('地图检查目前支持 trinary 栅格地图')
    resolution = float(config['resolution'])
    origin = [float(v) for v in config['origin']]
    free, occupied = float(config['free_thresh']), float(config['occupied_thresh'])
    if (len(origin) != 3 or not all(math.isfinite(v) for v in [resolution, *origin, free, occupied])
            or resolution <= 0 or not 0 <= free < occupied <= 1 or config.get('negate', 0) not in (0, 1)):
        raise ValueError('地图分辨率、原点或占用阈值无效')
    image = cv2.imread(str(path.parent / config['image']), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError('地图图像无法读取')
    probability = image.astype(float) / 255
    if not config.get('negate', 0):
        probability = 1 - probability
    counts = {'free': int((probability < free).sum()),
              'occupied': int((probability > occupied).sum())}
    counts['unknown'] = int(image.size) - sum(counts.values())
    ok, encoded = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('地图图像编码失败')
    return dict(name=path.parent.name, width=int(image.shape[1]), height=int(image.shape[0]),
                resolution=resolution, origin=origin, free_thresh=free,
                occupied_thresh=occupied, negate=config.get('negate', 0), counts=counts), encoded.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map')
    parser.add_argument('--port', type=int, default=18081)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('端口需为 1–65535')
    try:
        metadata, png = load_map(args.map)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    map_path = Path(args.map).expanduser().resolve()
    map_dir = map_path if map_path.is_dir() else map_path.parent
    metadata['surface_available'] = (map_dir / 'surface.ply').is_file()
    metadata['plaster_available'] = (map_dir / 'plaster.png').is_file()
    page = Path(__file__).with_name('map_review.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.headers.get('Host') not in (f'127.0.0.1:{args.port}', f'localhost:{args.port}'):
                self.send_error(403)
                return
            route = urlsplit(self.path).path
            routes = {'/': ('text/html; charset=utf-8', page),
                      '/api/map': ('application/json', json.dumps(metadata).encode()),
                      '/map.png': ('image/png', png)}
            assets = {
                '/plaster_viewer.js': 'plaster_viewer.js',
                '/assets/three/three.module.js': 'assets/three/three.module.js',
                '/assets/three/OrbitControls.js': 'assets/three/OrbitControls.js',
                '/assets/three/PLYLoader.js': 'assets/three/PLYLoader.js',
            }
            if route in assets:
                routes[route] = ('text/javascript; charset=utf-8', Path(__file__).with_name('map_review.html').parent.joinpath(assets[route]).read_bytes())
            if route == '/surface.ply' and (map_dir / 'surface.ply').is_file():
                routes[route] = ('application/octet-stream', (map_dir / 'surface.ply').read_bytes())
            if route == '/plaster.png' and (map_dir / 'plaster.png').is_file():
                routes[route] = ('image/png', (map_dir / 'plaster.png').read_bytes())
            if route not in routes:
                self.send_error(404)
                return
            kind, body = routes[route]
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

    with ThreadingHTTPServer(('127.0.0.1', args.port), Handler) as server:
        print(f'地图检查就绪：http://127.0.0.1:{args.port}/，Ctrl+C 结束', flush=True)
        print('仅查看已导出地图，不启动 ROS、定位或规划。', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
