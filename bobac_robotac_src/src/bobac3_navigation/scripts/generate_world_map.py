#!/usr/bin/env python3
"""Offline, deterministic axis-aligned USD cuboid projection; no ROS/Kit imports.

The evidence JSON describes metres in USD world, which is chosen as map's XY
coordinate system. The two outputs separate laser-height localization from
conservative full-height navigation obstacles. Neither is a runtime SLAM map.
"""
import argparse
import json
import math
from pathlib import Path


RESOLUTION = 0.05
ORIGIN = (-1.0, -10.5)
WIDTH, HEIGHT = 230, 320
# Only declare the rectangular interior bounded by the four enclosing walls
# known free. Unknown pixels outside it cannot become shortcuts around walls.
INTERIOR = (-0.05, -9.25, 9.45, 4.15)


def bounds(collision):
    points = collision['world_points_m']
    low = [min(point[axis] for point in points) for axis in range(3)]
    high = [max(point[axis] for point in points) for axis in range(3)]
    if not all(math.isfinite(number) for number in low + high):
        raise ValueError('Non-finite geometry: ' + collision['path'])
    # This fixture consists of cuboids rotated by 0/90/180 degrees. Refuse to
    # silently approximate future rotated/non-cuboid geometry as exact.
    if any(not any(abs(point[axis] - end) < 1e-7 for end in [low[axis], high[axis]])
           for point in points for axis in range(3)):
        raise ValueError('Expected axis-aligned cuboid vertices: ' + collision['path'])
    return low, high


def rasterize(evidence, scan_height=None):
    pixels = bytearray([205]) * (WIDTH * HEIGHT)
    for row in range(HEIGHT):
        y = ORIGIN[1] + (HEIGHT - row - .5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN[0] + (col + .5) * RESOLUTION
            if INTERIOR[0] < x < INTERIOR[2] and INTERIOR[1] < y < INTERIOR[3]:
                pixels[row * WIDTH + col] = 254
    for collision in evidence['collisions']:
        low, high = bounds(collision)
        if scan_height is not None and not low[2] <= scan_height <= high[2]:
            continue
        # Occupy every cell with positive-area overlap, allowing only sub-micron
        # authored float noise at exact grid boundaries. No inflation is baked
        # in: Nav2 applies its footprint/inflation once.
        col0 = math.floor((low[0] - ORIGIN[0]) / RESOLUTION + 1e-6)
        col1 = math.ceil((high[0] - ORIGIN[0]) / RESOLUTION - 1e-6)
        y0 = math.floor((low[1] - ORIGIN[1]) / RESOLUTION + 1e-6)
        y1 = math.ceil((high[1] - ORIGIN[1]) / RESOLUTION - 1e-6)
        if col0 < 0 or col1 > WIDTH or y0 < 0 or y1 > HEIGHT:
            raise ValueError('Geometry is outside the map: ' + collision['path'])
        for ycell in range(y0, y1):
            for col in range(col0, col1):
                pixels[(HEIGHT - 1 - ycell) * WIDTH + col] = 0
    return b'P5\n# USD world XY, candidate static map; runtime alignment pending\n230 320\n255\n' + pixels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    opts = parser.parse_args()
    evidence = json.loads(opts.geometry.read_text())
    if evidence['meters_per_unit'] != 1.0:
        parser.error('Expected geometry in metres')
    opts.output_dir.mkdir(parents=True, exist_ok=True)
    for name, height in [('world_scene', None), ('world_scene_lidar', evidence['scan_height_m'])]:
        (opts.output_dir / (name + '.pgm')).write_bytes(rasterize(evidence, height))
        (opts.output_dir / (name + '.yaml')).write_text(
            f'image: {name}.pgm\nmode: trinary\nresolution: 0.05\n'
            'origin: [-1.0, -10.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print('Generated world_scene (planning) and world_scene_lidar (localization); runtime pending')


if __name__ == '__main__':
    main()
