#!/usr/bin/env python3
"""Offline whole-polyline transport clearance gate; never connects to ROS.

Blocked means occupied OR unknown. Cell half-diagonal and interpolation error
are subtracted, yielding a conservative distance bound to the union of cells.
This is a static map check, not a dynamic-obstacle or tracking guarantee.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml

REFERENCE_ROUTE = [[8, 1.082], [6.6, 3], [4.8, 3], [2.35, .6]]
TRANSPORT_RADIUS = .45
REQUIRED_RESERVE = .10
SPACING = .01


def check(points, map_yaml):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Expected at least two finite XY path points')
    map_yaml = Path(map_yaml)
    metadata = yaml.safe_load(map_yaml.read_text())
    resolution = float(metadata['resolution'])
    if resolution <= 0 or metadata.get('negate', 0) or metadata['origin'][2] != 0 or metadata.get('mode') != 'trinary':
        raise ValueError('Expected positive resolution, world-axis-aligned trinary map, negate=0')
    image_path = map_yaml.parent / metadata['image']
    image = np.asarray(Image.open(image_path).convert('L'))[::-1]
    # Only map-server free cells count as known free, including threshold semantics.
    occupancy = (255 - image.astype(float)) / 255
    free = occupancy < metadata['free_thresh']
    rows, cols = np.nonzero(~free)
    origin = np.asarray(metadata['origin'][:2], dtype=float)
    blocked = origin + np.column_stack((cols+.5, rows+.5))*resolution
    samples = []
    max_gap = 0.
    for start, end in zip(points, points[1:]):
        length = float(np.linalg.norm(end-start))
        count = max(1, math.ceil(length/SPACING))
        max_gap = max(max_gap, length/count)
        samples.extend(start + np.linspace(0, 1, count+1)[:, None]*(end-start))
    samples = np.asarray(samples)
    if len(blocked):
        lower = cKDTree(blocked).query(samples)[0] - resolution*math.sqrt(2)/2
    else:
        lower = np.full(len(samples), np.inf)
    # Outside-map space is also blocked, even if a future map omits unknown borders.
    high = origin + np.array([image.shape[1], image.shape[0]])*resolution
    boundary = np.minimum(samples-origin, high-samples).min(axis=1)
    lower = np.minimum(lower, boundary)
    worst = int(np.argmin(lower))
    # Distance to any closed obstacle set is 1-Lipschitz. Each segment point is
    # at most half a sample gap away: this bound covers the entire polyline.
    bound = float(lower[worst] - max_gap/2)
    margin = bound-TRANSPORT_RADIUS
    return {
        'passed': bool(margin >= REQUIRED_RESERVE),
        'scope': 'offline static whole-polyline check only; no navigation execution',
        'map_yaml': map_yaml.name,
        'map_image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        'map_yaml_sha256': hashlib.sha256(map_yaml.read_bytes()).hexdigest(),
        'transport_radius_m': TRANSPORT_RADIUS,
        'required_transport_reserve_m': REQUIRED_RESERVE,
        'required_center_clearance_m': TRANSPORT_RADIUS+REQUIRED_RESERVE,
        'sample_count': len(samples),
        'maximum_sample_gap_m': max_gap,
        'interpolation_bound_subtracted_m': max_gap/2,
        'cell_half_diagonal_subtracted_m': resolution*math.sqrt(2)/2,
        'minimum_center_clearance_lower_bound_m': bound,
        'minimum_transport_margin_lower_bound_m': margin,
        'reserve_surplus_lower_bound_m': margin-REQUIRED_RESERVE,
        'worst_sample_xy': samples[worst].tolist(),
        'unknown_and_outside_blocked': True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--alignment-report', type=Path, help='JSON with planner.points_xy from A')
    source.add_argument('--reference-north-detour', action='store_true', help='Offline feasibility illustration, NOT a Nav2 result')
    parser.add_argument('--map', type=Path, required=True, help='Full planning map YAML; do not use laser slice')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.alignment_report:
            data = json.loads(args.alignment_report.read_text())
            planner = data['planner']
            if planner.get('status') != 4 or planner.get('error_code') != 0:
                raise ValueError('Input does not contain a successful planner result')
            points = planner['points_xy']
        else:
            points = REFERENCE_ROUTE
        result = check(points, args.map)
        result['input_source'] = str(args.alignment_report) if args.alignment_report else 'reference polyline, NOT Nav2 output'
        if args.alignment_report:
            result['input_sha256'] = hashlib.sha256(args.alignment_report.read_bytes()).hexdigest()
    except (ValueError, KeyError, OSError) as exc:
        result = {'passed': False, 'error': str(exc)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
