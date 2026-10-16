#!/usr/bin/env python3
"""Offline map/world-aligned transport trace gate, never starts ROS.

Requires the same map/world identity verified by A. Tracking error is distance
to the accepted reference polyline; it does not match time or progress. A must
record every actual replan and assess speed/acceleration with simulation stamps.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from check_path_clearance import check

TRACKING_LIMIT = .05


def tracking_errors(points, reference):
    points, reference = np.asarray(points, float), np.asarray(reference, float)
    for value in (points, reference):
        if value.ndim != 2 or value.shape[1] != 2 or len(value) < 2 or not np.isfinite(value).all():
            raise ValueError('Expected at least two finite XY samples')
    starts, vectors = reference[:-1], np.diff(reference, axis=0)
    lengths = np.sum(vectors*vectors, axis=1)
    errors = []
    for point in points:
        fractions = np.sum((point-starts)*vectors, axis=1) / np.maximum(lengths, 1e-24)
        closest = starts + np.clip(fractions, 0, 1)[:, None]*vectors
        errors.append(float(np.linalg.norm(closest-point, axis=1).min()))
    return np.asarray(errors)


def reference_points(report):
    points = report['planner']['points_xy']
    # Smac grid paths can begin a cell away from the measured start. Include
    # the captured map-base connector; never discard initial trace samples.
    start = report.get('initial_localization', {}).get('map_base_xy')
    return [start] + points if start is not None else points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-report', type=Path, required=True)
    parser.add_argument('--reference-report', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    try:
        motion = json.loads(args.motion_report.read_text())
        plan = json.loads(args.reference_report.read_text())
        points = [sample['world_xy'] for sample in motion['trace']]
        reference = reference_points(plan)
        errors = tracking_errors(points, reference)
        clearance = check(points, args.map)
        planned_clearance = check(reference, args.map)
        result = {
            'passed': bool(motion.get('status') == 'passed' and plan.get('status') == 'passed'
                           and plan['planner'].get('status') == 4 and plan['planner'].get('error_code') == 0
                           and planned_clearance['passed'] and clearance['passed'] and errors.max() <= TRACKING_LIMIT),
            'scope': 'offline saved trajectory geometry/tracking only; speed/acceleration not certified',
            'motion_status': motion.get('status'), 'tracking_limit_m': TRACKING_LIMIT,
            'maximum_tracking_error_m': float(errors.max()),
            'worst_tracking_sample_index': int(errors.argmax()),
            'sample_count': len(points), 'actual_trajectory_clearance': clearance,
            'reference_clearance': planned_clearance,
            'motion_sha256': hashlib.sha256(args.motion_report.read_bytes()).hexdigest(),
            'reference_sha256': hashlib.sha256(args.reference_report.read_bytes()).hexdigest(),
            'limits': 'Polyline interpolation is not proof of unobserved physical motion. Capture actual replan history and sim-stamped velocity/acceleration for runtime acceptance.',
        }
    except (ValueError, KeyError, OSError) as exc:
        result = {'passed': False, 'error': str(exc)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
