#!/usr/bin/env python3
"""Observe real simulator streams; never publish commands or synthetic inputs."""
import argparse
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from course_bringup.readiness import Readiness, TOPICS


def stamp_seconds(msg):
    stamp = msg.clock if hasattr(msg, 'clock') else msg.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class Observer(Readiness):
    def __init__(self, image_reliability):
        self.stamps = {topic: [] for topic in TOPICS}
        self.arrivals = {topic: [] for topic in TOPICS}
        super().__init__(image_reliability=image_reliability)

    def receive(self, topic, msg):
        super().receive(topic, msg)
        self.stamps[topic].append(stamp_seconds(msg))
        self.arrivals[topic].append(time.monotonic())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--image-reliability', choices=('best_effort', 'reliable'), default='reliable',
                        help='Diagnostic subscription reliability; must be compatible with the actual publisher')
    opts = parser.parse_args()
    if opts.duration < 10:
        parser.error('Use at least 10 wall seconds to check sustained output')
    rclpy.init()
    node = Observer(opts.image_reliability)
    try:
        started = time.monotonic()
        until = started + opts.duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        errors = node.problems()
        report = node.report(errors)
        report.update(scope='real course simulator sensor streams only; no motion or task success',
                      observation_wall_seconds=opts.duration, image_reliability=opts.image_reliability,
                      rates={}, image_pairing={})
        for topic, stamps in node.stamps.items():
            unique = sorted(set(stamps))
            span = unique[-1] - unique[0] if len(unique) > 1 else 0
            rate = (len(unique) - 1) / span if span > 0 else None
            report['rates'][topic] = {'messages': len(stamps), 'unique_stamps': len(unique),
                                     'sim_time_span': span, 'hz_sim_time': rate}
            if topic.startswith('/arm_camera/'):
                arrivals = node.arrivals[topic]
                wall_span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
                max_gap = max((b - a for a, b in zip(arrivals, arrivals[1:])), default=None)
                report['rates'][topic].update(observed_wall_span=wall_span,
                    max_wall_gap=max_gap, startup_delay=arrivals[0] - started if arrivals else None)
                if wall_span < opts.duration * 0.75 or max_gap is None or max_gap > 1:
                    errors.append(f'Camera did not cover observation window without >1s gaps: {topic}')
                if span < 1 or len(unique) < 10:
                    errors.append(f'Insufficient sustained camera output: {topic}')
                elif rate is None or not 20 <= rate <= 65:
                    errors.append(f'Unexpected camera rate: {topic}: {rate}')
                if any(b < a for a, b in zip(stamps, stamps[1:])):
                    errors.append(f'Camera timestamps moved backwards: {topic}')
        rgb_times = node.stamps['/arm_camera/rgb']
        for topic in ('/arm_camera/depth', '/arm_camera/camera_info'):
            others = np.array(node.stamps[topic])
            differences = [float(np.min(np.abs(others - value))) for value in rgb_times] if others.size else []
            fraction = sum(delta <= 0.05 for delta in differences) / len(rgb_times) if rgb_times else 0
            report['image_pairing'][topic] = {'fraction_within_50ms': fraction,
                                             'max_nearest_delta_seconds': max(differences, default=None)}
            if fraction < 0.9:
                errors.append(f'RGB pairing below 90% within 50ms: {topic}')
        bridge = CvBridge()
        opts.report.parent.mkdir(parents=True, exist_ok=True)
        rgb = node.messages.get('/arm_camera/rgb')
        if rgb is not None:
            image = bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            image_path = opts.report.with_name('arm-camera-rgb.png')
            if not cv2.imwrite(str(image_path), image):
                errors.append('Could not save observed RGB image')
            report['rgb_image'] = str(image_path)
        depth = node.messages.get('/arm_camera/depth')
        if depth is not None:
            image = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            meters = image.astype(np.float32) * (0.001 if depth.encoding == '16UC1' else 1.0)
            valid = meters[np.isfinite(meters) & (meters > 0)]
            report['depth'] = {'encoding': depth.encoding, 'unit_after_conversion': 'meters',
                               'finite_positive_pixels': int(valid.size),
                               'total_pixels': int(meters.size),
                               'percentiles_m': np.percentile(valid, [1, 50, 99]).tolist() if valid.size else []}
            if not valid.size:
                errors.append('Depth has no finite positive values')
            try:
                capture_time = rclpy.time.Time.from_msg(depth.header.stamp)
                tf_deadline = time.monotonic() + 2
                # The dynamic TF graph may publish just after this image. Wait
                # for that image's timestamp rather than silently using latest.
                while not node.tf.can_transform('base_link_arm', depth.header.frame_id, capture_time):
                    if time.monotonic() >= tf_deadline:
                        break
                    rclpy.spin_once(node, timeout_sec=0.05)
                tf = node.tf.lookup_transform('base_link_arm', depth.header.frame_id,
                                              capture_time)
                report['depth_capture_tf'] = {'parent': tf.header.frame_id, 'child': tf.child_frame_id,
                    'translation': [getattr(tf.transform.translation, axis) for axis in 'xyz'],
                    'rotation_xyzw': [getattr(tf.transform.rotation, axis) for axis in 'xyzw'],
                    'stamp': stamp_seconds(tf)}
            except Exception as exc:
                errors.append(f'TF at depth capture timestamp unavailable: {exc}')
        report['status'] = 'passed' if not errors else 'failed'
        opts.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0 if not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
