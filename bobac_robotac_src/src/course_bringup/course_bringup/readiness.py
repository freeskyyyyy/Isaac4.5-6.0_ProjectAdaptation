"""Wait for real simulator output before starting the ROS course stack."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_ros import Buffer, TransformListener

TOPICS = {
    '/clock': Clock, '/odom': Odometry, '/laser_scan_fuse': LaserScan,
    '/joint_states': JointState, '/arm/joint_states': JointState,
    '/arm_camera/rgb': Image, '/arm_camera/depth': Image,
    '/arm_camera/camera_info': CameraInfo,
}


class Readiness(Node):
    def __init__(self):
        super().__init__('course_readiness')
        self.messages = {}
        self.received = {}
        self.counts = dict.fromkeys(TOPICS, 0)
        self.clock_advanced = False
        self.last_clock = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subscriptions_list = [
            self.create_subscription(kind, topic, lambda msg, key=topic: self.receive(key, msg),
                                     qos_profile_sensor_data)
            for topic, kind in TOPICS.items()
        ]

    def receive(self, topic, msg):
        self.messages[topic] = msg
        self.received[topic] = time.monotonic()
        self.counts[topic] += 1
        if topic == '/clock':
            stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.last_clock is not None and stamp > self.last_clock:
                self.clock_advanced = True
            self.last_clock = stamp

    def problems(self):
        errors = []
        now = time.monotonic()
        for topic in TOPICS:
            if topic not in self.messages:
                errors.append(f'No data: {topic}; check Play, graph, ROS_DOMAIN_ID and QoS')
            elif now - self.received[topic] > 5:
                errors.append(f'Stale data (>5s): {topic}')
        if not self.clock_advanced:
            errors.append('/clock is not advancing; press Play')
        joints = self.messages.get('/arm/joint_states')
        if joints is not None:
            missing = set(f'joint_{i}' for i in range(1, 7)) - set(joints.name)
            if missing or len(joints.position) != len(joints.name):
                errors.append(f'Invalid arm JointState; missing joints: {sorted(missing)}')
        scan = self.messages.get('/laser_scan_fuse')
        if scan is not None and (not scan.ranges or scan.angle_increment <= 0):
            errors.append('Invalid or empty LaserScan')
        rgb = self.messages.get('/arm_camera/rgb')
        depth = self.messages.get('/arm_camera/depth')
        info = self.messages.get('/arm_camera/camera_info')
        for topic, img in (('/arm_camera/rgb', rgb), ('/arm_camera/depth', depth)):
            if img is not None and (not img.data or img.width == 0 or img.height == 0):
                errors.append(f'Empty image: {topic}')
        if depth is not None and depth.encoding not in ('32FC1', '16UC1'):
            errors.append(f'Unsupported depth encoding: {depth.encoding}')
        if info is not None:
            if info.k[0] <= 0 or info.k[4] <= 0:
                errors.append('CameraInfo focal lengths must be positive')
            for img in (rgb, depth):
                if img is not None and (img.width, img.height) != (info.width, info.height):
                    errors.append('RGB/Depth/CameraInfo resolution mismatch')
        for parent, child in [('odom', 'base_link'), ('base_link_arm', 'arm_Camera'),
                              ('base_link', 'fuse_lidar_link')]:
            if not self.tf.can_transform(parent, child, rclpy.time.Time()):
                errors.append(f'Missing TF: {parent} <- {child}')
        return errors

    def report(self, errors):
        return {'status': 'passed' if not errors else 'failed', 'errors': errors,
                'messages_received': self.counts,
                'topics': {topic: {
                    'message_type': TOPICS[topic].__name__,
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', None),
                    'encoding': getattr(msg, 'encoding', None),
                } for topic, msg in self.messages.items()}}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--report', type=Path)
    opts, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Readiness()
    deadline = time.monotonic() + opts.timeout
    errors = node.problems()
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            errors = node.problems()
            if not errors:
                break
        report = node.report(errors)
        if opts.report:
            opts.report.parent.mkdir(parents=True, exist_ok=True)
            opts.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if not errors else 1)
