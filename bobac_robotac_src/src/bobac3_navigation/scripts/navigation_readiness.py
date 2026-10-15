#!/usr/bin/env python3
"""Read-only Nav2 readiness and optional result lookup for an existing goal."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


LIFECYCLE_NODES = [
    'map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
]


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NavigationReadiness(Node):
    def __init__(self, scan_topic):
        super().__init__('navigation_readiness',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scan_topic = scan_topic
        self.messages = {}
        self.received = {}
        self.counts = {'/clock': 0, '/odom': 0, scan_topic: 0}
        self.clock_advanced = False
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.subs = [self.create_subscription(
            kind, topic, lambda msg, key=topic: self.receive(key, msg), qos_profile_sensor_data)
            for topic, kind in [('/clock', Clock), ('/odom', Odometry), (scan_topic, LaserScan)]]
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.states = {}
        self.sim_times = {}
        self.checked_at = {}
        self.service_clients = {}
        self.pending = {}
        for name in LIFECYCLE_NODES:
            self.service_clients[(name, 'state')] = self.create_client(GetState, f'/{name}/get_state')
            self.service_clients[(name, 'sim_time')] = self.create_client(
                GetParameters, f'/{name}/get_parameters')

    def receive(self, topic, msg):
        if topic == '/clock':
            old = self.messages.get(topic)
            if old is not None:
                if seconds(msg.clock) < seconds(old.clock):
                    self.clock_advanced = False
                elif seconds(msg.clock) > seconds(old.clock):
                    self.clock_advanced = True
        self.messages[topic] = msg
        self.received[topic] = time.monotonic()
        self.counts[topic] += 1

    def poll_services(self):
        for key, client in self.service_clients.items():
            pending = self.pending.get(key)
            if pending is not None:
                if not pending.done():
                    continue
                try:
                    response = pending.result()
                    if key[1] == 'state':
                        self.states[key[0]] = response.current_state.label
                    else:
                        self.sim_times[key[0]] = bool(
                            response.values and response.values[0].type == 1
                            and response.values[0].bool_value)
                    self.checked_at[key] = time.monotonic()
                except Exception as exc:
                    self.get_logger().warning(f'{key}: {exc}')
                del self.pending[key]
            if client.service_is_ready():
                if key[1] == 'state':
                    request = GetState.Request()
                else:
                    request = GetParameters.Request()
                    request.names = ['use_sim_time']
                self.pending[key] = client.call_async(request)
            else:
                self.states.pop(key[0], None)
                self.sim_times.pop(key[0], None)

    def problems(self):
        errors = []
        if os.environ.get('ROS_DOMAIN_ID') != '44':
            errors.append('Expected ROS_DOMAIN_ID=44')
        if rclpy.get_rmw_implementation_identifier() != 'rmw_cyclonedds_cpp':
            errors.append('Expected Cyclone DDS (RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)')
        now = time.monotonic()
        for topic in self.counts:
            if topic not in self.messages:
                errors.append(f'No data: {topic}; check Play, domain and QoS')
            elif now - self.received[topic] > 5:
                errors.append(f'Stale data (>5 wall seconds): {topic}')
        if not self.clock_advanced:
            errors.append('/clock has not advanced; check simulation Play/reset')
        clock = self.messages.get('/clock')
        for topic in ['/odom', self.scan_topic]:
            msg = self.messages.get(topic)
            if msg is not None and clock is not None:
                age = seconds(clock.clock) - seconds(msg.header.stamp)
                if age > 5 or age < -1:
                    errors.append(f'{topic} timestamp differs from /clock by {age:.3f}s')
        odom = self.messages.get('/odom')
        if odom is not None and (odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link'):
            errors.append(f'Odometry frames: {odom.header.frame_id} -> {odom.child_frame_id}; '
                          'expected odom -> base_link')
        scan = self.messages.get(self.scan_topic)
        if scan is not None:
            if not scan.ranges or scan.angle_increment <= 0 or scan.range_max <= scan.range_min:
                errors.append('Invalid/empty LaserScan geometry')
            if not any(math.isfinite(value) and scan.range_min <= value <= scan.range_max
                       for value in scan.ranges):
                errors.append('LaserScan has no finite in-range returns for AMCL')
            if scan.header.frame_id != 'fuse_lidar_link':
                errors.append(f'LaserScan frame is {scan.header.frame_id}; expected fuse_lidar_link')
        for parent, child in [('map', 'odom'), ('odom', 'base_link'),
                              ('base_link', 'fuse_lidar_link'), ('map', 'base_link')]:
            if not self.tf.can_transform(parent, child, rclpy.time.Time()):
                errors.append(f'Missing TF: {parent} <- {child}')
            elif clock is not None:
                transform = self.tf.lookup_transform(parent, child, rclpy.time.Time())
                stamp = seconds(transform.header.stamp)
                # Static TF stamps are zero; AMCL stamps may include transform_tolerance.
                if stamp and not -6 <= seconds(clock.clock) - stamp <= 5:
                    errors.append(f'Stale/future TF: {parent} <- {child}')
        for name in LIFECYCLE_NODES:
            if (self.states.get(name) != 'active' or
                    now - self.checked_at.get((name, 'state'), 0) > 3):
                errors.append(f'Lifecycle /{name}: {self.states.get(name, "service unavailable/pending")}')
            if (self.sim_times.get(name) is not True or
                    now - self.checked_at.get((name, 'sim_time'), 0) > 3):
                errors.append(f'/{name} use_sim_time is false or unavailable')
        if not self.action.server_is_ready():
            errors.append('/navigate_to_pose action server unavailable')
        publishers = self.get_publishers_info_by_topic('/cmd_vel')
        subscribers = self.get_subscriptions_info_by_topic('/cmd_vel')
        twist = 'geometry_msgs/msg/Twist'
        if not publishers or any(endpoint.topic_type != twist for endpoint in publishers):
            errors.append('/cmd_vel must have a Twist publisher from velocity_smoother')
        elif not any(endpoint.node_name == 'velocity_smoother' for endpoint in publishers):
            errors.append('velocity_smoother is not publishing /cmd_vel')
        if not any(endpoint.topic_type == twist for endpoint in subscribers):
            errors.append('No Twist subscriber on /cmd_vel; check simulator chassis graph')
        return errors

    def report(self, errors):
        return {'status': 'passed' if not errors else 'failed', 'errors': errors,
                'messages_received': self.counts, 'lifecycle': self.states,
                'use_sim_time': self.sim_times,
                'domain': os.environ.get('ROS_DOMAIN_ID'),
                'rmw': rclpy.get_rmw_implementation_identifier(),
                'scan_topic': self.scan_topic}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--scan-topic', default='/laser_scan_fuse')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--goal-id', type=uuid.UUID,
                        help='Read the result of an EXISTING goal UUID; never sends a goal')
    opts, ros_args = parser.parse_known_args(args)
    if opts.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init(args=ros_args)
    node = NavigationReadiness(opts.scan_topic)
    deadline = time.monotonic() + opts.timeout
    errors = node.problems()
    report = None
    try:
        next_poll = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_poll:
                node.poll_services()
                next_poll = time.monotonic() + 0.5
            errors = node.problems()
            if not errors:
                break
        report = node.report(errors)
        if not errors and opts.goal_id:
            client = node.create_client(NavigateToPose.Impl.GetResultService,
                                        '/navigate_to_pose/_action/get_result')
            request = NavigateToPose.Impl.GetResultService.Request()
            request.goal_id.uuid = list(opts.goal_id.bytes)
            future = client.call_async(request)
            # A separate timeout allows a goal already in progress to finish.
            rclpy.spin_until_future_complete(node, future, timeout_sec=opts.timeout)
            if not future.done() or future.exception() is not None:
                errors.append('Existing navigation goal result timed out/failed')
            else:
                result = future.result()
                report['goal'] = {'id': str(opts.goal_id), 'status': result.status,
                                  'error_code': result.result.error_code,
                                  'error_msg': getattr(result.result, 'error_msg', '')}
                if result.status != 4 or result.result.error_code != 0:
                    errors.append('Existing goal did not succeed; inspect goal status/error_code')
            report['status'] = 'failed' if errors else 'passed'
        if opts.report:
            opts.report.parent.mkdir(parents=True, exist_ok=True)
            opts.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(1 if errors else 0)


if __name__ == '__main__':
    main()
