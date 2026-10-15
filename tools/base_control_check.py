#!/usr/bin/env python3
"""Rotate the course base ~0.04 rad through velocity_smoother, then restore yaw.

Requires successful navigation_readiness and no navigation/behavior goal active.
Checks /cmd_vel Twist output and fresh odometry feedback; does not certify Nav2.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('base_control_observer', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    odom, outputs = [], []
    subscriptions = [node.create_subscription(Odometry, '/odom', odom.append, qos_profile_sensor_data),
                     node.create_subscription(Twist, '/cmd_vel', outputs.append, 10)]
    pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    report = {'status': 'failed', 'scope': 'small physical base rotation and velocity chain; no navigation goal',
              'errors': []}

    def yaw(msg):
        q = msg.pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def delta(a, b):
        return math.atan2(math.sin(a-b), math.cos(a-b))

    def step(speed):
        msg = Twist()
        msg.angular.z = speed
        pub.publish(msg)
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        deadline = time.monotonic() + 10
        while not odom or not pub.get_subscription_count():
            if time.monotonic() > deadline:
                raise RuntimeError('Missing odometry or velocity_smoother input')
            step(0.0)
        baseline = yaw(odom[-1])
        position = odom[-1].pose.pose.position
        start_xy = [position.x, position.y]
        report.update(initial_yaw=baseline, initial_xy=start_xy)
        deadline = time.monotonic() + 12
        while delta(yaw(odom[-1]), baseline) < 0.04:
            if abs(delta(yaw(odom[-1]), baseline)) > 0.2:
                raise RuntimeError('Unexpected rotation >0.2 rad; stop before further motion')
            if time.monotonic() > deadline:
                raise RuntimeError('Base failed to rotate +0.04 rad within 12 wall seconds')
            step(0.08)
        report['positive_yaw_delta'] = delta(yaw(odom[-1]), baseline)
        deadline = time.monotonic() + 12
        while delta(yaw(odom[-1]), baseline) > 0.005:
            if abs(delta(yaw(odom[-1]), baseline)) > 0.2:
                raise RuntimeError('Unexpected restoration rotation >0.2 rad')
            if time.monotonic() > deadline:
                raise RuntimeError('Base failed to restore initial yaw')
            step(-0.06)
        for _ in range(20):
            step(0.0)
        report.update(final_yaw_error=delta(yaw(odom[-1]), baseline), odom_messages=len(odom),
                      cmd_vel_messages=len(outputs),
                      positive_output=any(m.angular.z > 0.01 for m in outputs),
                      negative_output=any(m.angular.z < -0.01 for m in outputs),
                      final_command_z=outputs[-1].angular.z if outputs else None)
        if not report['positive_output'] or not report['negative_output']:
            raise RuntimeError('Smoother Twist output missing either direction')
        if abs(report['final_yaw_error']) > 0.02 or abs(report['final_command_z']) > 0.001:
            raise RuntimeError('Base has not stopped/restored within tolerance')
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        for _ in range(5):
            step(0.0)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
