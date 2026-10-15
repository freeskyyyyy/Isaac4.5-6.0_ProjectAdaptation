#!/usr/bin/env python3
"""Tiny bounded yaw pulse with complete odometry and wheel feedback trace.

Diagnostic only, no navigation goals. Always publishes stop in finally.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--speed', type=float, default=0.005)
    p.add_argument('--duration', type=float, default=0.3)
    p.add_argument('--axis', choices=('yaw', 'x', 'y'), default='yaw')
    a = p.parse_args()
    if abs(a.speed) > 0.01 or not 0 < a.duration <= 0.5:
        p.error('Diagnostic bound: speed <= 0.01 rad/s; duration <= 0.5 wall seconds')
    rclpy.init()
    n = rclpy.create_node('base_pulse_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    trace, wheels = [], []
    start = time.monotonic()

    def receive(m):
        q = m.pose.pose.orientation
        trace.append({'wall': time.monotonic()-start,
            'stamp': m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            'q_xyzw': [q.x, q.y, q.z, q.w],
            'yaw': math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            'xy': [m.pose.pose.position.x,m.pose.pose.position.y],
            'omega_z': m.twist.twist.angular.z})

    def joints(m):
        selected = {name: vel for name, vel in zip(m.name, m.velocity) if 'wheel_joint' in name}
        if selected:
            wheels.append({'wall': time.monotonic()-start, 'velocities': selected})

    subs = [n.create_subscription(Odometry, '/odom', receive, qos_profile_sensor_data),
            n.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)]
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    report = {'scope': 'limited raw Twist pulse diagnosis only; not base acceptance', 'status': 'failed', 'errors': []}
    baseline = None

    def step(speed):
        msg = Twist()
        if a.axis == 'yaw': msg.angular.z = speed
        elif a.axis == 'x': msg.linear.x = speed
        else: msg.linear.y = speed
        pub.publish(msg)
        end = time.monotonic()+0.03
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.005)

    try:
        end = time.monotonic()+2
        while time.monotonic() < end:
            step(0.0)
        if len(trace) < 20 or not pub.get_subscription_count():
            raise RuntimeError('No fresh odometry or Twist consumer')
        others = [x for x in n.get_publishers_info_by_topic('/cmd_vel') if x.node_name != n.get_name()]
        if others:
            raise RuntimeError('Other velocity publishers active; stop partial navigation stack first')
        baseline = trace[-1]['yaw']
        report.update(baseline=baseline, pulse_start_wall=time.monotonic()-start,
                      speed=a.speed, axis=a.axis, duration_wall=a.duration,
                      baseline_xy=trace[-1]['xy'])
        end = time.monotonic()+a.duration
        while time.monotonic() < end:
            step(a.speed)
            d = math.atan2(math.sin(trace[-1]['yaw']-baseline),math.cos(trace[-1]['yaw']-baseline))
            if abs(d) > 0.2:
                raise RuntimeError('Unexpected yaw change >0.2 rad; stopped pulse')
        report['pulse_end_wall'] = time.monotonic()-start
        end = time.monotonic()+2
        while time.monotonic() < end:
            step(0.0)
        report['final_yaw_delta'] = math.atan2(math.sin(trace[-1]['yaw']-baseline),math.cos(trace[-1]['yaw']-baseline))
        report['final_xy_delta'] = [v-u for u,v in zip(report['baseline_xy'],trace[-1]['xy'])]
        report['status'] = 'observed'
    except Exception as e:
        report['errors'].append(str(e))
    finally:
        for _ in range(10): step(0.0)
        report.update(odom_trace=trace, wheel_trace=wheels)
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('odom_trace','wheel_trace')},indent=2))
        n.destroy_node();rclpy.shutdown()
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
