#!/usr/bin/env python3
"""Target-container arm command/feedback and real gripper Action check.

Moves joint_1 by 0.05 rad and restores it, then opens/closes the empty gripper.
Run only in the course simulator before a grasp task, with no other motion task.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from eco65b_grasp_interfaces.action import GripperControl


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    opts = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('joint_control_observer', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    states = {}
    samples = []

    def receive(msg):
        states.update(zip(msg.name, msg.position))
        samples.append((time.monotonic(), dict(states)))

    sub = node.create_subscription(JointState, '/arm/joint_states', receive, qos_profile_sensor_data)
    pub = node.create_publisher(JointState, '/hand_command', 10)
    report = {'status': 'failed', 'scope': 'real simulated joint feedback; no grasp/lift acceptance',
              'checks': [], 'errors': []}

    def until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for ' + str(predicate))
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait(future):
        until(future.done, 15)
        return future.result()

    def arrival(name, target, after):
        until(lambda: len([s for t, s in samples if t > after and name in s
                           and abs(s[name] - target) < 0.015]) >= 5)
        actual = states[name]
        report['checks'].append({'joint': name, 'target': target, 'actual': actual,
                                 'absolute_error': abs(actual-target), 'fresh_samples_min': 5})

    baseline = None
    try:
        until(lambda: 'joint_1' in states and 'gripper_l_joint1' in states and pub.get_subscription_count() > 0)
        peers = node.get_publishers_info_by_topic('/hand_command')
        # The course motion server publishes only after an explicit Action;
        # the documented precondition is that no such task is running.
        if any(p.node_name not in (node.get_name(), 'gripper_control_node', 'eco65b_grasp_moveit_node') for p in peers):
            raise RuntimeError('Another arm command publisher is active; refuse overlapping control')
        report['command_publishers'] = [p.node_name for p in peers]
        baseline = states['joint_1']
        report['initial_joint_1'] = baseline
        for target in (baseline + 0.05, baseline):
            msg = JointState()
            msg.name, msg.position = ['joint_1'], [target]
            msg.header.stamp = node.get_clock().now().to_msg()
            after = time.monotonic()
            pub.publish(msg)
            arrival('joint_1', target, after)
        client = ActionClient(node, GripperControl, '/gripper_control')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper Action unavailable')
        for command, target in (('open', 0.78), ('close', 0.0)):
            goal = GripperControl.Goal()
            goal.command, goal.speed, goal.wait_for_completion = command, 50.0, True
            after = time.monotonic()
            handle = wait(client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Gripper goal rejected')
            result = wait(handle.get_result_async())
            if result.status != 4 or not result.result.success:
                raise RuntimeError('Gripper Action failed: ' + result.result.message)
            arrival('gripper_l_joint1', target, after)
            report['checks'][-1].update(action=command, action_status=result.status)
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if baseline is not None:
            # Restore the small diagnostic arm displacement even if feedback fails.
            msg = JointState()
            msg.name, msg.position = ['joint_1'], [baseline]
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
        opts.report.parent.mkdir(parents=True, exist_ok=True)
        opts.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
