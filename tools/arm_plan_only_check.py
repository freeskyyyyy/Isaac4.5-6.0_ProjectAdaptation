#!/usr/bin/env python3
"""Real MoveIt/Action regression: plan_only must publish zero commands.

Requires the arm server to remap /hand_command to /diagnostics/plan_only_command
and action_name to /diagnostics/grasp_plan_only. No robot controller subscribes
to this diagnostic command topic. Run with use_fixed_ee_offset=false.
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
from moveit_msgs.srv import GetPositionFK
from eco65b_grasp_interfaces.action import GraspToPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    opts = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('arm_plan_only_observer',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    report = {'status': 'failed', 'scope': 'real plan-only Action with command output remapped; no arm motion',
              'errors': [], 'command_messages': 0}
    joints = []
    commands = []
    subscriptions = [
        node.create_subscription(JointState, '/arm/joint_state_deal', lambda msg: joints.append(msg), qos_profile_sensor_data),
        node.create_subscription(JointState, '/diagnostics/plan_only_command', lambda msg: commands.append(msg), 10),
    ]

    def wait(future, timeout=30):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('ROS request timeout')
        return future.result()

    try:
        deadline = time.monotonic() + 10
        while (not joints or not node.get_publishers_info_by_topic('/diagnostics/plan_only_command')):
            if time.monotonic() > deadline:
                raise RuntimeError('No actual arm states or remapped diagnostic publisher')
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_publishers_info_by_topic('/hand_command'):
            raise RuntimeError('Original /hand_command has a publisher; isolate diagnostic server before testing')
        if node.get_subscriptions_info_by_topic('/diagnostics/plan_only_command'):
            peers = node.get_subscriptions_info_by_topic('/diagnostics/plan_only_command')
            if any(info.node_name != node.get_name() for info in peers):
                raise RuntimeError('Unexpected diagnostic command subscriber; refuse test')
        fk = node.create_client(GetPositionFK, '/compute_fk')
        if not fk.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt FK service unavailable')
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link_arm'
        request.fk_link_names = ['link_6']
        request.robot_state.joint_state = joints[-1]
        response = wait(fk.call_async(request))
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError(f'MoveIt FK failed: {response.error_code.val}')
        pose = response.pose_stamped[0]
        goal = GraspToPoint.Goal()
        goal.target.header = pose.header
        goal.target.point.x = pose.pose.position.x
        goal.target.point.y = pose.pose.position.y
        goal.target.point.z = pose.pose.position.z + 0.005
        goal.ee_link = 'link_6'
        goal.finger_link = 'link_6'
        goal.plan_only = True
        goal.normal.header.frame_id = 'base_link_arm'
        goal.short_axis.header.frame_id = 'base_link_arm'
        client = ActionClient(node, GraspToPoint, '/diagnostics/grasp_plan_only')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Isolated plan-only Action server unavailable')
        commands.clear()
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Plan-only goal rejected')
        response = wait(handle.get_result_async())
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        report.update(action_status=response.status, action_success=response.result.success,
                      action_message=response.result.message, command_messages=len(commands))
        if not response.result.success or response.status != 4:
            report['errors'].append(f'Planning did not succeed: {response.result.message}')
        if commands:
            report['errors'].append('plan_only published joint commands before returning')
        report['status'] = 'passed' if not report['errors'] else 'failed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        opts.report.parent.mkdir(parents=True, exist_ok=True)
        opts.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
