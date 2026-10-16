#!/usr/bin/env python3
"""Real DDS/GPU Action diagnostics; never certifies object localization/grasping.

Run in the initial course view (no pencil/pen visible). Tests an 0.99-confidence
no-target timeout, concurrent-goal rejection and real Action cancellation.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from eco65b_grasp_interfaces.action import DetectObject


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--expect-missing-tf-frame',
                        help='Negative test: require this deliberately absent frame in timeout diagnostics')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('vision_action_observer', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    client = ActionClient(node, DetectObject, '/detect_object')
    report = {'status': 'failed', 'scope': 'real vision Action diagnostics only; not positive localization',
              'checks': {}, 'errors': []}
    feedback = []
    active = None

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('Wall-clock wait exceeded ' + str(timeout))
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait(future, timeout=10):
        spin_until(future.done, timeout)
        return future.result()

    def send(timeout):
        goal = DetectObject.Goal()
        goal.target_classes = ['pencil', 'pen']
        goal.confidence_threshold = 0.99
        goal.timeout = timeout
        return wait(client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(
            {'state': msg.feedback.state, 'attempts': msg.feedback.detection_attempts})))

    try:
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Vision Action server unavailable')
        report['camera_subscriptions'] = {}
        for topic in ('/arm_camera/rgb', '/arm_camera/depth', '/arm_camera/camera_info'):
            peers = [p for p in node.get_subscriptions_info_by_topic(topic)
                     if p.node_name == 'yoloe_grasp_node']
            if len(peers) != 1 or peers[0].qos_profile.reliability.name != 'RELIABLE':
                raise RuntimeError('Missing RELIABLE live vision endpoint: ' + topic)
            report['camera_subscriptions'][topic] = {'reliability': 'RELIABLE',
                                                    'depth': peers[0].qos_profile.depth}
        start = time.monotonic()
        active = send(3.0)
        if not active.accepted:
            raise RuntimeError('Timeout diagnostic goal rejected')
        result = wait(active.get_result_async(), 10)
        elapsed = time.monotonic() - start
        report['checks']['no_target_timeout'] = {'wall_seconds': elapsed, 'status': result.status,
            'success': result.result.success, 'message': result.result.message,
            'feedback_states': sorted({f['state'] for f in feedback}),
            'max_attempts': max((f['attempts'] for f in feedback), default=0)}
        active = None
        if result.status != 6 or result.result.success or elapsed > 6:
            raise RuntimeError('Expected bounded no-target ABORTED result')
        if args.expect_missing_tf_frame:
            if args.expect_missing_tf_frame not in result.result.message:
                report['errors'].append('Missing TF frame was not identified in timeout diagnostics')
            if any(f['attempts'] > 0 for f in feedback):
                report['errors'].append('Missing TF should prevent inference submission')
        elif not any(f['attempts'] > 0 for f in feedback):
            report['errors'].append('Timeout goal did not submit any image inference')
        feedback.clear()
        active = send(20.0)
        if not active.accepted:
            raise RuntimeError('Cancellation diagnostic goal rejected')
        spin_until(lambda: bool(feedback) if args.expect_missing_tf_frame
                   else any(f['attempts'] > 0 for f in feedback), 8)
        second = send(2.0)
        report['checks']['concurrent_goal'] = {'accepted': second.accepted}
        if second.accepted:
            wait(second.cancel_goal_async())
            raise RuntimeError('Concurrent vision goal accepted')
        start = time.monotonic()
        cancel = wait(active.cancel_goal_async())
        result = wait(active.get_result_async(), 5)
        elapsed = time.monotonic() - start
        report['checks']['cancel'] = {'wall_seconds': elapsed, 'status': result.status,
            'goals_canceling': len(cancel.goals_canceling), 'success': result.result.success,
            'message': result.result.message}
        active = None
        if not cancel.goals_canceling or result.status != 5 or result.result.success or elapsed > 2:
            raise RuntimeError('Cancellation did not terminate within 2 wall seconds')
        report['status'] = 'passed' if not report['errors'] else 'failed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if active is not None:
            try:
                wait(active.cancel_goal_async(), 3)
            except Exception as exc:
                report['errors'].append('Cleanup cancellation: ' + str(exc))
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
