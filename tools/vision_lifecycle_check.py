#!/usr/bin/env python3
"""Exercise real vision Action guards across official pause/reset/play.

Resets the scene. Run without navigation or grasp tasks. No motion commands.
Does not certify positive object localization or physical grasping.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from eco65b_grasp_interfaces.action import DetectObject
from simulation_interfaces.srv import SetSimulationState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('vision_lifecycle_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, DetectObject, '/detect_object')
    setter = node.create_client(SetSimulationState, '/set_simulation_state')
    report = {'status': 'failed', 'scope': 'real vision pause/reset guards only; no localization acceptance',
              'checks': {}, 'errors': []}
    active = None
    feedback = []

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(future, timeout=5):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError('ROS wall timeout')
            rclpy.spin_once(node, timeout_sec=.01)
        return future.result()

    def state(value):
        request = SetSimulationState.Request()
        request.state.state = value
        response = wait(setter.call_async(request))
        if response.result.result != 1:
            raise RuntimeError(str(response))

    def send(timeout):
        feedback.clear()
        goal = DetectObject.Goal(target_classes=['pencil', 'pen'], confidence_threshold=.99, timeout=timeout)
        handle = wait(client.send_goal_async(goal, feedback_callback=lambda m: feedback.append(
            {'state': m.feedback.state, 'attempts': m.feedback.detection_attempts})))
        if not handle.accepted:
            raise RuntimeError('Diagnostic goal rejected (possibly residual inference)')
        return handle

    def record(name, response, elapsed):
        data = {'status': response.status, 'success': response.result.success,
                'message': response.result.message, 'wall_seconds': elapsed,
                'max_attempts': max((m['attempts'] for m in feedback), default=0)}
        report['checks'][name] = data
        return data

    try:
        if not client.wait_for_server(timeout_sec=15) or not setter.wait_for_service(timeout_sec=10):
            raise RuntimeError('Vision or official simulation service missing')
        state(2)
        pump(.5)
        start = time.monotonic()
        active = send(1.5)
        response = wait(active.get_result_async(), 4)
        active = None
        data = record('paused_wall_timeout', response, time.monotonic() - start)
        if data['status'] != 6 or data['success'] or data['wall_seconds'] > 3.5 or 'timed out' not in data['message']:
            raise RuntimeError('Paused Action did not obey wall timeout')
        pump(.5)
        active = send(20.)
        pump(.2)
        start = time.monotonic()
        cancellation = wait(active.cancel_goal_async())
        response = wait(active.get_result_async(), 3)
        active = None
        data = record('paused_cancel', response, time.monotonic() - start)
        if not cancellation.goals_canceling or data['status'] != 5 or data['success'] or data['wall_seconds'] > 2:
            raise RuntimeError('Paused Action cancellation failed')
        state(1)
        pump(2)
        active = send(20.)
        pump(.2)
        start = time.monotonic()
        state(0)
        # Stop freezes /clock at its previous value. The negative clock jump
        # becomes observable to ROS subscribers only after replay publishes.
        pump(.2)
        state(1)
        response = wait(active.get_result_async(), 4)
        active = None
        data = record('reset_aborts_active_goal', response, time.monotonic() - start)
        if data['status'] != 6 or data['success'] or 'backwards' not in data['message']:
            raise RuntimeError('Reset did not abort the previous epoch goal')
        state(1)
        pump(2)
        start = time.monotonic()
        active = send(2.)
        response = wait(active.get_result_async(), 5)
        active = None
        data = record('new_goal_after_reset', response, time.monotonic() - start)
        if data['status'] != 6 or data['success'] or 'timed out' not in data['message'] or not data['max_attempts']:
            raise RuntimeError('Fresh post-reset goal did not consume camera data')
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if active is not None:
            try:
                wait(active.cancel_goal_async(), 3)
            except Exception as exc:
                report['errors'].append('Cleanup: ' + str(exc))
        try:
            state(1)
        except Exception as exc:
            report['errors'].append('Restore play: ' + str(exc))
        if report['errors']:
            report['status'] = 'failed'
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
