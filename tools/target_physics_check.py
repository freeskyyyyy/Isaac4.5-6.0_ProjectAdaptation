#!/usr/bin/env python3
"""Observe an authored marker settling and resetting through official services.

Run with robot/control stacks stopped. This resets the scene; it never sends
robot motion commands. Physics evidence does not certify vision or grasping.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from simulation_interfaces.srv import GetEntityState, SetSimulationState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--preparation', type=Path, required=True)
    args = parser.parse_args()
    preparation = json.loads(args.preparation.read_text())
    expected = preparation['authored_position_m']
    rclpy.init()
    node = rclpy.create_node('course_target_physics_observer')
    getter = node.create_client(GetEntityState, '/get_entity_state')
    setter = node.create_client(SetSimulationState, '/set_simulation_state')
    report = {'status': 'failed', 'scope': 'marker settling/reset only; no vision or grasp acceptance',
              'entity': preparation['prim'], 'phases': {}, 'errors': []}

    def call(client, request):
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('Missing official service: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('Official service timed out: ' + client.srv_name)
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError(str(response))
        return response

    def state(value):
        request = SetSimulationState.Request()
        request.state.state = value
        call(setter, request)

    def observe(seconds):
        result = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            request = GetEntityState.Request(entity=preparation['prim'])
            response = call(getter, request)
            p, q = response.state.pose.position, response.state.pose.orientation
            result.append({'position': [p.x, p.y, p.z], 'orientation_xyzw': [q.x, q.y, q.z, q.w]})
            rclpy.spin_once(node, timeout_sec=.1)
        return result

    try:
        state(0)
        report['phases']['initial'] = observe(.5)
        for index in range(2):
            initial = report['phases']['initial' if index == 0 else 'reset'][-1]['position']
            if math.dist(initial, expected) > .0005:
                raise RuntimeError('Stopped marker did not return to authored position')
            state(1)
            samples = observe(6)
            report['phases'][f'settling_{index + 1}'] = samples
            final = samples[-1]['position']
            drop = initial[2] - final[2]
            report[f'drop_{index + 1}_m'] = drop
            if not .0005 < drop < .015:
                raise RuntimeError(f'Marker did not settle onto table: drop={drop}')
            if math.dist(initial[:2], final[:2]) > .03:
                raise RuntimeError('Marker moved away from placement')
            if max(math.dist(final, sample['position']) for sample in samples[-10:]) > .001:
                raise RuntimeError('Marker remained unstable')
            if index == 0:
                state(0)
                report['phases']['reset'] = observe(.5)
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'phases'}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
