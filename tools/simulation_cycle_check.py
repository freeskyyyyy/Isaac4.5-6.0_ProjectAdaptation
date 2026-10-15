#!/usr/bin/env python3
"""Verify official pause/resume/stop/replay with real clock and camera data.

Resets the simulation to its saved initial state. Never sends robot commands.
Keep navigation, MoveIt, vision Actions and other task processes stopped.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from simulation_interfaces.srv import GetSimulationState, SetSimulationState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    opts = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('course_simulation_cycle_check',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    result = {'status': 'failed', 'scope': 'simulation lifecycle and sensor streams; no robot task acceptance',
              'phases': {}, 'errors': []}
    samples = {'clock': [], 'rgb': [], 'depth': []}
    arrivals = {key: [] for key in samples}

    def receive(msg, key):
        stamp = msg.clock if key == 'clock' else msg.header.stamp
        samples[key].append(stamp.sec + stamp.nanosec * 1e-9)
        arrivals[key].append(time.monotonic())

    subscriptions = [node.create_subscription(Clock, '/clock', lambda m: receive(m, 'clock'), qos_profile_sensor_data)]
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    for key in ('rgb', 'depth'):
        subscriptions.append(node.create_subscription(Image, f'/arm_camera/{key}',
                                                      lambda m, key=key: receive(m, key), qos))
    setter = node.create_client(SetSimulationState, '/set_simulation_state')
    getter = node.create_client(GetSimulationState, '/get_simulation_state')

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'No official service: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Official service timeout: {client.srv_name}')
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError(str(response))
        return response

    def state(value):
        request = SetSimulationState.Request()
        request.state.state = value
        call(setter, request)
        actual = call(getter, GetSimulationState.Request()).state.state
        if actual != value:
            raise RuntimeError(f'Simulation state {actual}, expected {value}')
        # Drain pre-transition samples before measuring a distinct phase.
        pump(0.5)

    def observe(name, seconds, playing):
        for key in samples:
            samples[key].clear()
            arrivals[key].clear()
        pump(seconds)
        summary = {}
        for key, stamps in samples.items():
            times = arrivals[key]
            span = stamps[-1] - stamps[0] if stamps else 0
            wall_span = times[-1] - times[0] if times else 0
            gap = max((b - a for a, b in zip(times, times[1:])), default=None)
            summary[key] = {'messages': len(stamps), 'unique_stamps': len(set(stamps)),
                            'sim_span': span, 'wall_span': wall_span, 'max_wall_gap': gap,
                            'first_stamp': stamps[0] if stamps else None,
                            'last_stamp': stamps[-1] if stamps else None}
            if playing and (len(set(stamps)) < 10 or span < 1 or wall_span < seconds * 0.75
                            or gap is None or gap > 1 or any(b < a for a, b in zip(stamps, stamps[1:]))):
                result['errors'].append(f'{name}: {key} did not remain continuous')
            if not playing and len(set(stamps)) > 1:
                result['errors'].append(f'{name}: {key} timestamps advanced while paused')
        result['phases'][name] = summary
        print(f'{name}: {json.dumps(summary)}', flush=True)
        return summary

    try:
        state(2)
        observe('paused', 2, False)
        state(1)
        before = observe('resumed', 20, True)
        state(0)
        observe('stopped', 2, False)
        state(1)
        after = observe('replayed', 20, True)
        old_time = before['clock']['last_stamp']
        new_time = after['clock']['first_stamp']
        if old_time is None or new_time is None or new_time >= old_time:
            result['errors'].append('Stop/replay did not reset the simulation clock')
        result['status'] = 'passed' if not result['errors'] else 'failed'
    except Exception as exc:
        result['errors'].append(str(exc))
    finally:
        opts.report.parent.mkdir(parents=True, exist_ok=True)
        opts.report.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
