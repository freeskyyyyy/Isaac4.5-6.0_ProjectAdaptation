#!/usr/bin/env python3
"""Execute only the accepted staging goal after the whole-path gate passes.

Observes simulator ground truth, arm transport posture, localization and final
stop. Cancels on bounded geometry/localization failures. Never approaches 2.02m.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from navigation_guard import guard_reason, stopping_reserve, tracking_error
from simulation_interfaces.srv import GetEntityState
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan-report', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--timeout', type=float, default=900.)
    a = p.parse_args()
    accepted = json.loads(a.plan_report.read_text())
    gate = accepted.get('clearance_gate', {})
    if accepted.get('status') != 'passed' or accepted.get('execution_ready') is not True or gate.get('exit_code') != 0:
        p.error('A successful real plan and propagated whole-path clearance gate are required')
    detail = json.loads(Path(gate['report']).read_text())
    if detail.get('passed') is not True or detail.get('minimum_center_clearance_lower_bound_m', -1) < .55:
        p.error('Clearance lower bound must be >=0.55m')
    if hashlib.sha256(Path(gate['input_report']).read_bytes()).hexdigest() != detail['input_sha256']:
        p.error('Frozen gate input hash changed')
    map_path = Path(get_package_share_directory('bobac3_navigation'))/'maps/world_scene.yaml'
    meta = yaml.safe_load(map_path.read_text())
    image_path = map_path.parent/meta['image']
    if hashlib.sha256(image_path.read_bytes()).hexdigest() != detail['map_image_sha256']:
        p.error('Planning map changed since clearance acceptance')
    image = np.asarray(Image.open(image_path).convert('L'))[::-1]
    free = (255-image.astype(float))/255 < meta['free_thresh']
    rows, cols = np.nonzero(~free)
    origin = np.array(meta['origin'][:2]); resolution = float(meta['resolution'])
    obstacles = cKDTree(origin+np.column_stack((cols+.5, rows+.5))*resolution)
    high = origin+np.array([image.shape[1], image.shape[0]])*resolution
    script = Path(get_package_prefix('bobac3_navigation'))/'lib/bobac3_navigation/check_path_clearance.py'
    spec = importlib.util.spec_from_file_location('course_path_clearance', script)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    reference = [accepted['initial_localization']['map_base_xy']] + accepted['planner']['points_xy']
    rclpy.init()
    n = rclpy.create_node('staging_goal_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer(node=n); listener = TransformListener(tf, n)
    joints, commands, feedback = [], [], []
    last_joint_wall = [time.monotonic()]
    last_odom_wall = [0.]
    current_path = [reference]
    plan_failure = []
    collecting_braking = [False]
    def stamp(value):
        return value.sec + value.nanosec*1e-9
    def sim_now():
        return n.get_clock().now().nanoseconds*1e-9
    def receive_command(message, topic):
        if topic == '/cmd_vel':
            commands.append(message)
        report['velocity_history'].append({'topic': topic, 'receipt_sim_seconds': sim_now(),
            'stamp_kind': 'simulation receipt time; Twist has no header',
            'wall_seconds': time.monotonic()-start,
            'xy_wz': [message.linear.x, message.linear.y, message.angular.z]})
    def receive_odom(message):
        last_odom_wall[0] = time.monotonic()
        t = message.twist.twist
        report['odom_history'].append({'sim_seconds': stamp(message.header.stamp),
            'receipt_sim_seconds': sim_now(), 'wall_seconds': time.monotonic()-start,
            'xy_wz': [t.linear.x, t.linear.y, t.angular.z]})
    def receive_path(message):
        points = [[p.pose.position.x, p.pose.position.y] for p in message.poses]
        item = {'sim_seconds': stamp(message.header.stamp), 'receipt_sim_seconds': sim_now(),
                'frame': message.header.frame_id, 'points_xy': points}
        try:
            item['clearance'] = module.check(points, map_path)
            if message.header.frame_id != 'map' or not item['clearance']['passed']:
                plan_failure.append('New runtime plan failed frame/whole-path clearance gate')
            else:
                # Grid planner first point is a quantized cell, not the exact
                # robot start. Preserve the observed start connector explicitly.
                connector = report['trace'][-1]['world_xy'] if report['trace'] else reference[0]
                item['observed_start_connector_xy'] = connector
                item['connected_clearance'] = module.check([connector]+points, map_path)
                if not item['connected_clearance']['passed']:
                    plan_failure.append('Runtime plan start connector failed clearance')
                current_path[0] = [connector]+points
        except Exception as exc:
            item['error'] = str(exc); plan_failure.append('Invalid runtime plan: '+str(exc))
        report['replanning_history'].append(item)
    def receive_joints(message):
        joints.append(message)
        last_joint_wall[0] = time.monotonic()
    subs = [n.create_subscription(JointState, '/arm/joint_state_deal', receive_joints, qos_profile_sensor_data),
            n.create_subscription(Twist, '/cmd_vel', lambda m: receive_command(m, '/cmd_vel'), 100),
            n.create_subscription(Twist, '/cmd_vel_nav', lambda m: receive_command(m, '/cmd_vel_nav'), 100),
            n.create_subscription(Odometry, '/odom', receive_odom, qos_profile_sensor_data),
            n.create_subscription(NavPath, '/plan', receive_path, 10)]
    stop = n.create_publisher(Twist, '/cmd_vel', 10)
    client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    getter = n.create_client(GetEntityState, '/get_entity_state')
    report = {'status': 'failed', 'scope': 'real staging navigation only; no grasp acceptance',
        'goal_xy_yaw': [2.35, .6, math.pi], 'trace': [], 'errors': [], 'feedback': feedback,
        'accepted_plan': str(a.plan_report), 'plan_sha256': hashlib.sha256(a.plan_report.read_bytes()).hexdigest(),
        'velocity_history': [], 'odom_history': [], 'replanning_history': [],
        'guard_policy': {'clearance_limit_m': .55, 'tracking_limit_m': .05,
            'assumed_braking_floor_m_s2': .10, 'reaction_seconds': .40,
            'clearance_uncertainty_m': .025, 'physical_braking_validated': False}}
    active = None
    pending = None
    last_query = 0.
    query_sim_time = 0.
    start = time.monotonic()

    def wait(future, timeout=10):
        end = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= end:
                raise RuntimeError('ROS wall timeout')
            rclpy.spin_once(n, timeout_sec=.02)
        return future.result()

    def observe(enforce=True):
        nonlocal pending, last_query, query_sim_time
        rclpy.spin_once(n, timeout_sec=.01)
        now = time.monotonic()
        if pending is not None and pending.done():
            response = pending.result(); pending = None
            if response is None or response.result.result != 1:
                raise RuntimeError('Ground truth query failed')
            pose = response.state.pose
            source_stamp = stamp(response.state.header.stamp)
            # rc.59 entity_utils explicitly returns Header().stamp (zero).
            # Preserve that fact and bracket the asynchronous read using /clock.
            # Native /odom stamps remain authoritative for speed derivatives.
            sample_time = source_stamp if source_stamp > 0 else sim_now()
            xy = [pose.position.x, pose.position.y]
            clear = min(float(obstacles.query(xy)[0])-resolution*math.sqrt(2)/2,
                        float(np.minimum(np.array(xy)-origin, high-np.array(xy)).min()))
            twist = response.state.twist
            speed = math.hypot(twist.linear.x, twist.linear.y)
            sample = {'wall_seconds': now-start, 'sim_seconds': sample_time,
                      'source_sim_stamp_seconds': source_stamp,
                      'stamp_kind': 'native' if source_stamp > 0 else 'simulation receipt time; unstamped SDK service',
                      'query_sim_interval': [query_sim_time, sim_now()],
                      'receipt_sim_seconds': sim_now(), 'world_xy': xy, 'world_yaw': yaw(pose.orientation),
                      'world_velocity': [twist.linear.x, twist.linear.y, twist.angular.z],
                      'phase': 'braking' if collecting_braking[0] else ('motion' if active is not None else 'warmup'),
                      'center_clearance_m': clear, 'query_latency_wall_seconds': now-last_query}
            capture = Time(seconds=sample_time)
            tf_end = time.monotonic()+.5
            while enforce and active is not None and not tf.can_transform('map', 'base_link', capture) and time.monotonic() < tf_end:
                rclpy.spin_once(n, timeout_sec=.005)
            if tf.can_transform('map', 'base_link', capture):
                t = tf.lookup_transform('map', 'base_link', capture).transform
                sample['localization_position_error_m'] = math.dist(xy, [t.translation.x, t.translation.y])
            deviation = max(tracking_error(xy, reference), tracking_error(xy, current_path[0]))
            previous = report['trace'][-1] if report['trace'] else sample
            dt = sample_time-previous['sim_seconds']
            outward = max(0., (deviation-previous.get('tracking_error_m', deviation))/dt) if dt > 0 else 0.
            age = max(0., sim_now()-query_sim_time)
            sample.update(tracking_error_m=deviation, outward_speed_m_s=outward,
                          stopping_reserve_m=stopping_reserve(speed, age), speed_m_s=speed)
            report['trace'].append(sample)
            if active is not None and enforce:
                if dt < 0:
                    raise RuntimeError('Simulation clock rewound during motion')
                if 'localization_position_error_m' not in sample:
                    try:
                        tf.lookup_transform('map', 'base_link', capture)
                    except Exception as exc:
                        report['localization_tf_failure'] = str(exc)
                    raise RuntimeError('Capture-time localization TF unavailable during motion')
                if sample['localization_position_error_m'] > .03:
                    raise RuntimeError('Localization differs from ground truth by >0.03m')
                reason = guard_reason(clear, deviation, speed, age, outward)
                if reason:
                    report['stop_trigger'] = dict(sample, reason=reason)
                    raise RuntimeError(reason)
        if pending is None and now-last_query >= .04:
            pending = getter.call_async(GetEntityState.Request(entity=
                '/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link'))
            last_query = now
            query_sim_time = sim_now()
        if active is not None and enforce and plan_failure:
            raise RuntimeError(plan_failure[0])
        if active is not None and enforce and now-last_odom_wall[0] > .5:
            raise RuntimeError('Closed-loop odometry became stale')
        if joints and any(abs(v) > .03 for v in joints[-1].position):
            raise RuntimeError('Arm left verified transport posture; canceling')
        if joints and now-last_joint_wall[0] > 1.5:
            raise RuntimeError('Arm posture feedback became stale; canceling')
        if report['trace'] and now-start-report['trace'][-1]['wall_seconds'] > 1.5:
            raise RuntimeError('Simulator ground-truth feedback became stale; canceling')

    try:
        if not client.wait_for_server(timeout_sec=15) or not getter.wait_for_service(timeout_sec=10):
            raise RuntimeError('Navigation or simulator service unavailable')
        end = time.monotonic()+25
        while time.monotonic() < end:
            observe()
            if (len(report['trace']) >= 5 and joints and report['odom_history'] and
                    'localization_position_error_m' in report['trace'][-1]):
                break
        if not joints or len(report['trace']) < 5:
            raise RuntimeError('Missing transport posture or fresh ground truth')
        first = report['trace'][-1]
        if first.get('localization_position_error_m', 1) > .03:
            raise RuntimeError('Fresh capture-time localization not within 0.03m')
        if math.dist(first['world_xy'], accepted['initial_localization']['ground_truth_xy']) > .03:
            raise RuntimeError('Robot moved since accepted planning start')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = n.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = 2.35, .6
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = 1., 0.
        def receive(msg):
            f = msg.feedback
            feedback.append({'wall_seconds': time.monotonic()-start, 'receipt_sim_seconds': sim_now(),
                'distance_remaining': f.distance_remaining, 'recoveries': f.number_of_recoveries})
        active = wait(client.send_goal_async(goal, feedback_callback=receive))
        if not active.accepted:
            raise RuntimeError('Staging goal rejected')
        report['goal_uuid'] = bytes(active.goal_id.uuid).hex()
        result_future = active.get_result_async()
        deadline = time.monotonic()+a.timeout
        while not result_future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError('Staging navigation wall timeout')
            observe()
        response = result_future.result(); active = None
        report['action_status'] = response.status
        report['action_error_code'] = response.result.error_code
        report['action_error_msg'] = response.result.error_msg
        if response.status != 4 or response.result.error_code:
            raise RuntimeError('NavigateToPose did not succeed')
        end = time.monotonic()+2
        while time.monotonic() < end:
            observe()
        final = report['trace'][-1]
        report['final_position_error_m'] = math.dist(final['world_xy'], [2.35, .6])
        report['final_yaw_error_rad'] = abs(math.atan2(math.sin(final['world_yaw']-math.pi), math.cos(final['world_yaw']-math.pi)))
        if report['final_position_error_m'] > .08 or report['final_yaw_error_rad'] > .08:
            raise RuntimeError('Actual simulator pose differs from requested staging goal')
        if not commands or any(abs(v) > .01 for v in [commands[-1].linear.x, commands[-1].linear.y, commands[-1].angular.z]):
            raise RuntimeError('Final velocity command is not zero')
        if math.dist(final['world_xy'], report['trace'][-5]['world_xy']) > .003:
            raise RuntimeError('Robot still moving after success')
        report['actual_trajectory_clearance'] = module.check([x['world_xy'] for x in report['trace']], map_path)
        if not report['actual_trajectory_clearance']['passed']:
            raise RuntimeError('Observed trajectory did not retain required transport clearance')
        report['status'] = 'passed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if active is not None:
            try:
                collecting_braking[0] = True
                report['cancel_sim_seconds'] = sim_now()
                cancellation = active.cancel_goal_async()
                braking_end = time.monotonic()+8
                next_zero = 0.
                while time.monotonic() < braking_end:
                    if time.monotonic() >= next_zero:
                        stop.publish(Twist()); next_zero = time.monotonic()+.05
                    observe(enforce=False)
                report['cancel_acknowledged'] = cancellation.done()
                report['cancel_action_status'] = wait(active.get_result_async(), 5).status
                report['cleanup_canceled_goal'] = True
            except Exception as exc:
                report['errors'].append('Cancel cleanup: '+str(exc))
        end = time.monotonic()+.5
        next_zero = 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_zero:
                stop.publish(Twist()); next_zero = time.monotonic()+.05
            rclpy.spin_once(n, timeout_sec=.01)
        report['final_zero_twist_sent'] = True
        if report['trace']:
            report['actual_trajectory_clearance'] = module.check([x['world_xy'] for x in report['trace']], map_path)
            report['maximum_tracking_error_m'] = max(x['tracking_error_m'] for x in report['trace'])
            if not report['actual_trajectory_clearance']['passed'] or report['maximum_tracking_error_m'] > .05:
                report['status'] = 'failed'
            braking = [s for s in report['trace'] if s['phase'] == 'braking']
            if braking:
                stopped = [s for index, s in enumerate(braking)
                           if len(braking[index:index+5]) == 5 and
                           all(x['speed_m_s'] < .005 for x in braking[index:index+5])]
                trigger = report.get('stop_trigger', braking[0])
                report['braking_summary'] = {'observed_stopped': bool(stopped),
                    'stop_sim_seconds': stopped[0]['sim_seconds'] if stopped else None,
                    'distance_from_trigger_m': math.dist(trigger['world_xy'], (stopped[0] if stopped else braking[-1])['world_xy']),
                    'final_speed_m_s': braking[-1]['speed_m_s']}
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in
            ('trace','feedback','velocity_history','odom_history','replanning_history')}, indent=2), flush=True)
        n.destroy_node();rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
