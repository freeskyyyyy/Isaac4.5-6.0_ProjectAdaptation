#!/usr/bin/env python3
"""Read-only real map/laser/ground-truth alignment and northern Nav2 planning.

Never sends NavigateToPose or velocity commands. Path clearance is reported
separately; a successful planner result alone is not navigation acceptance.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import ComputePathToPose
from simulation_interfaces.srv import GetEntityState
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--clearance-map', type=Path,
                        help='Full planning map; defaults to installed world_scene.yaml')
    args = parser.parse_args()
    clearance_map = args.clearance_map or Path(get_package_share_directory('bobac3_navigation'))/'maps/world_scene.yaml'
    clearance_tool = Path(get_package_prefix('bobac3_navigation'))/'lib/bobac3_navigation/check_path_clearance.py'
    rclpy.init()
    node = rclpy.create_node('world_navigation_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer(node=node)
    listener = TransformListener(tf, node)
    maps, scans = {}, []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(OccupancyGrid, topic, lambda m, t=topic: maps.update({t: m}), qos)
            for topic in ('/map', '/world_obstacles')]
    subs.append(node.create_subscription(LaserScan, '/laser_scan_fuse', lambda m: scans.append(m), qos_profile_sensor_data))
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    getter = node.create_client(GetEntityState, '/get_entity_state')
    report = {'status': 'failed', 'scope': 'real static alignment and plan only; no navigation execution',
              'execution_ready': False, 'execution_blockers': [], 'errors': []}
    active = None

    def wait(future, timeout=15):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError('ROS wall timeout')
            rclpy.spin_once(node, timeout_sec=.02)
        return future.result()

    def transform(parent, child, stamp):
        deadline = time.monotonic() + 3
        while not tf.can_transform(parent, child, stamp):
            if time.monotonic() >= deadline:
                raise RuntimeError(f'No capture-time TF: {parent} <- {child}')
            rclpy.spin_once(node, timeout_sec=.02)
        return tf.lookup_transform(parent, child, stamp).transform

    def occupied(message):
        grid = np.asarray(message.data).reshape(message.info.height, message.info.width)
        rows, cols = np.nonzero(grid >= 65)
        return np.column_stack((message.info.origin.position.x + (cols+.5)*message.info.resolution,
                                message.info.origin.position.y + (rows+.5)*message.info.resolution))

    def distances(points, obstacles):
        return np.array([np.sqrt(((obstacles - p)**2).sum(axis=1).min()) for p in points])

    try:
        deadline = time.monotonic() + 20
        while (len(maps) < 2 or len(scans) < 10 or not tf.can_transform(
                'map', scans[-1].header.frame_id, Time.from_msg(scans[-1].header.stamp))):
            if time.monotonic() >= deadline:
                raise RuntimeError('Maps/scans or capture-time map TF not ready after warmup')
            rclpy.spin_once(node, timeout_sec=.05)
        localization, obstacle = maps['/map'], maps['/world_obstacles']
        for message in (localization, obstacle):
            q = message.info.origin.orientation
            if abs(yaw(q)) > 1e-6 or message.header.frame_id != 'map':
                raise RuntimeError('Expected world-axis-aligned map')
        if (localization.info.width, localization.info.height, localization.info.resolution,
            localization.info.origin) != (obstacle.info.width, obstacle.info.height,
                                         obstacle.info.resolution, obstacle.info.origin):
            raise RuntimeError('Localization and planning map grids differ')
        scan = scans[-1]
        t = transform('map', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        report['scan_pose'] = {'map_xy': [t.translation.x, t.translation.y], 'map_yaw': yaw(t.rotation),
            'quaternion_xyzw': [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
            'frame': scan.header.frame_id, 'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
            'range_min': scan.range_min, 'range_max': scan.range_max, 'ranges': list(scan.ranges)}
        a = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        r = np.asarray(scan.ranges)
        valid = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max-.05)
        # This course mounts the lidar upside down. Full rotation is required;
        # yaw-only projection would mirror Y and falsely accuse the scan order.
        q = t.rotation
        rotation = np.array([
            [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w)],
            [2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w)],
            [2*(q.x*q.z-q.y*q.w), 2*(q.y*q.z+q.x*q.w), 1-2*(q.x*q.x+q.y*q.y)]])
        local = np.column_stack((r[valid]*np.cos(a[valid]), r[valid]*np.sin(a[valid]), np.zeros(valid.sum())))
        world = local @ rotation.T + np.array([t.translation.x, t.translation.y, t.translation.z])
        points = world[:, :2]
        angles = np.arctan2(points[:, 1]-t.translation.y, points[:, 0]-t.translation.x)
        residuals = distances(points, occupied(localization))
        quadrants = {}
        for index in range(4):
            mask = ((angles % (2*math.pi)) // (math.pi/2)).astype(int) == index
            if mask.any():
                quadrants[str(index)] = {'hits': int(mask.sum()),
                    'fraction_within_0_10m': float((residuals[mask] <= .10).mean())}
        report['laser_alignment'] = {'valid_hits': len(points), 'median_residual_m': float(np.median(residuals)),
            'p95_residual_m': float(np.percentile(residuals, 95)),
            'fraction_within_0_10m': float((residuals <= .10).mean()), 'quadrants': quadrants,
            'points_xy': points.tolist()}
        if not getter.wait_for_service(timeout_sec=5):
            raise RuntimeError('No simulator ground-truth service')
        entity = '/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint/base_link'
        response = wait(getter.call_async(GetEntityState.Request(entity=entity)))
        if response.result.result != 1:
            raise RuntimeError(str(response))
        ground = response.state.pose
        estimate = transform('map', 'base_link', Time())
        xy_error = math.hypot(ground.position.x-estimate.translation.x, ground.position.y-estimate.translation.y)
        angular_error = abs(math.atan2(math.sin(yaw(ground.orientation)-yaw(estimate.rotation)),
                                      math.cos(yaw(ground.orientation)-yaw(estimate.rotation))))
        report['initial_localization'] = {'ground_truth_xy': [ground.position.x, ground.position.y],
            'ground_truth_yaw': yaw(ground.orientation),
            'map_base_xy': [estimate.translation.x, estimate.translation.y],
            'xy_error_m': xy_error, 'yaw_error_rad': angular_error}
        if xy_error > .1 or angular_error > .08:
            raise RuntimeError('AMCL estimate differs from actual simulator pose')
        if len(points) < 30 or (residuals <= .10).mean() < .85 or sum(
                v['hits'] >= 5 and v['fraction_within_0_10m'] >= .8 for v in quadrants.values()) < 3:
            raise RuntimeError('Real laser does not match at least three map directions')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Planner Action unavailable')
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = 2.35, .6
        goal.goal.pose.orientation.z = 1.
        goal.planner_id = 'GridBased'
        goal.use_start = False
        active = wait(client.send_goal_async(goal))
        if not active.accepted:
            raise RuntimeError('Planning request rejected')
        response = wait(active.get_result_async(), 30)
        active = None
        result = response.result
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in result.path.poses])
        report['planner'] = {'status': response.status, 'error_code': result.error_code,
                             'error_msg': result.error_msg, 'points_xy': path.tolist()}
        if response.status != 4 or result.error_code or len(path) < 2:
            raise RuntimeError('Planning failed')
        crossing = path[(path[:, 0] >= 5.4) & (path[:, 0] <= 6.)]
        report['planner']['north_crossing_min_y'] = float(crossing[:, 1].min()) if len(crossing) else None
        clearance = distances(path, occupied(obstacle)) - obstacle.info.resolution*math.sqrt(2)/2
        report['planner']['min_center_clearance_m'] = float(clearance.min())
        report['planner']['circular_0_45m_envelope_clear'] = bool(clearance.min() >= .45)
        report['planner']['min_clearance_point_xy'] = path[int(clearance.argmin())].tolist()
        if clearance.min() < .45:
            report['execution_blockers'].append('Path center clearance is smaller than the 0.45m transit envelope')
        if not len(crossing) or crossing[:, 1].min() < 2.5:
            raise RuntimeError('Planned path does not use the northern passage')
        report['status'] = 'passed'
        report['execution_ready'] = not report['execution_blockers']
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if active is not None:
            try:
                wait(active.cancel_goal_async(), 3)
            except Exception:
                pass
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
        if report.get('planner', {}).get('status') == 4 and report['status'] == 'passed':
            clearance_report = args.report.with_name(args.report.stem+'.clearance.json')
            plan_input = args.report.with_name(args.report.stem+'.plan-input.json')
            plan_input.write_bytes(args.report.read_bytes())
            try:
                gate = subprocess.run([sys.executable, str(clearance_tool),
                    '--alignment-report', str(plan_input), '--map', str(clearance_map),
                    '--report', str(clearance_report)], capture_output=True, text=True, timeout=30)
                detail = json.loads(clearance_report.read_text())
                report['clearance_gate'] = {'exit_code': gate.returncode, 'report': str(clearance_report),
                    'input_report': str(plan_input),
                    'passed': detail.get('passed', False),
                    'minimum_center_clearance_lower_bound_m': detail.get('minimum_center_clearance_lower_bound_m')}
                report['execution_ready'] = (gate.returncode == 0 and detail.get('passed') is True
                    and detail.get('minimum_center_clearance_lower_bound_m', -1) >= .55)
                if not report['execution_ready']:
                    report['status'] = 'failed'
                    report['errors'].append('Whole-path clearance gate failed; motion is blocked')
                else:
                    report['execution_blockers'] = []
            except Exception as exc:
                report['status'] = 'failed'
                report['execution_ready'] = False
                report['errors'].append('Whole-path clearance gate could not complete: '+str(exc))
            args.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('laser_alignment', 'planner')}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
