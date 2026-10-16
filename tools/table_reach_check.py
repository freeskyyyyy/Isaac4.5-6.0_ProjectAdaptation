#!/usr/bin/env python3
"""Evaluate hypothetical desk stations using real MoveIt with table collisions.

No robot/base commands. Temporarily places table collision boxes in arm_base
as viewed from each hypothetical station, then removes only its own objects.
Run with all motion tasks stopped. An IK/path success is not physical grasping.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK, GetMotionPlan, GetPositionFK, GetStateValidity, GetCartesianPath


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--geometry', type=Path, required=True)
    p.add_argument('--candidates', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--finger-x', type=float,
                   help='Hypothetical target world X only; does not move the scene object')
    p.add_argument('--finger-z', type=float,
                   help='Hypothetical finger world Z for approach/lift planning; no motion')
    p.add_argument('--candidate-name', action='append',
                   help='Evaluate only these named stations (repeatable)')
    p.add_argument('--grasp-z', type=float, help='Optionally plan vertical approach to this world finger Z')
    p.add_argument('--lift-z', type=float, help='Then plan vertical lift to this world finger Z')
    a = p.parse_args()
    geometry = json.loads(a.geometry.read_text())
    candidates = json.loads(a.candidates.read_text())['candidates']
    if a.candidate_name:
        candidates = [c for c in candidates if c['name'] in a.candidate_name]
        if len(candidates) != len(set(a.candidate_name)):
            p.error('Unknown candidate name')
    rclpy.init()
    n = rclpy.create_node('table_reach_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    joints, commands = [], []
    subs = [n.create_subscription(JointState, '/arm/joint_state_deal', joints.append, qos_profile_sensor_data),
            n.create_subscription(JointState, '/hand_command', commands.append, 10)]
    clients = {key: n.create_client(kind, name) for key, kind, name in [
        ('apply', ApplyPlanningScene, '/apply_planning_scene'), ('scene', GetPlanningScene, '/get_planning_scene'),
        ('ik', GetPositionIK, '/compute_ik'), ('plan', GetMotionPlan, '/plan_kinematic_path'),
        ('fk', GetPositionFK, '/compute_fk'), ('valid', GetStateValidity, '/check_state_validity'),
        ('cartesian', GetCartesianPath, '/compute_cartesian_path')]}
    report = {'status': 'failed', 'scope': 'hypothetical stationary arm planning with real MoveIt/table collisions; no motion',
              'candidates': [], 'errors': [], 'collision_model': 'all six table USD collision mesh world AABBs'}
    report['finger_x_override_m'] = a.finger_x
    report['finger_z_override_m'] = a.finger_z
    report['requested_grasp_z_m'] = a.grasp_z
    report['requested_lift_z_m'] = a.lift_z
    report['requested_lift_distance_m'] = (a.lift_z-a.grasp_z
        if a.lift_z is not None and a.grasp_z is not None else None)
    object_ids = []

    def wait(future, timeout=15):
        end = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() >= end:
                raise RuntimeError('MoveIt service wall timeout')
            rclpy.spin_once(n, timeout_sec=.02)
        return future.result()

    def apply(objects):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        if not wait(clients['apply'].call_async(ApplyPlanningScene.Request(scene=scene))).success:
            raise RuntimeError('PlanningScene update failed')

    def rz(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])

    try:
        for client in clients.values():
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Missing MoveIt service '+client.srv_name)
        end = time.monotonic()+10
        while len(joints) < 5:
            if time.monotonic() >= end:
                raise RuntimeError('No fresh six-axis state')
            rclpy.spin_once(n, timeout_sec=.02)
        baseline = copy.deepcopy(joints[-1])
        if len(baseline.name) != 6 or any(abs(x) > .02 for x in baseline.position):
            raise RuntimeError('Expected verified transport posture with six joints at zero')
        initial = geometry['robot_transforms']
        mount_yaw = initial['arm_base']['world_yaw'] - initial['base_link']['world_yaw']
        offset = np.array(initial['arm_base']['base_link_translation'])
        tables = [entry for entry in geometry['collisions'] if entry['path'].startswith(('/World/table1/', '/World/table2/'))]
        if len(tables) != 6:
            raise RuntimeError('Expected all six table collision bodies')
        for candidate in candidates:
            x, y, heading = candidate['map_pose_xy_yaw']
            mount = np.array([x, y, initial['base_link']['world_translation'][2]])+rz(heading)@offset
            rotation = rz(heading+mount_yaw)
            objects = []
            for index, table in enumerate(tables):
                vertices = np.array(table['world_points_m'])
                low, high = vertices.min(axis=0), vertices.max(axis=0)
                item = CollisionObject(id=f'course_reach_table_{index}', operation=CollisionObject.ADD)
                item.header.frame_id = 'base_link_arm'
                box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=(high-low).tolist())
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (rotation.T@((high+low)/2-mount)).tolist()
                pose.orientation.z = math.sin(-(heading+mount_yaw)/2)
                pose.orientation.w = math.cos(-(heading+mount_yaw)/2)
                item.primitives, item.primitive_poses = [box], [pose]
                objects.append(item)
            object_ids = [item.id for item in objects]
            apply(objects)
            request = GetPlanningScene.Request()
            request.components.components = 16  # WORLD_OBJECT_GEOMETRY
            actual = wait(clients['scene'].call_async(request)).scene.world.collision_objects
            present = {obj.id for obj in actual}
            if not set(object_ids) <= present:
                raise RuntimeError('MoveIt did not retain all table collision objects')
            record = {'name': candidate['name'], 'station_xy_yaw': [x, y, heading],
                      'collision_objects_confirmed': object_ids, 'ik_attempts': []}
            report['candidates'].append(record)
            position = candidate['wrist_target_in_arm_base_m']
            if a.finger_x is not None or a.finger_z is not None:
                position = (np.array(position) + rotation.T@np.array([
                    0. if a.finger_x is None else a.finger_x-candidate['finger_target_world_m'][0],
                    0., 0. if a.finger_z is None else a.finger_z-candidate['finger_target_world_m'][2]])).tolist()
            record['requested_wrist_position_arm_base_m'] = position
            # Desired world X=-X, Y=Y, Z=-Z; transformed into candidate arm frame.
            # All C candidates face pi; mount_yaw is independently read above.
            angle = -(heading+mount_yaw)
            seeds = [baseline, copy.deepcopy(baseline)]
            seeds[-1].position = candidate['offline_ik']['joints_rad']
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link_arm'
            request.fk_link_names = ['link_6']
            request.robot_state.joint_state = seeds[-1]
            response = wait(clients['fk'].call_async(request))
            if response.error_code.val == 1:
                fp = response.pose_stamped[0].pose.position
                record['offline_seed_real_fk_position_error_m'] = math.dist(position, [fp.x, fp.y, fp.z])
            validity = GetStateValidity.Request()
            validity.robot_state.joint_state = seeds[-1]
            validity.group_name = 'eco65_arm'
            checked = wait(clients['valid'].call_async(validity))
            record['offline_seed_collision_valid'] = checked.valid
            record['offline_seed_contacts'] = sorted({(c.contact_body_1, c.contact_body_2) for c in checked.contacts})
            solution = None
            for seed in seeds:
                request = GetPositionIK.Request()
                ik = request.ik_request
                ik.group_name, ik.ik_link_name = 'eco65_arm', 'link_6'
                ik.robot_state.joint_state = seed
                ik.avoid_collisions = True
                ik.timeout.sec = 1
                ik.pose_stamped.header.frame_id = 'base_link_arm'
                pose = ik.pose_stamped.pose
                pose.position.x, pose.position.y, pose.position.z = position
                # Rz(angle) * Ry(pi).
                pose.orientation.x = -math.sin(angle/2)
                pose.orientation.y = math.cos(angle/2)
                pose.orientation.z = 0.
                pose.orientation.w = 0.
                response = wait(clients['ik'].call_async(request), 8)
                record['ik_attempts'].append({'error_code': response.error_code.val})
                if response.error_code.val == 1:
                    solution = response.solution
                    break
            if solution is None:
                record['result'] = 'no_collision_free_ik_found'
                continue
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link_arm'
            request.fk_link_names = ['link_6']
            request.robot_state = solution
            response = wait(clients['fk'].call_async(request))
            if response.error_code.val != 1:
                raise RuntimeError('Independent target FK failed')
            fp = response.pose_stamped[0].pose.position
            record['ik_fk_position_error_m'] = math.dist(position, [fp.x, fp.y, fp.z])
            fq = response.pose_stamped[0].pose.orientation
            expected_q = [-math.sin(angle/2), math.cos(angle/2), 0., 0.]
            dot = abs(sum(u*v for u, v in zip([fq.x, fq.y, fq.z, fq.w], expected_q)))
            record['ik_fk_orientation_error_rad'] = 2*math.acos(min(1., dot))
            if record['ik_fk_position_error_m'] > .005 or record['ik_fk_orientation_error_rad'] > .01:
                raise RuntimeError('IK solution does not reach requested target position/orientation')
            request = GetMotionPlan.Request()
            plan = request.motion_plan_request
            plan.group_name = 'eco65_arm'
            plan.start_state.joint_state = baseline
            plan.allowed_planning_time = 5.
            plan.num_planning_attempts = 3
            plan.max_velocity_scaling_factor = .1
            plan.max_acceleration_scaling_factor = .1
            values = dict(zip(solution.joint_state.name, solution.joint_state.position))
            goal = Constraints(name='table_aware_grasp_goal')
            goal.joint_constraints = [JointConstraint(joint_name=name, position=values[name], tolerance_above=.001,
                tolerance_below=.001, weight=1.) for name in baseline.name]
            plan.goal_constraints = [goal]
            response = wait(clients['plan'].call_async(request), 20).motion_plan_response
            record.update(plan_error_code=response.error_code.val,
                trajectory_points=len(response.trajectory.joint_trajectory.points),
                result='planned_only' if response.error_code.val == 1 else 'planning_failed')
            if a.grasp_z is not None and a.lift_z is not None and response.error_code.val == 1:
                start_state = copy.deepcopy(solution)
                record['vertical_cycle'] = []
                for name, finger_z in [('approach', a.grasp_z), ('lift', a.lift_z)]:
                    request = GetCartesianPath.Request()
                    request.header.frame_id = 'base_link_arm'
                    request.start_state = start_state
                    request.group_name, request.link_name = 'eco65_arm', 'link_6'
                    request.max_step = .005
                    request.avoid_collisions = True
                    waypoint = Pose()
                    world_finger_z = candidate['finger_target_world_m'][2] if a.finger_z is None else a.finger_z
                    waypoint_position = np.array(position)+rotation.T@np.array([0., 0., finger_z-world_finger_z])
                    waypoint.position.x, waypoint.position.y, waypoint.position.z = waypoint_position.tolist()
                    waypoint.orientation.x, waypoint.orientation.y = -math.sin(angle/2), math.cos(angle/2)
                    waypoint.orientation.z, waypoint.orientation.w = 0., 0.
                    request.waypoints = [waypoint]
                    path = wait(clients['cartesian'].call_async(request), 15)
                    trajectory = path.solution.joint_trajectory
                    record['vertical_cycle'].append({'phase': name, 'finger_world_z_m': finger_z,
                        'fraction': path.fraction, 'error_code': path.error_code.val,
                        'trajectory_points': len(trajectory.points)})
                    if path.fraction < .999 or path.error_code.val != 1 or not trajectory.points:
                        record['result'] = 'vertical_cycle_incomplete'
                        break
                    start_state = copy.deepcopy(path.start_state)
                    start_state.joint_state.name = list(trajectory.joint_names)
                    start_state.joint_state.position = list(trajectory.points[-1].positions)
                else:
                    record['result'] = 'approach_and_lift_planned_only'
        report['command_messages'] = len(commands)
        if commands:
            raise RuntimeError('Unexpected /hand_command traffic during plan-only evaluation')
        report['status'] = 'evaluated'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if object_ids:
            try:
                apply([CollisionObject(id=name, operation=CollisionObject.REMOVE) for name in object_ids])
                report['temporary_objects_removed'] = True
            except Exception as exc:
                report['errors'].append('Cleanup: '+str(exc))
        if report['errors']:
            report['status'] = 'failed'
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        n.destroy_node()
        rclpy.shutdown()
    return 0 if report['status'] == 'evaluated' else 1


if __name__ == '__main__':
    raise SystemExit(main())
