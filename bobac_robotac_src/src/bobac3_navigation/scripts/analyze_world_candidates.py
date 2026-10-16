#!/usr/bin/env python3
"""Offline geometric/URDF IK evidence only. No ROS, MoveIt, Kit, or commands.

Numerical IK checks a nominal downward grasp with the existing 0.166m tool
offset. It does NOT check collision, manipulability, camera view, uncertainty,
or the executed trajectory. A must use real MoveIt and scene evidence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class Kinematics:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.origins, low, high = [], [], []
        for index in range(1, 7):
            joint = root.find(f"joint[@name='joint_{index}']")
            if joint.find('axis').get('xyz') != '0 0 1':
                raise ValueError('Expected revolute Z-axis URDF joint')
            origin = joint.find('origin')
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_euler('xyz', [float(x) for x in origin.get('rpy').split()]).as_matrix()
            matrix[:3, 3] = [float(x) for x in origin.get('xyz').split()]
            self.origins.append(matrix)
            low.append(float(joint.find('limit').get('lower')))
            high.append(float(joint.find('limit').get('upper')))
        self.limits = np.array(low), np.array(high)

    def fk(self, joints):
        matrix = np.eye(4)
        for origin, angle in zip(self.origins, joints):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec([0, 0, angle]).as_matrix()
            matrix = matrix @ origin @ rotation
        return matrix

    def solve(self, target):
        def residual(joints):
            current = self.fk(joints)
            return np.concatenate([current[:3, 3] - target[:3, 3],
                                   Rotation.from_matrix(target[:3, :3].T @ current[:3, :3]).as_rotvec()])
        random = np.random.default_rng(44)
        seeds = [np.zeros(6)] + [random.uniform(*self.limits) for _ in range(15)]
        best = None
        for seed in seeds:
            solution = least_squares(residual, seed, bounds=self.limits, max_nfev=160)
            error = residual(solution.x)
            result = {'joints_rad': solution.x.tolist(),
                      'position_error_m': float(np.linalg.norm(error[:3])),
                      'orientation_error_rad': float(np.linalg.norm(error[3:]))}
            result['kinematic_solution_found'] = result['position_error_m'] < .001 and result['orientation_error_rad'] < .01
            if best is None or np.linalg.norm(error) < best[0]:
                best = np.linalg.norm(error), result
            if result['kinematic_solution_found']:
                break
        return best[1]


def analyze(geometry, model, finger_z):
    transform = geometry['robot_transforms']['arm_base']
    base_z = geometry['robot_transforms']['base_link']['world_translation'][2]
    offset = np.array(transform['base_link_translation'])
    # Nominal planar base, retaining measured mounting translation and yaw.
    mount_yaw = transform['world_yaw'] - geometry['robot_transforms']['base_link']['world_yaw']
    finger = np.array([1.4, .6, finger_z])
    # Match the current grasp node: local Y = marker long axis, local Z = down.
    desired_world_rotation = np.diag([-1., 1., -1.])
    records = []
    for name, x, y in [('staging', 2.35, .6), ('reach_outer', 2.15, .6), ('reach_close', 2.1, .6), ('tight_diagnostic_only', 2.02, .6)]:
        yaw = math.pi
        base_rotation = Rotation.from_euler('z', yaw).as_matrix()
        mount_world = np.array([x, y, base_z]) + base_rotation @ offset
        arm_rotation = Rotation.from_euler('z', yaw + mount_yaw).as_matrix()
        wrist_world = finger - desired_world_rotation[:, 2] * .166
        target = np.eye(4)
        target[:3, :3] = arm_rotation.T @ desired_world_rotation
        target[:3, 3] = arm_rotation.T @ (wrist_world - mount_world)
        result = model.solve(target)
        records.append({'name': name, 'map_pose_xy_yaw': [x, y, yaw],
                        'navigation_candidate': name != 'tight_diagnostic_only',
                        'grasp_execution_accepted': False,
                        'quaternion_xyzw': [0., 0., 1., 0.],
                        'arm_mount_world_m': mount_world.tolist(),
                        'finger_target_world_m': finger.tolist(),
                        'finger_horizontal_reach_m': float(np.linalg.norm((finger-mount_world)[:2])),
                        'table_edge_clearance_with_candidate_footprint_m': x - .26 - 1.7,
                        'wrist_target_in_arm_base_m': target[:3, 3].tolist(),
                        'offline_ik': result})
    initial = geometry['robot_transforms']['base_link']
    legacy_pose = np.array([-3.783, 2.137])
    angle = initial['world_yaw'] - (-1.5708)
    legacy_rotation = Rotation.from_euler('z', angle).as_matrix()[:2, :2]
    translation = np.array(initial['world_translation'][:2]) - legacy_rotation @ legacy_pose
    legacy_target = legacy_rotation.T @ (np.array([1.4, .6]) - translation)
    return {'scope': 'Static candidate geometry and nominal URDF IK only; collision/runtime pending',
            'world_from_map_xy_yaw': [0., 0., 0.],
            'legacy_spawn_anchor_only_NOT_registration': {
                'world_from_legacy_map_xy_yaw': [*translation.tolist(), angle],
                'object_in_legacy_map_xy': legacy_target.tolist(),
                'note': 'Single-pose anchoring is not map registration; object lies outside old map bounds'},
            'initial_map_pose': geometry['robot_transforms']['base_link'],
            'transit_via_xy': [[8., 1.082], [6.6, 3.], [4.8, 3.], [2.35, .6]],
            'candidates': records,
            'limitations': ['No collision or self-collision checking', 'No real MoveIt planning',
                            'Nominal planar arm mounting rotation', 'Finger z and tool offset are assumptions',
                            'No camera visibility or grasp/lift evidence',
                            'Unsuccessful numerical search is not proof of unreachability']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--finger-z', type=float, default=.6127375614254722)
    parser.add_argument('--output', type=Path, required=True)
    opts = parser.parse_args()
    geometry = json.loads(opts.geometry.read_text())
    result = analyze(geometry, Kinematics(opts.urdf), opts.finger_z)
    result['urdf_sha256'] = hashlib.sha256(opts.urdf.read_bytes()).hexdigest()
    result['geometry_sha256'] = hashlib.sha256(opts.geometry.read_bytes()).hexdigest()
    opts.output.write_text(json.dumps(result, indent=2) + '\n')
    for candidate in result['candidates']:
        print(candidate['name'], candidate['map_pose_xy_yaw'], candidate['offline_ik'])


if __name__ == '__main__':
    main()
