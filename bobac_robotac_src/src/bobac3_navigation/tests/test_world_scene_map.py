"""Offline evidence for the opt-in USD-world map. Never creates ROS nodes."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
MAPS = PACKAGE / 'maps'


class WorldSceneMapTest(unittest.TestCase):
    def load_map(self, name):
        metadata = MAPS / (name + '.yaml')
        self.assertTrue(metadata.exists(), f'Missing independent map: {metadata.name}')
        data = yaml.safe_load(metadata.read_text())
        return data, np.array(Image.open(MAPS / data['image']))

    def pixel(self, data, image, x, y):
        col = math.floor((x - data['origin'][0]) / data['resolution'])
        row = image.shape[0] - 1 - math.floor((y - data['origin'][1]) / data['resolution'])
        return image[row, col]

    def test_world_coordinates_and_scan_height_projection(self):
        full, image = self.load_map('world_scene')
        scan, scan_image = self.load_map('world_scene_lidar')
        self.assertEqual(full['origin'], [-1.0, -10.5, 0.0])
        self.assertEqual(image.shape, (320, 230))
        self.assertEqual(full['resolution'], .05)
        for x, y, expected in [(8, 1.082, 254), (5.7, 0, 0),
                               (1.2, .6, 0), (1.2, -.4, 0), (-.8, -10.3, 205)]:
            self.assertEqual(self.pixel(full, image, x, y), expected, (x, y))
        self.assertEqual(self.pixel(scan, scan_image, 1.2, .6), 254,
                         'A 0.229m scan sees below the 0.4m table underside')
        self.assertEqual(self.pixel(scan, scan_image, 1.2, -.4), 0)

    def test_north_detour_has_transit_clearance(self):
        data, image = self.load_map('world_scene')
        clearance = distance_transform_edt(image == 254) * data['resolution']
        route = [(8, 1.082), (6.6, 3), (4.8, 3), (2.35, .6)]
        for start, end in zip(route, route[1:]):
            for fraction in np.linspace(0, 1, 201):
                x, y = np.array(start) * (1-fraction) + np.array(end) * fraction
                self.assertGreaterEqual(self.pixel(data, clearance, x, y), .50)
        self.assertEqual(self.pixel(data, image, 5.7, .6), 0,
                         'The direct route crosses the vertical wall')

    def test_explicit_world_profile_uses_distinct_localization_and_planning_maps(self):
        path = PACKAGE / 'params/bobac3_navigation_world_params.yaml'
        self.assertTrue(path.exists(), 'Missing opt-in initial localization/world profile')
        params = yaml.safe_load(path.read_text())
        pose = params['amcl']['ros__parameters']['initial_pose']
        self.assertAlmostEqual(pose['x'], 7.999990895390511)
        self.assertAlmostEqual(pose['y'], 1.0820035936354835)
        self.assertAlmostEqual(pose['yaw'], -3.577751974137747e-05)
        for prefix in ['global', 'local']:
            costmap = params[prefix+'_costmap'][prefix+'_costmap']['ros__parameters']
            self.assertIn('world_static_layer', costmap['plugins'])
            self.assertEqual(costmap['world_static_layer']['map_topic'], '/world_obstacles')
        self.assertFalse(params['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
        self.assertTrue((PACKAGE / 'launch/world_scene_navigation.launch.py').exists())

    def test_generator_is_reproducible(self):
        script = PACKAGE / 'scripts/generate_world_map.py'
        self.assertTrue(script.exists(), 'Missing reproducible projection tool')
        with tempfile.TemporaryDirectory(prefix='bobac-map-test-') as temporary:
            subprocess.run([sys.executable, str(script), '--geometry', str(MAPS/'world_scene_geometry.json'),
                            '--output-dir', temporary], check=True, capture_output=True)
            for name in ['world_scene.pgm', 'world_scene.yaml', 'world_scene_lidar.pgm', 'world_scene_lidar.yaml']:
                self.assertEqual((Path(temporary)/name).read_bytes(), (MAPS/name).read_bytes())

    def test_geometry_evidence_matches_current_scene_and_asset_files(self):
        evidence = json.loads((MAPS/'world_scene_geometry.json').read_text())
        for path, key in [(evidence['source'], 'source_sha256'),
                          (evidence['course_scene'], 'course_scene_sha256'),
                          (evidence['robot_asset'], 'robot_asset_sha256')]:
            source = PACKAGE.parents[2] / path
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), evidence[key],
                             'Source changed: regenerate/verify map evidence before accepting alignment')

    def test_world_launch_wires_two_maps_and_checks_tenth_lifecycle_node(self):
        from ament_index_python.packages import get_package_share_directory
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.utilities import perform_substitutions
        from launch_ros.actions import Node
        spec = importlib.util.spec_from_file_location('world_launch', PACKAGE/'launch/world_scene_navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
        include = next(action for action in description.entities if isinstance(action, IncludeLaunchDescription))
        arguments = dict(include.launch_arguments)
        share = Path(get_package_share_directory('bobac3_navigation'))
        self.assertEqual(perform_substitutions(context, [arguments['map']]), str(share/'maps/world_scene_lidar.yaml'))
        self.assertEqual(perform_substitutions(context, [arguments['params_file']]), str(share/'params/bobac3_navigation_world_params.yaml'))
        self.assertEqual(arguments['check_readiness'], 'false')
        nodes = [action for action in description.entities if isinstance(action, Node)]
        self.assertEqual([node.node_executable for node in nodes].count('map_server'), 1)
        readiness = next(node for node in nodes if node.node_executable == 'navigation_readiness.py')
        self.assertIn('world_obstacle_map_server',
                      [perform_substitutions(context, item) for item in readiness.cmd])

    def test_old_maps_preserved(self):
        expected = {
            'map.png': '97ec031e58e95bf35a0fa51f6ec83e832289bf4130d7b50497317f0809704e28',
            'map.yaml': '8c80cb579f8c9e55c501946dd7ef1d7095d73b44734e684a92856538d91a5bf6',
            'map_older.png': '2116944cd7d93db60f85f410343360543739f4efe372c6d3630339ba2812d570',
            'map_older.yaml': '759c6376081c9fac59a74b84f77572139063ad63e40a6030648ee40c03a83009',
        }
        for name, digest in expected.items():
            self.assertEqual(hashlib.sha256((MAPS/name).read_bytes()).hexdigest(), digest)

    def test_offline_arm_model_matches_recorded_zero_joint_fk(self):
        script = PACKAGE / 'scripts/analyze_world_candidates.py'
        self.assertTrue(script.exists(), 'Missing repeatable candidate reach assessment')
        spec = importlib.util.spec_from_file_location('candidates', script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        urdf = PACKAGE.parent / 'eco65b_moveit_description/urdf/ECO65-B_base_link_arm.urdf'
        model = module.Kinematics(urdf)
        actual = model.fk(np.zeros(6))
        np.testing.assert_allclose(actual[:3, 3], [-.086, .02062, .7725], atol=1e-5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
