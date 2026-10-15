"""Offline regression checks. Source the independent install before running."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml


SHARE = Path(get_package_share_directory('bobac3_navigation'))
spec = importlib.util.spec_from_file_location('navigation_launch', SHARE / 'launch/bobac3_navigation.launch.py')
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


class NavigationConfigTest(unittest.TestCase):
    def launch(self, overrides=None):
        description = launch_module.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
        context.launch_configurations.update(overrides or {})
        return description, context

    def test_all_presets_and_relay_selection(self):
        for preset in SHARE.glob('params/bobac3_navigation_params*.yaml'):
            for relay in ['false', 'true']:
                with self.subTest(preset=preset.name, relay=relay):
                    description, context = self.launch({'params_file': str(preset), 'use_scan_relay': relay})
                    server = next(action for action in description.entities if isinstance(action, Node)
                                  and action.node_executable == 'bt_navigator')
                    # Evaluate only the actual ParameterFile, without executing the launch.
                    parameter_file = server._Node__parameters[0]
                    try:
                        data = yaml.safe_load(parameter_file.evaluate(context).read_text())
                        bt = Path(data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
                        self.assertEqual(bt, SHARE / 'behavior_trees/navigate_to_pose_no_spin.xml')
                        self.assertTrue(bt.is_file())
                        self.assertEqual(data['map_server']['ros__parameters']['yaml_filename'], str(SHARE / 'maps/map.yaml'))
                        scan = '/laser_scan_fuse_best_effort' if relay == 'true' else '/laser_scan_fuse'
                        self.assertEqual(data['amcl']['ros__parameters']['scan_topic'], scan)
                        local = data['local_costmap']['local_costmap']['ros__parameters']
                        self.assertEqual(local['front_rplidar_obstacle_layer']['scan']['topic'], scan)
                        self.assertNotIn('reliability', local['front_rplidar_obstacle_layer']['scan'])
                        behavior = data['behavior_server']['ros__parameters']
                        self.assertEqual((behavior['local_frame'], behavior['global_frame']), ('odom', 'map'))
                        self.assertIn('local_costmap_topic', behavior)
                        self.assertIn('global_footprint_topic', behavior)
                        for node in ['controller_server', 'velocity_smoother', 'behavior_server']:
                            self.assertIs(data[node]['ros__parameters']['enable_stamped_cmd_vel'], False)
                        def times(mapping):
                            for key, value in mapping.items():
                                if key == 'use_sim_time':
                                    self.assertIs(value, True)
                                if isinstance(value, dict):
                                    times(value)
                        times(data)
                    finally:
                        parameter_file.cleanup()

    def test_custom_map_is_injected(self):
        description, context = self.launch({'map': str(SHARE / 'maps/map_older.yaml')})
        server = next(action for action in description.entities if isinstance(action, Node)
                      and action.node_executable == 'map_server')
        parameter_file = server._Node__parameters[0]
        try:
            data = yaml.safe_load(parameter_file.evaluate(context).read_text())
            self.assertEqual(data['map_server']['ros__parameters']['yaml_filename'], str(SHARE / 'maps/map_older.yaml'))
        finally:
            parameter_file.cleanup()

    def test_required_plugins_exist_in_installed_registry(self):
        registry = set()
        for package in ['nav2_amcl', 'nav2_controller', 'nav2_mppi_controller', 'nav2_smac_planner',
                        'nav2_costmap_2d', 'nav2_smoother', 'nav2_behaviors', 'nav2_waypoint_follower']:
            for xml in Path(get_package_share_directory(package)).glob('*.xml'):
                for plugin in ET.parse(xml).iter('class'):
                    registry.add(plugin.attrib.get('name', plugin.attrib['type']))
        for preset in SHARE.glob('params/bobac3_navigation_params*.yaml'):
            def plugins(mapping):
                for key, value in mapping.items():
                    if key in ['plugin', 'robot_model_type']:
                        self.assertIn(value, registry, f'{preset.name}: {value}')
                    if isinstance(value, dict):
                        plugins(value)
            plugins(yaml.safe_load(preset.read_text()))

    def test_installed_maps_and_programs(self):
        for metadata in SHARE.glob('maps/*.yaml'):
            image = yaml.safe_load(metadata.read_text())['image']
            self.assertTrue((metadata.parent / image).is_file())
        prefix = SHARE.parents[1]
        for program in ['laser_qos_relay.py', 'navigation_readiness.py']:
            self.assertTrue((prefix / 'lib/bobac3_navigation' / program).is_file())

    def test_course_node_list_and_command_chain(self):
        description, context = self.launch()
        nodes = [action for action in description.entities if isinstance(action, Node)]
        executables = [node.node_executable for node in nodes]
        for executable in launch_module.LOCALIZATION_NODES + launch_module.NAVIGATION_NODES:
            self.assertIn(executable, executables)
        for extra in ['collision_monitor', 'route_server', 'opennav_docking']:
            self.assertNotIn(extra, executables)
        for node in nodes:
            executable = node.node_executable
            if executable in ['controller_server', 'behavior_server', 'velocity_smoother']:
                node._perform_substitutions(context)
                self.assertIn(('cmd_vel', 'cmd_vel_nav'), node.expanded_remapping_rules)
                if executable == 'velocity_smoother':
                    self.assertIn(('cmd_vel_smoothed', 'cmd_vel'), node.expanded_remapping_rules)


if __name__ == '__main__':
    unittest.main(verbosity=2)
