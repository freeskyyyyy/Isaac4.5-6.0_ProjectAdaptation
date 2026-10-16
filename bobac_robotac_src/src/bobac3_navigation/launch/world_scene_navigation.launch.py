"""Opt-in USD-world localization and conservative table-aware navigation.

Does not start Isaac Sim or send any motion/goal. A must validate alignment
before using this launch for target execution. The legacy entry is unchanged.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bobac3_navigation')
    obstacles = Node(
        package='nav2_map_server', executable='map_server', name='world_obstacle_map_server',
        output='screen', parameters=[{'use_sim_time': True, 'frame_id': 'map',
                                     'yaml_filename': LaunchConfiguration('obstacle_map')}],
        remappings=[('map', '/world_obstacles')],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_world_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['world_obstacle_map_server']}],
    )
    readiness = Node(
        package='bobac3_navigation', executable='navigation_readiness.py', output='screen',
        arguments=['--timeout', LaunchConfiguration('world_readiness_timeout'),
                   '--extra-lifecycle-node', 'world_obstacle_map_server'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('world_check_readiness')),
    )

    def failure(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='World obstacle map process failed'))]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('localization_map', default_value=os.path.join(
            share, 'maps', 'world_scene_lidar.yaml')),
        DeclareLaunchArgument('obstacle_map', default_value=os.path.join(
            share, 'maps', 'world_scene.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'params', 'bobac3_navigation_world_params.yaml')),
        DeclareLaunchArgument('world_check_readiness', default_value='true'),
        DeclareLaunchArgument('world_readiness_timeout', default_value='90'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '44'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        RegisterEventHandler(OnProcessExit(target_action=obstacles, on_exit=failure)),
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=failure)),
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=failure)),
        obstacles, manager,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'bobac3_navigation.launch.py')),
            launch_arguments={'map': LaunchConfiguration('localization_map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'check_readiness': 'false', 'use_scan_relay': 'false',
                              'use_sim_time_smoother': 'true'}.items()),
        readiness,
    ])
