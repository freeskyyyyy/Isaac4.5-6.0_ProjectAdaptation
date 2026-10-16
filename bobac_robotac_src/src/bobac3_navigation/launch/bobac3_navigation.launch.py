## Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

"""Course Nav2 nodes and interfaces, independent of upstream bringup additions."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother',
]


def generate_launch_description():
    share = get_package_share_directory('bobac3_navigation')
    sim_time = LaunchConfiguration('use_sim_time')
    sim_smoother = LaunchConfiguration('use_sim_time_smoother')
    scan_relay = LaunchConfiguration('use_scan_relay')
    scan_topic = PythonExpression([
        "'/laser_scan_fuse_best_effort' if '", scan_relay,
        "'.lower() == 'true' else '/laser_scan_fuse'",
    ])
    # Full paths also INSERT missing parameters; leaf-only rewrites do not.
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'use_sim_time': sim_time,
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(
                share, 'behavior_trees', 'navigate_to_pose_no_spin.xml'),
            'map_server.ros__parameters.yaml_filename': LaunchConfiguration('map'),
            'amcl.ros__parameters.scan_topic': scan_topic,
            'local_costmap.local_costmap.ros__parameters.front_rplidar_obstacle_layer.scan.topic': scan_topic,
        },
        convert_types=True,
    ), allow_substs=True)
    common = [params, {'use_sim_time': ParameterValue(sim_time, value_type=bool)}]
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def server(package, executable, remaps=None):
        return Node(package=package, executable=executable, name=executable,
                    output='screen', parameters=common,
                    remappings=tf_remaps + (remaps or []))

    servers = [
        server('nav2_map_server', 'map_server'),
        server('nav2_amcl', 'amcl'),
        server('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_smoother', 'smoother_server'),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower'),
        Node(package=PythonExpression(["'bobac3_navigation' if '", sim_smoother,
                                       "'.lower() == 'true' else 'nav2_velocity_smoother'"]),
             executable=PythonExpression(["'sim_time_velocity_smoother' if '", sim_smoother,
                                          "'.lower() == 'true' else 'velocity_smoother'"]),
             name='velocity_smoother', output='screen', parameters=common,
             remappings=tf_remaps + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ]
    managers = [Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_' + group, output='screen',
        parameters=[{'use_sim_time': ParameterValue(sim_time, value_type=bool),
                     'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                     'node_names': names}],
    ) for group, names in [('localization', LOCALIZATION_NODES), ('navigation', NAVIGATION_NODES)]]
    relay = Node(
        package='bobac3_navigation', executable='laser_qos_relay.py',
        name='laser_qos_relay', output='screen',
        parameters=[{'use_sim_time': ParameterValue(sim_time, value_type=bool)}],
        condition=IfCondition(scan_relay),
    )
    readiness = Node(
        package='bobac3_navigation', executable='navigation_readiness.py',
        output='screen', arguments=['--timeout', LaunchConfiguration('readiness_timeout'),
                                    '--scan-topic', scan_topic],
        parameters=[{'use_sim_time': ParameterValue(sim_time, value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('check_readiness')),
    )

    def failure(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=
                'Navigation process failed; inspect Nav2 and navigation_readiness diagnostics'))]
        return []

    actions = [
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'params', 'bobac3_navigation_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'True'],
                              description='Course navigation requires the simulator clock'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time_smoother', default_value='false', choices=['true', 'false'],
                              description='World opt-in simulation-clock CLOSED_LOOP adapter'),
        DeclareLaunchArgument('use_scan_relay', default_value='false',
                              description='Relay the raw scan to the existing best_effort topic'),
        DeclareLaunchArgument('check_readiness', default_value='true',
                              description='Fail launch if navigation is not ready before timeout'),
        DeclareLaunchArgument('readiness_timeout', default_value='90'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz2', 'bobac3_navigation.rviz')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '44'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ]
    for process in servers + managers + [relay, readiness]:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=failure)))
    actions.extend([relay] + servers + managers + [readiness])
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': ParameterValue(sim_time, value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))
    return LaunchDescription(actions)
