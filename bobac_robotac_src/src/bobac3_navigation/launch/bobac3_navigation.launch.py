## Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import conditions
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share_path = get_package_share_directory("bobac3_navigation")
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    nav2_localization_launch = os.path.join(nav2_bringup_launch_dir, "localization_launch.py")
    nav2_navigation_launch = os.path.join(nav2_bringup_launch_dir, "navigation_launch.py")

    # 动态获取behavior tree XML文件路径
    bt_xml_path = os.path.join(share_path, "behavior_trees", "navigate_to_pose_no_spin.xml")

    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(share_path, "maps", "map.yaml"),
        description="Full path to map file to load",
    )

    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_path, "params", "bobac3_navigation_params.yaml"),
        description="Full path to param file to load",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Omniverse Isaac Sim) clock if true",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Launch RViz2 if true",
    )

    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(share_path, "rviz2", "bobac3_navigation.rviz"),
        description="Full path to RViz2 config file",
    )

    map_dir = LaunchConfiguration("map")
    param_dir = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # 使用RewrittenYaml动态替换bt_xml路径
    configured_params = RewrittenYaml(
        source_file=param_dir,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path
        },
        convert_types=True
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_localization_launch]),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "autostart": "True",
            "use_composition": "False",
            "map": map_dir,
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_navigation_launch]),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "autostart": "True",
            "use_composition": "False",
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
        }.items(),
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=conditions.IfCondition(rviz),
    )
    return LaunchDescription(
        [
            declare_map_arg,
            declare_params_file_arg,
            declare_use_sim_time_arg,
            declare_rviz_arg,
            declare_rviz_config_arg,
            localization,
            navigation,
            rviz2,
        ]
    )
