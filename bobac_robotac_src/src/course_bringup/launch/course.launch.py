"""Start the course only after simulator topics and transforms are healthy."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def include(package, name, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', name)),
            launch_arguments=(arguments or {}).items(),
        )

    readiness = Node(package='course_bringup', executable='course_readiness', output='screen',
                     arguments=['--timeout', LaunchConfiguration('readiness_timeout')])
    vision_share = get_package_share_directory('yoloe_grasp')
    vision = Node(
        package='yoloe_grasp', executable='yoloe_grasp_node', output='screen',
        prefix=[os.environ.get('VISION_PYTHON', '/opt/course-vision/bin/python')],
        parameters=[os.path.join(vision_share, 'config', 'pen_grasp_params.yaml'),
                    {'use_sim_time': True, 'text_model_cache_dir': LaunchConfiguration('vision_cache_dir')}],
    )
    motion = Node(
        package='eco65b_moveit_grasp', executable='grasp_moveit_node', output='screen',
        parameters=[{'use_sim_time': True, 'joint_state_topic': '/arm/joint_states'}],
        remappings=[('joint_states', '/arm/joint_state_deal')],
    )
    stack = [
        include('bobac3_navigation', 'bobac3_navigation.launch.py',
                {'use_sim_time': 'true', 'rviz': LaunchConfiguration('rviz')}),
        include('eco65b_moveit_config', 'move_group.launch.py',
                {'use_sim_time': 'true', 'publish_robot_tf': 'false'}),
        motion, vision,
        Node(package='grasp_control', executable='gripper_control_node', output='screen',
             parameters=[{'use_sim_time': True}]),
    ]

    def checked(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulator readiness failed; see diagnostics above'))]
        return stack

    return LaunchDescription([
        DeclareLaunchArgument('readiness_timeout', default_value='60'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('vision_cache_dir', default_value=EnvironmentVariable(
            'COURSE_VISION_CACHE', default_value='/workspace/course/.runtime/arm64/vision/weights')),
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=checked)),
        readiness,
    ])
