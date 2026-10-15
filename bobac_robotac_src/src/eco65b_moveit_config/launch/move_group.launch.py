from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    desc_pkg = get_package_share_directory('eco65b_moveit_description')
    urdf_path = f"{desc_pkg}/urdf/ECO65-B_base_link_arm.urdf"

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    moveit_pkg = get_package_share_directory('eco65b_moveit_config')
    srdf_path = f"{moveit_pkg}/config/eco65b.srdf"
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    def config(name):
        with open(f'{moveit_pkg}/config/{name}', 'r') as stream:
            data = yaml.safe_load(stream)
        return data.get('/**', {}).get('ros__parameters', data)

    pipeline = config('planning_pipelines.yaml')
    pipeline['ompl'].update(config('ompl_planning.yaml'))
    moveit_parameters = [
        {'use_sim_time': use_sim_time,
         'robot_description': robot_description,
         'robot_description_semantic': robot_description_semantic,
         'robot_description_kinematics': config('kinematics.yaml'),
         'robot_description_planning': config('joint_limits.yaml')},
        pipeline,
    ]

    # Filter joint_states so MoveIt does not see non-arm joints (wheels/grippers)
    joint_filter = Node(
        package='joint_state_filter',
        executable='joint_state_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'input_topic': '/arm/joint_states',
             'output_topic': '/arm/joint_state_deal',
             'joints': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']},
        ],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=IfCondition(LaunchConfiguration('publish_robot_tf')),
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        remappings=[('joint_states', '/arm/joint_state_deal')],
        parameters=moveit_parameters,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_robot_tf', default_value='true',
                              description='Disable when Isaac publishes the complete arm link TF'),
        joint_filter,
        rsp,
        move_group,
    ])
