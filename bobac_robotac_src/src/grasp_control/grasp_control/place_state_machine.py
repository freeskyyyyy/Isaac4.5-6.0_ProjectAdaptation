#!/usr/bin/env python3
"""
放置状态机节点 - 完全重写版本
整体流程：
1. 移动到观测位置 [0.0, 1.0, -1.19, -0.23, 1.52, 0.0]
2. 调用 yoloe_grasp_node 服务识别 'open light blue glass box'
3. 获取当前 link_6 姿态，计算 Z 轴旋转 90 度后的目标姿态
4. 移动到预放置位置（容器位置 + 旋转后姿态的 Z 轴负方向 0.3m）
5. 使用笛卡尔规划平移到最终放置位置
6. 打开夹爪到 20%
7. 等待 2.5 秒
8. 返回预放置位置
9. 关闭夹爪
10. 返回零位 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Vector3Stamped
from eco65b_grasp_interfaces.action import DetectObject, GraspToPoint, GripperControl
from enum import Enum
import time
import math
import copy
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class PlaceState(Enum):
    """放置状态枚举"""
    IDLE = 0
    MOVE_TO_OBSERVATION = 1
    DETECTING_CONTAINER = 2
    ROTATE_LINK6 = 3
    MOVE_TO_PRE_PLACE = 4
    MOVE_TO_PLACE = 5
    OPEN_GRIPPER = 6
    WAIT = 7
    RETURN_TO_PRE_PLACE = 8
    CLOSE_GRIPPER = 9
    RETURN_TO_HOME = 10
    DONE = 11
    ERROR = 12


class PlaceStateMachine(Node):
    def __init__(self):
        super().__init__('place_state_machine')

        # 参数声明
        self.declare_parameter('observation_pose', [0.0, 1.0, -1.19, -0.23, 1.52, 0.0])
        self.declare_parameter('home_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('target_classes', ['open light blue glass box'])
        self.declare_parameter('confidence_threshold', 0.01)
        self.declare_parameter('finger_tip_offset', 0.03)
        self.declare_parameter('pre_place_offset', 0.25)
        self.declare_parameter('gripper_open_position', 10.0)
        self.declare_parameter('gripper_close_position', 0.0)
        self.declare_parameter('wait_time_after_release', 2.5)
        self.declare_parameter('link6_rotation_degrees', 0.0)

        # 获取参数
        self.observation_pose = self.get_parameter('observation_pose').value
        self.home_pose = self.get_parameter('home_pose').value
        self.joint_names = self.get_parameter('joint_names').value
        self.target_classes = self.get_parameter('target_classes').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.finger_tip_offset = self.get_parameter('finger_tip_offset').value
        self.pre_place_offset = self.get_parameter('pre_place_offset').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.wait_time_after_release = self.get_parameter('wait_time_after_release').value
        self.link6_rotation_degrees = self.get_parameter('link6_rotation_degrees').value

        # 发布器：用于发送关节命令
        self.hand_cmd_pub = self.create_publisher(JointState, '/hand_command', 10)

        # 订阅器：订阅当前关节状态
        self.current_joint_state = None
        self.declare_parameter('joint_state_topic', '/arm/joint_states')
        self.create_subscription(JointState, self.get_parameter('joint_state_topic').value,
                                 self._joint_state_callback, qos_profile_sensor_data)

        # TF2 监听器
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Clients
        self.detect_client = ActionClient(self, DetectObject, '/detect_object')
        self.grasp_client = ActionClient(self, GraspToPoint, '/grasp_to_point')
        self.gripper_client = ActionClient(self, GripperControl, '/gripper_control')

        # 状态变量
        self.current_state = PlaceState.IDLE
        self.detected_container = None
        self.rotated_rotation_matrix = None
        self.pre_place_position = None
        self.final_place_position = None

        self.get_logger().info('放置状态机节点已启动')
        self.get_logger().info(f'目标容器: {self.target_classes}')
        self.get_logger().info(f'预放置偏移: {self.pre_place_offset}m')
        self.get_logger().info(f'link6 旋转角度: {self.link6_rotation_degrees}度')

    def _joint_state_callback(self, msg):
        """关节状态回调 - 只提取机械臂的6个关节"""
        arm_positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                arm_positions.append(msg.position[idx])
            else:
                return

        if len(arm_positions) == 6:
            arm_state = JointState()
            arm_state.header = msg.header
            arm_state.name = self.joint_names
            arm_state.position = arm_positions
            self.current_joint_state = arm_state

    def wait_for_motion_complete(self, wait_time=2.0):
        """等待机械臂运动完成并稳定"""
        self.get_logger().info(f'等待机械臂稳定 {wait_time}秒...')
        time.sleep(wait_time)
        # 在等待期间持续 spin 以接收最新的关节状态和 TF
        for _ in range(int(wait_time * 10)):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('机械臂已稳定')

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi] 范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_shortest_rotation(self, current_angle, target_angle):
        """找到从当前角度到目标角度的最短旋转路径"""
        current = self.normalize_angle(current_angle)
        target = self.normalize_angle(target_angle)
        diff = target - current
        diff = self.normalize_angle(diff)
        shortest_target = current + diff
        return shortest_target

    def get_link6_rotation_matrix(self):
        """获取 link6 相对于 base_link 的旋转矩阵"""
        try:
            self.get_logger().info('等待 TF 变换 base_link_arm -> link_6...')
            max_wait_time = 10.0
            start_time = time.time()

            while (time.time() - start_time) < max_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)

                try:
                    if self.tf_buffer.can_transform(
                        'base_link_arm',
                        'link_6',
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.1)
                    ):
                        transform = self.tf_buffer.lookup_transform(
                            'base_link_arm',
                            'link_6',
                            rclpy.time.Time()
                        )

                        q = transform.transform.rotation
                        qx, qy, qz, qw = q.x, q.y, q.z, q.w

                        R = np.array([
                            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
                        ])

                        self.get_logger().info(f'✓ 成功获取 link_6 旋转矩阵')
                        self.get_logger().info(f'  四元数: [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]')
                        return R

                except Exception:
                    pass

            self.get_logger().error(f'等待 TF 变换超时 ({max_wait_time}s)')
            return None

        except Exception as e:
            self.get_logger().error(f'获取 link_6 姿态失败: {str(e)}')
            return None

    def run(self):
        """执行完整的放置流程"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始执行放置流程')
        self.get_logger().info('=' * 60)

        # 步骤1: 移动到观测位置
        if not self.move_to_observation_pose():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤2: 检测目标容器
        if not self.detect_container():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤3: 获取 link_6 姿态并计算旋转 90 度后的姿态
        if not self.calculate_rotated_pose():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤4: 移动到预放置位置
        if not self.move_to_pre_place():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤5: 使用笛卡尔规划平移到最终放置位置
        if not self.move_to_place():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤6: 打开夹爪
        if not self.open_gripper():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤7: 等待
        if not self.wait_after_release():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤8: 返回预放置位置
        if not self.return_to_pre_place():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤9: 关闭夹爪
        if not self.close_gripper():
            self.current_state = PlaceState.ERROR
            return False

        # 步骤10: 返回零位
        if not self.return_to_home():
            self.current_state = PlaceState.ERROR
            return False

        self.current_state = PlaceState.DONE
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ 放置流程完成！')
        self.get_logger().info('=' * 60)
        return True

    def move_to_observation_pose(self):
        """步骤1: 移动到观测位置"""
        self.current_state = PlaceState.MOVE_TO_OBSERVATION
        self.get_logger().info('[步骤1] 移动到观测位置...')
        self.get_logger().info(f'  目标关节角度: {self.observation_pose}')

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.observation_pose
        self.hand_cmd_pub.publish(msg)

        # 等待机械臂到达观测位置
        self.get_logger().info('等待机械臂到达观测位置...')
        max_wait_time = 20.0
        start_time = time.time()
        position_reached = False

        while (time.time() - start_time) < max_wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_joint_state is not None:
                if len(self.current_joint_state.position) >= len(self.observation_pose):
                    errors = [abs(self.current_joint_state.position[i] - self.observation_pose[i])
                             for i in range(len(self.observation_pose))]
                    max_error = max(errors)

                    if max_error < 0.1:
                        position_reached = True
                        self.get_logger().info(f'✓ 机械臂已到达观测位置（最大误差: {max_error:.4f} rad）')
                        break

            time.sleep(0.1)

        if not position_reached:
            self.get_logger().warn('机械臂可能未完全到达观测位置，继续执行...')

        # 额外等待确保稳定
        self.wait_for_motion_complete(2.0)

        self.get_logger().info('✓ 到达观测位置')
        return True

    def detect_container(self):
        """步骤2: 检测目标容器"""
        self.current_state = PlaceState.DETECTING_CONTAINER
        self.get_logger().info('[步骤2] 开始检测目标容器...')

        if not self.detect_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('检测服务不可用: /detect_object')
            return False

        goal = DetectObject.Goal()
        goal.target_classes = self.target_classes
        goal.confidence_threshold = self.confidence_threshold
        goal.timeout = 30.0

        self.get_logger().info(f'发送检测请求: classes={goal.target_classes}, conf={goal.confidence_threshold}')

        send_goal_future = self.detect_client.send_goal_async(
            goal,
            feedback_callback=self.detect_feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('检测请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'检测失败: {result.message}')
            return False

        self.detected_container = {
            'class': result.detected_class,
            'position': result.object_position,
            'normal': result.normal,
            'long_axis': result.long_axis,
            'confidence': result.confidence
        }

        self.get_logger().info(f'✓ 检测成功: {result.detected_class} (置信度: {result.confidence:.3f})')
        self.get_logger().info(f'  位置: ({result.object_position.point.x:.3f}, '
                              f'{result.object_position.point.y:.3f}, '
                              f'{result.object_position.point.z:.3f})')
        return True

    def detect_feedback_callback(self, feedback_msg):
        """检测反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'  检测状态: {feedback.state}, 尝试次数: {feedback.detection_attempts}')

    def calculate_rotated_pose(self):
        """步骤3: 获取当前 link_6 姿态并计算 Z 轴旋转 90 度后的目标姿态"""
        self.current_state = PlaceState.ROTATE_LINK6
        self.get_logger().info('[步骤3] 获取 link_6 姿态并计算旋转后的姿态...')

        # 获取当前 link_6 的旋转矩阵
        R_current = self.get_link6_rotation_matrix()

        if R_current is None:
            self.get_logger().error('未能获取 link_6 姿态')
            return False

        # 计算绕 Z 轴旋转的旋转矩阵
        angle_rad = math.radians(self.link6_rotation_degrees)
        Rz = np.array([
            [math.cos(angle_rad), -math.sin(angle_rad), 0],
            [math.sin(angle_rad), math.cos(angle_rad), 0],
            [0, 0, 1]
        ])

        # 计算旋转后的姿态：R_rotated = R_current @ Rz
        self.rotated_rotation_matrix = R_current @ Rz

        self.get_logger().info(f'✓ 计算完成，link_6 将绕 Z 轴旋转 {self.link6_rotation_degrees}度')
        self.get_logger().info(f'  旋转后的旋转矩阵:')
        self.get_logger().info(f'    [{self.rotated_rotation_matrix[0,0]:.3f}, {self.rotated_rotation_matrix[0,1]:.3f}, {self.rotated_rotation_matrix[0,2]:.3f}]')
        self.get_logger().info(f'    [{self.rotated_rotation_matrix[1,0]:.3f}, {self.rotated_rotation_matrix[1,1]:.3f}, {self.rotated_rotation_matrix[1,2]:.3f}]')
        self.get_logger().info(f'    [{self.rotated_rotation_matrix[2,0]:.3f}, {self.rotated_rotation_matrix[2,1]:.3f}, {self.rotated_rotation_matrix[2,2]:.3f}]')

        return True

    def move_to_pre_place(self):
        """步骤4: 移动到预放置位置（容器位置 + 旋转后姿态的 Z 轴负方向 0.3m）"""
        self.current_state = PlaceState.MOVE_TO_PRE_PLACE
        self.get_logger().info('[步骤4] 移动到预放置位置...')

        if self.detected_container is None:
            self.get_logger().error('没有检测结果')
            return False

        if self.rotated_rotation_matrix is None:
            self.get_logger().error('没有旋转后的姿态')
            return False

        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 提取旋转后姿态的 Z 轴方向（接近方向）
        z_axis = np.array([0.0, 0.0, 1.0])
        approach_direction = self.rotated_rotation_matrix @ z_axis

        # 提取旋转后姿态的 X 轴方向（短轴）
        x_axis = np.array([1.0, 0.0, 0.0])
        short_axis_direction = self.rotated_rotation_matrix @ x_axis

        # 计算预放置位置：容器位置 + finger_tip_offset - pre_place_offset（沿 Z 轴负方向）
        self.pre_place_position = copy.deepcopy(self.detected_container['position'])
        total_offset = self.finger_tip_offset + self.pre_place_offset
        self.pre_place_position.point.x -= approach_direction[0] * total_offset
        self.pre_place_position.point.y -= approach_direction[1] * total_offset
        self.pre_place_position.point.z -= approach_direction[2] * total_offset

        self.get_logger().info(f'  容器位置: ({self.detected_container["position"].point.x:.3f}, '
                              f'{self.detected_container["position"].point.y:.3f}, '
                              f'{self.detected_container["position"].point.z:.3f})')
        self.get_logger().info(f'  接近方向: ({approach_direction[0]:.3f}, '
                              f'{approach_direction[1]:.3f}, '
                              f'{approach_direction[2]:.3f})')
        self.get_logger().info(f'  预放置位置: ({self.pre_place_position.point.x:.3f}, '
                              f'{self.pre_place_position.point.y:.3f}, '
                              f'{self.pre_place_position.point.z:.3f})')
        self.get_logger().info(f'  偏移量: {total_offset:.3f}m (沿 Z 轴负方向)')

        # 创建姿态向量
        approach_normal = Vector3Stamped()
        approach_normal.header.frame_id = 'base_link_arm'
        approach_normal.header.stamp = self.get_clock().now().to_msg()
        approach_normal.vector.x = approach_direction[0]
        approach_normal.vector.y = approach_direction[1]
        approach_normal.vector.z = approach_direction[2]

        short_axis = Vector3Stamped()
        short_axis.header.frame_id = 'base_link_arm'
        short_axis.header.stamp = self.get_clock().now().to_msg()
        short_axis.vector.x = short_axis_direction[0]
        short_axis.vector.y = short_axis_direction[1]
        short_axis.vector.z = short_axis_direction[2]

        # 保存姿态向量供后续使用
        self.approach_normal = approach_normal
        self.short_axis = short_axis

        # 发送移动请求
        goal = GraspToPoint.Goal()
        goal.target = self.pre_place_position
        goal.normal = approach_normal
        goal.short_axis = short_axis
        goal.label = self.detected_container['class']
        goal.confidence = self.detected_container['confidence']
        goal.plan_only = False

        self.get_logger().info('发送移动请求到 grasp_moveit_node（移动到预放置位置）...')

        send_goal_future = self.grasp_client.send_goal_async(
            goal,
            feedback_callback=self.grasp_feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('移动请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'移动到预放置位置失败: {result.message}')
            return False

        # 等待机械臂稳定
        self.wait_for_motion_complete(2.0)

        # 检查 joint6 角度
        if self.current_joint_state is not None and len(self.current_joint_state.position) >= 6:
            final_j6 = self.current_joint_state.position[5]
            self.get_logger().info(f'  到达预放置位置，joint6: {math.degrees(final_j6):.2f}度')

        self.get_logger().info('✓ 已到达预放置位置')
        return True

    def grasp_feedback_callback(self, feedback_msg):
        """抓取反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'  移动状态: {feedback.state}')

    def move_to_place(self):
        """步骤5: 使用笛卡尔规划平移到最终放置位置"""
        self.current_state = PlaceState.MOVE_TO_PLACE
        self.get_logger().info(f'[步骤5] 平移到最终放置位置（沿接近方向前进 {self.pre_place_offset}m）...')

        if self.detected_container is None:
            self.get_logger().error('没有检测结果')
            return False

        if self.current_joint_state is None:
            self.get_logger().error('未获取到当前关节状态')
            return False

        # 保存当前的 joint6 角度
        current_joint6 = self.current_joint_state.position[5]
        self.get_logger().info(f'  当前 joint6: {math.degrees(current_joint6):.2f}度')

        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 计算最终放置位置：容器位置 + finger_tip_offset（沿 Z 轴负方向）
        approach_direction = np.array([
            self.approach_normal.vector.x,
            self.approach_normal.vector.y,
            self.approach_normal.vector.z
        ])

        self.final_place_position = copy.deepcopy(self.detected_container['position'])
        self.final_place_position.point.x -= approach_direction[0] * self.finger_tip_offset
        self.final_place_position.point.y -= approach_direction[1] * self.finger_tip_offset
        self.final_place_position.point.z -= approach_direction[2] * self.finger_tip_offset

        self.get_logger().info(f'  最终放置位置: ({self.final_place_position.point.x:.3f}, '
                              f'{self.final_place_position.point.y:.3f}, '
                              f'{self.final_place_position.point.z:.3f})')
        self.get_logger().info(f'  平移距离: {self.pre_place_offset}m')

        # 发送移动请求（笛卡尔规划）
        goal = GraspToPoint.Goal()
        goal.target = self.final_place_position
        goal.normal = self.approach_normal
        goal.short_axis = self.short_axis
        goal.label = self.detected_container['class']
        goal.confidence = self.detected_container['confidence']
        goal.plan_only = False

        self.get_logger().info('发送平移请求到 grasp_moveit_node（笛卡尔路径）...')

        send_goal_future = self.grasp_client.send_goal_async(
            goal,
            feedback_callback=self.grasp_feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('移动请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'平移到放置位置失败: {result.message}')
            return False

        # 等待机械臂稳定
        self.wait_for_motion_complete(2.0)

        # 检查 joint6 是否改变
        if self.current_joint_state is not None and len(self.current_joint_state.position) >= 6:
            final_joint6 = self.current_joint_state.position[5]
            joint6_change = abs(final_joint6 - current_joint6)
            self.get_logger().info(f'  最终 joint6: {math.degrees(final_joint6):.2f}度')
            self.get_logger().info(f'  joint6 变化: {math.degrees(joint6_change):.2f}度')

            if joint6_change > 0.5:
                self.get_logger().warn(f'警告：joint6 改变了 {math.degrees(joint6_change):.2f}度！')
            else:
                self.get_logger().info('✓ joint6 角度保持稳定')

        self.get_logger().info('✓ 已到达最终放置位置')
        return True

    def open_gripper(self):
        """步骤6: 打开夹爪到 20%"""
        self.current_state = PlaceState.OPEN_GRIPPER
        self.get_logger().info(f'[步骤6] 打开夹爪到 {self.gripper_open_position}%...')

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('夹爪服务不可用: /gripper_control')
            return False

        goal = GripperControl.Goal()
        goal.command = 'open'
        goal.position = self.gripper_open_position
        goal.speed = 50.0
        goal.wait_for_completion = True

        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('夹爪控制请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'夹爪打开失败: {result.message}')
            return False

        self.get_logger().info(f'✓ 夹爪已打开到: {result.final_position:.1f}%')
        return True

    def wait_after_release(self):
        """步骤7: 等待 2.5 秒"""
        self.current_state = PlaceState.WAIT
        self.get_logger().info(f'[步骤7] 等待 {self.wait_time_after_release} 秒...')

        time.sleep(self.wait_time_after_release)

        self.get_logger().info('✓ 等待完成')
        return True

    def return_to_pre_place(self):
        """步骤8: 返回预放置位置"""
        self.current_state = PlaceState.RETURN_TO_PRE_PLACE
        self.get_logger().info('[步骤8] 返回预放置位置...')

        if self.pre_place_position is None:
            self.get_logger().error('没有预放置位置信息')
            return False

        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 发送移动请求
        goal = GraspToPoint.Goal()
        goal.target = self.pre_place_position
        goal.normal = self.approach_normal
        goal.short_axis = self.short_axis
        goal.label = self.detected_container['class']
        goal.confidence = self.detected_container['confidence']
        goal.plan_only = False

        self.get_logger().info('发送移动请求到 grasp_moveit_node（返回预放置位置）...')

        send_goal_future = self.grasp_client.send_goal_async(
            goal,
            feedback_callback=self.grasp_feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('移动请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'返回预放置位置失败: {result.message}')
            return False

        # 等待机械臂稳定
        self.wait_for_motion_complete(2.0)

        self.get_logger().info('✓ 已返回预放置位置')
        return True

    def close_gripper(self):
        """步骤9: 关闭夹爪"""
        self.current_state = PlaceState.CLOSE_GRIPPER
        self.get_logger().info(f'[步骤9] 关闭夹爪到 {self.gripper_close_position}%...')

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('夹爪服务不可用: /gripper_control')
            return False

        goal = GripperControl.Goal()
        goal.command = 'close'
        goal.position = self.gripper_close_position
        goal.speed = 50.0
        goal.wait_for_completion = True

        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('夹爪控制请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'夹爪关闭失败: {result.message}')
            return False

        self.get_logger().info(f'✓ 夹爪已关闭到: {result.final_position:.1f}%')
        return True

    def return_to_home(self):
        """步骤10: 返回零位"""
        self.current_state = PlaceState.RETURN_TO_HOME
        self.get_logger().info('[步骤10] 返回零位...')
        self.get_logger().info(f'  目标关节角度: {self.home_pose}')

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.home_pose
        self.hand_cmd_pub.publish(msg)

        # 等待机械臂到达零位
        self.get_logger().info('等待机械臂返回零位...')
        max_wait_time = 20.0
        start_time = time.time()
        position_reached = False

        while (time.time() - start_time) < max_wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_joint_state is not None:
                if len(self.current_joint_state.position) >= len(self.home_pose):
                    errors = [abs(self.current_joint_state.position[i] - self.home_pose[i])
                             for i in range(len(self.home_pose))]
                    max_error = max(errors)

                    if max_error < 0.1:
                        position_reached = True
                        self.get_logger().info(f'✓ 机械臂已返回零位（最大误差: {max_error:.4f} rad）')
                        break

            time.sleep(0.1)

        if not position_reached:
            self.get_logger().warn('机械臂可能未完全到达零位，继续执行...')

        # 等待机械臂稳定
        self.wait_for_motion_complete(1.0)

        self.get_logger().info('✓ 已返回零位')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PlaceStateMachine()

    # 等待所有服务就绪
    node.get_logger().info('等待所有 action servers 就绪...')
    time.sleep(2.0)

    # 执行放置流程
    success = node.run()

    if success:
        node.get_logger().info('放置任务成功完成')
    else:
        node.get_logger().error('放置任务失败')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
