#!/usr/bin/env python3
"""
抓取状态机节点
作为中心大脑，编排整个抓取流程：
1. 移动到观察位置
2. 调用检测服务
3. 调用抓取服务（到预抓取位置）
4. 打开夹爪
5. 前进到抓取位置
6. 关闭夹爪
7. 提升物体
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


class GraspState(Enum):
    """抓取状态枚举"""
    IDLE = 0
    MOVE_TO_OBSERVATION = 1
    DETECTING_OBJECT = 2
    MOVE_TO_PRE_GRASP = 3
    OPEN_GRIPPER = 4
    MOVE_TO_GRASP = 5
    CLOSE_GRIPPER = 6
    LIFT_OBJECT = 7
    DONE = 8
    ERROR = 9


class GraspStateMachine(Node):
    def __init__(self):
        super().__init__('grasp_state_machine')

        # 参数声明
        self.declare_parameter('observation_pose', [0.0, 0.58, -1.67, -0.5, 1.51, 0.0])
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('target_classes', ['pencil', 'pen'])
        self.declare_parameter('confidence_threshold', 0.01)
        self.declare_parameter('finger_tip_offset', 0.03)  # 指尖固定偏移（米）
        self.declare_parameter('pre_grasp_offset', 0.05)  # 预抓取偏移（米）
        self.declare_parameter('lift_offset', 0.1)  # 提升偏移（米）
        self.declare_parameter('gripper_open_position', 100.0)  # 夹爪打开位置（0-100%，100=完全打开）
        self.declare_parameter('gripper_close_position', 0.0)  # 夹爪关闭位置（0-100%，0=完全关闭）

        # 获取参数
        self.observation_pose = self.get_parameter('observation_pose').value
        self.joint_names = self.get_parameter('joint_names').value
        self.target_classes = self.get_parameter('target_classes').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.finger_tip_offset = self.get_parameter('finger_tip_offset').value
        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset').value
        self.lift_offset = self.get_parameter('lift_offset').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value

        # 发布器：用于发送关节命令（移动到观察位置）
        self.hand_cmd_pub = self.create_publisher(JointState, '/hand_command', 10)

        # 订阅器：订阅当前关节状态
        self.current_joint_state = None
        self.declare_parameter('joint_state_topic', '/arm/joint_states')
        self.create_subscription(JointState, self.get_parameter('joint_state_topic').value,
                                 self._joint_state_callback, qos_profile_sensor_data)

        # Action Clients
        self.detect_client = ActionClient(self, DetectObject, '/detect_object')
        self.grasp_client = ActionClient(self, GraspToPoint, '/grasp_to_point')
        self.gripper_client = ActionClient(self, GripperControl, '/gripper_control')

        # 状态变量
        self.current_state = GraspState.IDLE
        self.detected_object = None  # 检测结果
        self.pre_grasp_pose = None  # 预抓取位置
        self.grasp_pose = None  # 实际抓取位置
        self.lift_pose = None  # 提升位置

        self.get_logger().info('抓取状态机节点已启动')
        self.get_logger().info(f'目标类别: {self.target_classes}')
        self.get_logger().info(f'指尖偏移: {self.finger_tip_offset}m, 预抓取偏移: {self.pre_grasp_offset}m, 提升偏移: {self.lift_offset}m')

    def _joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_state = msg

    def verify_position_reached(self, target_pose, tolerance=0.01):
        """
        验证机械臂是否到达目标位置

        Args:
            target_pose: 目标位姿（PoseStamped）
            tolerance: 位置容差（米）

        Returns:
            bool: 是否到达目标位置
        """
        if self.current_joint_state is None:
            self.get_logger().warn('无法验证位置：未收到关节状态')
            return True  # 如果没有状态信息，假设到达

        # TODO: 这里需要通过正运动学计算当前末端位置，然后与目标位置比较
        # 暂时返回 True，后续可以添加更严格的验证
        return True

    def wait_for_motion_complete(self, wait_time=2.0):
        """
        等待机械臂运动完成并稳定

        Args:
            wait_time: 等待时间（秒）
        """
        self.get_logger().info(f'等待机械臂稳定 {wait_time}秒...')
        time.sleep(wait_time)
        self.get_logger().info('机械臂已稳定')

    def calculate_offset_position(self, base_position, normal, offset_distance):
        """
        根据法向量计算偏移后的位置

        Args:
            base_position: PointStamped，基准位置（物体位置）
            normal: Vector3Stamped，法向量（接近方向）
            offset_distance: float，偏移距离（正数=沿法向量方向，负数=反方向）

        Returns:
            PointStamped，偏移后的位置
        """
        import copy
        offset_position = copy.deepcopy(base_position)

        # 归一化法向量
        import math
        nx = normal.vector.x
        ny = normal.vector.y
        nz = normal.vector.z
        norm = math.sqrt(nx*nx + ny*ny + nz*nz)

        if norm > 1e-6:
            nx /= norm
            ny /= norm
            nz /= norm

        # 应用偏移（沿法向量反方向 = 远离物体）
        offset_position.point.x -= nx * offset_distance
        offset_position.point.y -= ny * offset_distance
        offset_position.point.z -= nz * offset_distance

        return offset_position

    def run(self):
        """执行完整的抓取流程"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始执行抓取流程')
        self.get_logger().info('=' * 60)

        # 步骤1: 移动到观察位置
        if not self.move_to_observation_pose():
            self.current_state = GraspState.ERROR
            return False

        # 步骤2: 检测物体
        if not self.detect_object():
            self.current_state = GraspState.ERROR
            return False

        # 步骤3: 移动到预抓取位置
        if not self.move_to_pre_grasp():
            self.current_state = GraspState.ERROR
            return False

        # 步骤4: 打开夹爪
        if not self.open_gripper():
            self.current_state = GraspState.ERROR
            return False

        # 步骤5: 前进到实际抓取位置
        if not self.move_to_grasp():
            self.current_state = GraspState.ERROR
            return False

        # 步骤6: 关闭夹爪
        if not self.close_gripper():
            self.current_state = GraspState.ERROR
            return False

        # 步骤7: 提升物体
        if not self.lift_object():
            self.current_state = GraspState.ERROR
            return False

        self.current_state = GraspState.DONE
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ 抓取流程完成！')
        self.get_logger().info('=' * 60)
        return True

    def move_to_observation_pose(self):
        """步骤1: 移动到观察位置"""
        self.current_state = GraspState.MOVE_TO_OBSERVATION
        self.get_logger().info('[步骤1] 移动到观察位置...')

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.observation_pose
        self.hand_cmd_pub.publish(msg)

        # 等待机械臂到达
        time.sleep(5.0)

        self.get_logger().info('✓ 到达观察位置')
        return True

    def detect_object(self):
        """步骤2: 检测物体"""
        self.current_state = GraspState.DETECTING_OBJECT
        self.get_logger().info('[步骤2] 开始检测物体...')

        # 等待 action server
        if not self.detect_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('检测服务不可用: /detect_object')
            return False

        # 发送检测请求
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

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'检测失败: {result.message}')
            return False

        # 保存检测结果
        self.detected_object = {
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

    def move_to_pre_grasp(self):
        """步骤3: 移动到预抓取位置"""
        self.current_state = GraspState.MOVE_TO_PRE_GRASP
        self.get_logger().info('[步骤3] 移动到预抓取位置...')

        if self.detected_object is None:
            self.get_logger().error('没有检测结果')
            return False

        # 等待 action server
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 计算预抓取位置：直接在物体位置的 Z 轴上加偏移
        import copy
        pre_grasp_position = copy.deepcopy(self.detected_object['position'])
        total_offset = self.finger_tip_offset + self.pre_grasp_offset
        pre_grasp_position.point.z += total_offset  # 直接在 Z 轴上向上偏移

        self.get_logger().info(f'  物体位置: ({self.detected_object["position"].point.x:.3f}, '
                              f'{self.detected_object["position"].point.y:.3f}, '
                              f'{self.detected_object["position"].point.z:.3f})')
        self.get_logger().info(f'  预抓取位置: ({pre_grasp_position.point.x:.3f}, '
                              f'{pre_grasp_position.point.y:.3f}, '
                              f'{pre_grasp_position.point.z:.3f})')
        self.get_logger().info(f'  Z轴偏移量: {total_offset:.3f}m')

        # 发送抓取请求
        goal = GraspToPoint.Goal()
        goal.target = pre_grasp_position
        goal.normal = self.detected_object['normal']
        goal.short_axis = self.detected_object['long_axis']
        goal.label = self.detected_object['class']
        goal.confidence = self.detected_object['confidence']
        goal.plan_only = False

        self.get_logger().info('发送抓取请求到 grasp_moveit_node...')

        send_goal_future = self.grasp_client.send_goal_async(
            goal,
            feedback_callback=self.grasp_feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('抓取请求被拒绝')
            return False

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'移动到预抓取位置失败: {result.message}')
            return False

        # 保存预抓取位置
        self.pre_grasp_pose = result.commanded_pose

        # 等待机械臂稳定
        self.wait_for_motion_complete(2.0)

        self.get_logger().info(f'✓ 到达预抓取位置')
        return True

    def grasp_feedback_callback(self, feedback_msg):
        """抓取反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'  抓取状态: {feedback.state}')

    def open_gripper(self):
        """步骤4: 打开夹爪"""
        self.current_state = GraspState.OPEN_GRIPPER
        self.get_logger().info('[步骤4] 打开夹爪...')

        result = self._control_gripper('open', self.gripper_open_position)

        if result:
            # 等待夹爪完全打开并稳定（增加等待时间）
            self.wait_for_motion_complete(3.0)

        return result

    def close_gripper(self):
        """步骤6: 关闭夹爪"""
        self.current_state = GraspState.CLOSE_GRIPPER
        self.get_logger().info('[步骤6] 关闭夹爪...')

        result = self._control_gripper('close', self.gripper_close_position)

        if result:
            # 等待夹爪完全关闭并抓稳物体（增加等待时间）
            self.wait_for_motion_complete(5.0)

        return result

    def _control_gripper(self, command, position):
        """控制夹爪的通用方法"""
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('夹爪服务不可用: /gripper_control')
            return False

        goal = GripperControl.Goal()
        goal.command = command
        goal.position = position
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
            self.get_logger().error(f'夹爪控制失败: {result.message}')
            return False

        self.get_logger().info(f'✓ 夹爪控制完成: {result.final_position:.1f}%')
        return True

    def move_to_grasp(self):
        """步骤5: 前进到实际抓取位置"""
        self.current_state = GraspState.MOVE_TO_GRASP
        self.get_logger().info(f'[步骤5] 前进到实际抓取位置（下降 {self.pre_grasp_offset}m）...')

        if self.detected_object is None:
            self.get_logger().error('没有检测结果')
            return False

        # 等待 action server
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 计算抓取位置：直接在物体位置的 Z 轴上加 finger_tip_offset
        import copy
        grasp_position = copy.deepcopy(self.detected_object['position'])
        grasp_position.point.z += self.finger_tip_offset  # 直接在 Z 轴上向上偏移

        self.get_logger().info(f'  抓取位置: ({grasp_position.point.x:.3f}, '
                              f'{grasp_position.point.y:.3f}, '
                              f'{grasp_position.point.z:.3f})')
        self.get_logger().info(f'  Z轴偏移量: {self.finger_tip_offset:.3f}m')

        # 发送抓取请求
        goal = GraspToPoint.Goal()
        goal.target = grasp_position
        goal.normal = self.detected_object['normal']
        goal.short_axis = self.detected_object['long_axis']
        goal.plan_only = False

        self.get_logger().info('发送抓取请求...')

        send_goal_future = self.grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('抓取请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'移动到抓取位置失败: {result.message}')
            return False

        self.grasp_pose = result.commanded_pose

        # 等待机械臂稳定
        self.wait_for_motion_complete(1.5)

        self.get_logger().info('✓ 到达抓取位置')
        return True

    def lift_object(self):
        """步骤7: 提升物体"""
        self.current_state = GraspState.LIFT_OBJECT
        self.get_logger().info(f'[步骤7] 提升物体（向上 {self.lift_offset}m）...')

        if self.detected_object is None:
            self.get_logger().error('没有检测结果')
            return False

        # 等待 action server
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('抓取服务不可用: /grasp_to_point')
            return False

        # 计算提升位置：从抓取位置直接在 Z 轴上向上移动 lift_offset
        # 不使用法向量，直接增加 Z 坐标
        import copy
        grasp_position = self.calculate_offset_position(
            self.detected_object['position'],
            self.detected_object['normal'],
            self.finger_tip_offset
        )

        lift_position = copy.deepcopy(grasp_position)
        lift_position.point.z += self.lift_offset  # 直接在 Z 轴上向上移动

        self.get_logger().info(f'  抓取位置: ({grasp_position.point.x:.3f}, '
                              f'{grasp_position.point.y:.3f}, '
                              f'{grasp_position.point.z:.3f})')
        self.get_logger().info(f'  提升位置: ({lift_position.point.x:.3f}, '
                              f'{lift_position.point.y:.3f}, '
                              f'{lift_position.point.z:.3f})')

        # 发送提升请求
        goal = GraspToPoint.Goal()
        goal.target = lift_position
        goal.normal = self.detected_object['normal']
        goal.short_axis = self.detected_object['long_axis']
        goal.plan_only = False

        self.get_logger().info('发送提升请求...')

        send_goal_future = self.grasp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('提升请求被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if not result.success:
            self.get_logger().error(f'提升失败: {result.message}')
            return False

        self.lift_pose = result.commanded_pose

        # 等待机械臂稳定
        self.wait_for_motion_complete(1.5)

        self.get_logger().info('✓ 物体已提升')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GraspStateMachine()

    # 等待所有服务就绪
    node.get_logger().info('等待所有 action servers 就绪...')
    time.sleep(2.0)

    # 执行抓取流程
    success = node.run()

    if success:
        node.get_logger().info('抓取任务成功完成')
    else:
        node.get_logger().error('抓取任务失败')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
