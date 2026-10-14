#!/usr/bin/env python3
"""
夹爪控制节点
提供 Action Server 接口来控制夹爪
支持：
- open: 完全打开（0度）
- close: 完全关闭（65度）
- position: 指定位置（0-100%）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import JointState
from eco65b_grasp_interfaces.action import GripperControl
import time
import threading


class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')

        # 夹爪参数
        self.gripper_joint = 'gripper_l_joint1'
        self.min_angle = 0.78        # 完全打开（弧度）
        self.max_angle = 0      # 完全关闭

        # 发布器
        self.hand_cmd_pub = self.create_publisher(JointState, '/hand_command', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            GripperControl,
            '/gripper_control',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

        self.get_logger().info('夹爪控制节点已启动')
        self.get_logger().info(f'夹爪范围: {self.min_angle:.3f} - {self.max_angle:.3f} 弧度 (0-65度)')

    def goal_callback(self, goal_request):
        """处理新的 goal 请求"""
        self.get_logger().info(f'收到夹爪控制请求: command={goal_request.command}, position={goal_request.position}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理取消请求"""
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def percentage_to_radians(self, percentage):
        """
        将百分比位置（0-100）转换为弧度
        0% = 完全打开 = 0.0 弧度
        100% = 完全关闭 = 1.134 弧度
        """
        percentage = max(0.0, min(100.0, percentage))
        return self.min_angle + (self.max_angle - self.min_angle) * (percentage / 100.0)

    def radians_to_percentage(self, radians):
        """
        将弧度转换为百分比位置（0-100）
        """
        if self.max_angle == self.min_angle:
            return 0.0
        percentage = ((radians - self.min_angle) / (self.max_angle - self.min_angle)) * 100.0
        return max(0.0, min(100.0, percentage))

    def publish_gripper_command(self, position_radians):
        """发布夹爪命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.gripper_joint]
        msg.position = [float(position_radians)]
        self.hand_cmd_pub.publish(msg)

        percentage = self.radians_to_percentage(position_radians)
        self.get_logger().info(f'发布夹爪命令: {position_radians:.3f} 弧度 ({percentage:.1f}%)')

    async def execute_callback(self, goal_handle):
        """执行夹爪控制"""
        self.get_logger().info('开始执行夹爪控制...')

        goal = goal_handle.request
        feedback_msg = GripperControl.Feedback()
        result = GripperControl.Result()

        # 解析命令
        target_position_radians = 0.0

        if goal.command == 'open':
            target_position_radians = self.min_angle
            self.get_logger().info('命令: 打开夹爪')
        elif goal.command == 'close':
            target_position_radians = self.max_angle
            self.get_logger().info('命令: 关闭夹爪')
        elif goal.command == 'position':
            target_position_radians = self.percentage_to_radians(goal.position)
            self.get_logger().info(f'命令: 移动到位置 {goal.position:.1f}%')
        else:
            result.success = False
            result.message = f'未知命令: {goal.command}'
            goal_handle.abort()
            return result

        # 发布夹爪命令
        feedback_msg.state = 'MOVING'
        feedback_msg.current_position = goal.position if goal.command == 'position' else \
                                        (0.0 if goal.command == 'open' else 100.0)
        goal_handle.publish_feedback(feedback_msg)

        self.publish_gripper_command(target_position_radians)

        # 等待夹爪到达目标位置
        if goal.wait_for_completion:
            # 根据速度计算等待时间（简化模型）
            speed = goal.speed if goal.speed > 0 else 50.0
            wait_time = 2.0 * (50.0 / speed)  # 基准时间2秒，根据速度调整

            self.get_logger().info(f'等待夹爪到达目标位置... ({wait_time:.1f}秒)')
            time.sleep(wait_time)

        # 检查是否被取消
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = '已取消'
            return result

        # 完成
        feedback_msg.state = 'REACHED'
        goal_handle.publish_feedback(feedback_msg)

        result.success = True
        result.message = '夹爪控制完成'
        result.final_position = self.radians_to_percentage(target_position_radians)

        self.get_logger().info(f'✓ 夹爪控制完成: {result.final_position:.1f}%')
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
