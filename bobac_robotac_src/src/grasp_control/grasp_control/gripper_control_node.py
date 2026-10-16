#!/usr/bin/env python3
"""Gripper Action using measured joints: 0% open (0.78rad), 100% closed (0rad).

Contact may stop a finger short of the requested position. That is a timeout,
not proof of holding an object; physical grasp acceptance is separate.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from eco65b_grasp_interfaces.action import GripperControl


class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.gripper_joint = 'gripper_l_joint1'
        self.min_angle = .78  # Fully open; names retained for existing callers.
        self.max_angle = 0.  # Fully closed.
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('goal_timeout', 15.)  # Wall watchdog, including pauses.
        self.declare_parameter('feedback_timeout', 1.5)
        self.declare_parameter('settle_time', .1)  # Simulation seconds at target.
        self.declare_parameter('position_tolerance', .01)  # Radians.
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        if any(not math.isfinite(v) or v <= 0. for v in [self.goal_timeout,
                self.feedback_timeout, self.settle_time, self.position_tolerance]):
            raise ValueError('Gripper timeouts and tolerances must be finite and positive')
        self._feedback_lock = threading.Lock()
        self._joint_feedback = None
        self._goal_lock = threading.Lock()
        self._busy = False
        self._current_goal_handle = None
        self.hand_cmd_pub = self.create_publisher(JointState, '/hand_command', 10)
        self._callbacks = ReentrantCallbackGroup()
        self.joint_sub = self.create_subscription(JointState,
            self.get_parameter('joint_state_topic').value, self._receive_joint,
            qos_profile_sensor_data, callback_group=self._callbacks)
        self._action_server = ActionServer(self, GripperControl, '/gripper_control',
            execute_callback=self.execute_callback, goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callbacks)
        self.get_logger().info('Gripper Action ready; completion requires fresh measured joint feedback')

    def _now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def _receive_joint(self, message):
        if message.name.count(self.gripper_joint) != 1:
            return
        index = message.name.index(self.gripper_joint)
        if index >= len(message.position):
            return
        position = message.position[index]
        source = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not math.isfinite(position) or not -.05 <= position <= .83:
            return
        with self._feedback_lock:
            self._joint_feedback = (position, source, time.monotonic())

    def _measured(self):
        with self._feedback_lock:
            value = self._joint_feedback
        if value is None:
            raise RuntimeError('No gripper joint feedback')
        position, source, wall = value
        age = self._now()-source
        if source <= 0. or not -.05 <= age <= .15 or time.monotonic()-wall > self.feedback_timeout:
            raise RuntimeError('Gripper joint feedback is stale or has an invalid source stamp')
        return value

    def goal_callback(self, request):
        if (request.command not in ('open', 'close', 'position')
                or not math.isfinite(request.position) or not 0. <= request.position <= 100.
                or not math.isfinite(request.speed) or not 0. <= request.speed <= 100.):
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True  # Reserve before the accepted callback is dispatched.
        return GoalResponse.ACCEPT

    def percentage_to_radians(self, percentage):
        return self.min_angle+(self.max_angle-self.min_angle)*max(0., min(100., percentage))/100.

    def radians_to_percentage(self, radians):
        return max(0., min(100., 100.*(radians-self.min_angle)/(self.max_angle-self.min_angle)))

    def publish_gripper_command(self, position_radians):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.gripper_joint]
        message.position = [float(position_radians)]
        self.hand_cmd_pub.publish(message)

    def _hold(self):
        try:
            self.publish_gripper_command(self._measured()[0])
        except RuntimeError:
            pass  # An old/unknown joint position is not a safe new target.

    def execute_callback(self, handle):
        self._current_goal_handle = handle
        result = GripperControl.Result(success=False)
        started_wall = time.monotonic()
        started_sim = self._now()
        last_sim = started_sim
        reached_since = None
        reached_stamps = set()
        try:
            # Do not issue any command before receiving usable feedback.
            while True:
                if handle.is_cancel_requested:
                    self._hold()
                    handle.canceled()
                    result.message = 'Canceled before motion'
                    return result
                try:
                    measured = self._measured()
                    break
                except RuntimeError:
                    if time.monotonic()-started_wall >= self.feedback_timeout:
                        raise RuntimeError('No fresh gripper joint feedback; no command sent')
                    time.sleep(.01)
            goal = handle.request
            target = (self.min_angle if goal.command == 'open' else
                      self.max_angle if goal.command == 'close' else
                      self.percentage_to_radians(goal.position))
            commanded = measured[0]
            speed = goal.speed or 50.
            rate = .78*speed/100.  # Full stroke is 2 simulation seconds at speed=50.
            feedback = GripperControl.Feedback(state='MOVING',
                current_position=self.radians_to_percentage(measured[0]))
            handle.publish_feedback(feedback)
            if not goal.wait_for_completion:
                self.publish_gripper_command(target)
                result.success = True
                result.final_position = self.radians_to_percentage(measured[0])
                result.message = 'Command sent without waiting; final_position is measured, arrival unverified'
                handle.succeed()
                return result
            while rclpy.ok():
                now = self._now()
                if now < last_sim:
                    raise RuntimeError('Simulation clock rewound during gripper Action')
                if handle.is_cancel_requested:
                    self._hold()
                    handle.canceled()
                    result.message = 'Canceled; fresh measured position commanded as hold'
                    return result
                measured = self._measured()
                result.final_position = self.radians_to_percentage(measured[0])
                if time.monotonic()-started_wall >= self.goal_timeout or now-started_sim >= 10.:
                    raise RuntimeError('Gripper target not reached before deadline; gripping unverified')
                step = rate*min(.05, max(0., now-last_sim))
                commanded += max(-step, min(step, target-commanded))
                self.publish_gripper_command(commanded)
                feedback.current_position = result.final_position
                handle.publish_feedback(feedback)
                if abs(measured[0]-target) <= self.position_tolerance:
                    if reached_since is None:
                        reached_since = now
                    reached_stamps.add(measured[1])
                    if now-reached_since >= self.settle_time and len(reached_stamps) >= 3:
                        feedback.state = 'REACHED'
                        handle.publish_feedback(feedback)
                        result.success = True
                        result.message = 'Fresh joint feedback confirms target position; object holding requires separate evidence'
                        handle.succeed()
                        return result
                else:
                    reached_since = None
                    reached_stamps.clear()
                last_sim = now
                time.sleep(.01)  # Other executor threads keep feedback/cancel callbacks live.
            raise RuntimeError('ROS shutdown during gripper Action')
        except Exception as exc:
            self._hold()
            result.message = str(exc)
            if handle.is_active:
                handle.abort()
            return result
        finally:
            with self._goal_lock:
                self._busy = False
                self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
