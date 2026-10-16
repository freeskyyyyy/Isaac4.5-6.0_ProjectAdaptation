"""Real rclpy Action regression with an isolated synthetic joint publisher.

Run inside the course container with ROS_DOMAIN_ID=146. This is a software
contract test, not evidence of physical gripping in Isaac.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from eco65b_grasp_interfaces.action import GripperControl
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class GripperFeedbackTest(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=146)
        self.node = rclpy.create_node('gripper_feedback_test', context=self.context)
        self.position = .78
        self.target = .78
        self.follow = False
        self.publish_feedback = True
        self.simulation_seconds = 100.
        self.paused = False
        self.commands = []
        self.feedback = []
        self.publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.clock_publisher = self.node.create_publisher(Clock, '/clock', 10)
        self.subscription = self.node.create_subscription(JointState, '/hand_command', self.command, 10)
        self.timer = self.node.create_timer(.02, self.tick)
        self.client = ActionClient(self.node, GripperControl, '/gripper_control')
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        source = str(Path(__file__).resolve().parents[1])
        node_source = os.environ.get('GRIPPER_REGRESSION_NODE_PATH',
            str(Path(source)/'grasp_control/gripper_control_node.py'))
        self.server = subprocess.Popen([sys.executable, node_source,
            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'goal_timeout:=2.5',
            '-p', 'feedback_timeout:=0.6', '-p', 'settle_time:=0.08'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
            env={**os.environ, 'ROS_DOMAIN_ID': '146'})
        self.assertTrue(self.client.wait_for_server(timeout_sec=8))
        time.sleep(.15)

    def tearDown(self):
        os.killpg(self.server.pid, signal.SIGINT)
        try:
            self.server.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.server.pid, signal.SIGKILL)
            self.server.wait(timeout=3)
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def command(self, message):
        self.commands.append(list(message.position))
        if message.position:
            self.target = message.position[0]

    def tick(self):
        if not self.paused:
            self.simulation_seconds += .02
        seconds = int(self.simulation_seconds)
        stamp = Time(sec=seconds, nanosec=int((self.simulation_seconds-seconds)*1e9))
        self.clock_publisher.publish(Clock(clock=stamp))
        if self.follow:
            delta = self.target-self.position
            self.position += max(-.012, min(.012, delta))
        if self.publish_feedback:
            message = JointState()
            message.header.stamp = stamp
            message.name = ['gripper_l_joint1']
            message.position = [self.position]
            self.publisher.publish(message)

    def wait(self, future, timeout=5):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(future.done(), 'Action did not respond before wall deadline')
        return future.result()

    def send(self, command='close', position=100., speed=100.):
        goal = GripperControl.Goal(command=command, position=position, speed=speed,
                                   wait_for_completion=True)
        return self.wait(self.client.send_goal_async(goal,
            feedback_callback=lambda m: self.feedback.append(m.feedback)))

    def test_no_feedback_aborts_without_sending_motion(self):
        self.publish_feedback = False
        time.sleep(.7)
        handle = self.send()
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertFalse(result.result.success, 'Missing joint feedback must not become REACHED')
        self.assertEqual(result.status, 6)
        self.assertEqual(self.commands, [])
        self.assertFalse(any(m.state == 'REACHED' for m in self.feedback))

    def test_stalled_joint_is_not_a_successful_close(self):
        handle = self.send()
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertFalse(result.result.success, 'Elapsed sleep cannot establish gripping')
        self.assertEqual(result.status, 6)
        self.assertLess(result.result.final_position, 2.)

    def test_success_and_feedback_use_measured_position(self):
        self.follow = True
        handle = self.send('position', 50.)
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertTrue(result.result.success)
        self.assertEqual(result.status, 4)
        self.assertAlmostEqual(result.result.final_position, 50., delta=2.)
        self.assertAlmostEqual(self.position, .39, delta=.015)
        self.assertTrue(self.feedback)
        self.assertLess(self.feedback[0].current_position, 2., 'Initial feedback must be measured')
        self.assertEqual(self.feedback[-1].state, 'REACHED')

    def test_mutual_exclusion_and_cancel_remain_responsive(self):
        handle = self.send()
        self.assertTrue(handle.accepted)
        time.sleep(.15)
        other = self.send('position', 50.)
        self.assertFalse(other.accepted)
        start = time.monotonic()
        cancel = self.wait(handle.cancel_goal_async(), timeout=.6)
        self.assertTrue(cancel.goals_canceling)
        result = self.wait(handle.get_result_async(), timeout=.6)
        self.assertLess(time.monotonic()-start, .6)
        self.assertEqual(result.status, 5)
        self.assertFalse(result.result.success)
        self.assertAlmostEqual(self.commands[-1][0], self.position, delta=.015)

    def test_invalid_goal_is_rejected_before_command(self):
        for position, speed in [(float('nan'), 50.), (50., float('inf')),
                                (-1., 50.), (101., 50.), (50., -1.), (50., 101.)]:
            with self.subTest(position=position, speed=speed):
                self.assertFalse(self.send('position', position, speed).accepted)
        self.assertEqual(self.commands, [])

    def test_pause_cannot_report_arrival_from_wall_sleep(self):
        self.paused = True
        handle = self.send()
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertTrue(all(abs(p[0]-.78) < .001 for p in self.commands))

    def test_clock_rewind_aborts_inflight_request(self):
        handle = self.send()
        time.sleep(.15)
        self.simulation_seconds -= 20.
        result = self.wait(handle.get_result_async(), timeout=.8)
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertIn('clock', result.result.message.lower())


if __name__ == '__main__':
    unittest.main()
