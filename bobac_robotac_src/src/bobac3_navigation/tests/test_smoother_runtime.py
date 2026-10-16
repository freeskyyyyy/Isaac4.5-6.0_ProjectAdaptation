"""Run the installed smoother with synthetic clock/odom in an isolated DDS domain.

No simulator, shared course topic, or actuator is used. Run explicitly after
building the package: python3 .../tests/test_smoother_runtime.py
"""
import math
import os
from pathlib import Path
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class SmootherRuntimeTest(unittest.TestCase):
    def setUp(self):
        # Separate from the course's domain 44, and from other test processes.
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(100 + os.getpid() % 100))
        os.environ['ROS_DOMAIN_ID'] = self.env['ROS_DOMAIN_ID']
        rclpy.init()
        self.node = rclpy.create_node('isolated_smoother_regression')
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.command = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.output = []
        self.node.create_subscription(Twist, '/cmd_vel', lambda m: self.output.append(
            [m.linear.x, m.linear.y, m.angular.z]), 100)
        self.sim = 10.
        self.log = open('/tmp/bobac-smoother-regression.log', 'a')
        executable = Path(get_package_prefix('bobac3_navigation')) / 'lib/bobac3_navigation/sim_time_velocity_smoother'
        params = Path(get_package_share_directory('bobac3_navigation')) / 'params/bobac3_navigation_world_params.yaml'
        self.process = subprocess.Popen([str(executable), '--ros-args', '-r', '__node:=velocity_smoother',
            '--params-file', str(params), '-r', 'cmd_vel:=cmd_vel_nav',
            '-r', 'cmd_vel_smoothed:=cmd_vel'], env=self.env, stdout=self.log, stderr=self.log)
        client = self.node.create_client(ChangeState, '/velocity_smoother/change_state')
        self.assertTrue(client.wait_for_service(timeout_sec=8.))
        self.pump(.2)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=4.)
            self.assertTrue(future.done() and future.result().success)
        deadline = time.monotonic() + 4.
        while self.command.get_subscription_count() < 1 and time.monotonic() < deadline:
            self.pump(.05)
        self.assertGreaterEqual(self.command.get_subscription_count(), 1)
        self.advance([0., 0., 0.], [0., 0., 0.], 5)

    def tearDown(self):
        if hasattr(self, 'process'):
            self.process.terminate()
            self.process.wait(timeout=5.)
            self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.005)

    def advance(self, request, feedback, count, *, stale=False):
        first = len(self.output)
        for _ in range(count):
            self.sim += .05
            clock = Clock()
            clock.clock.sec = int(self.sim)
            clock.clock.nanosec = round((self.sim % 1) * 1e9)
            # Odom is delivered before the timer-triggering /clock update.
            odom = Odometry()
            stamp = self.sim - (.5 if stale else 0.)
            odom.header.stamp.sec = int(stamp)
            odom.header.stamp.nanosec = round((stamp % 1) * 1e9)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = feedback
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = request
            self.odom.publish(odom)
            self.command.publish(cmd)
            self.pump(.025)
            self.clock.publish(clock)
            self.pump(.04)
        result = self.output[first:]
        self.assertTrue(result, 'No real smoother output')
        return result

    def test_feedback_above_request_cannot_amplify_command(self):
        rows = self.advance([.04, 0., 0.], [.06, .002, .003], 12)
        for row in rows:
            self.assertLessEqual(row[0], .04 + 1e-9)
            self.assertEqual(row[1:], [0., 0.])
        self.assertAlmostEqual(rows[-1][0], .04)

    def test_normal_zero_brakes_despite_feedback_lag(self):
        self.advance([.04, 0., 0.], [.04, 0., 0.], 12)
        rows = self.advance([0., 0., 0.], [.04, 0., 0.], 12)
        self.assertEqual(rows[-1], [0., 0., 0.])
        for previous, current in zip([[.04, 0., 0.]] + rows, rows):
            self.assertGreaterEqual(current[0], -1e-9)
            self.assertLessEqual(current[0], previous[0] + 1e-9)
            self.assertLessEqual(previous[0] - current[0], .0075 + 1e-6)

    def test_normal_acceleration_uses_measured_feedback(self):
        # A stationary measured base must not receive an open-loop ramp to .04.
        rows = self.advance([.04, 0., 0.], [0., 0., 0.], 10)
        self.assertLessEqual(max(r[0] for r in rows), .0075 + 1e-6)
        rows = self.advance([.04, 0., 0.], [.02, 0., 0.], 10)
        self.assertGreater(rows[-1][0], .02)

    def test_negative_request_and_direction_change_remain_bounded(self):
        rows = self.advance([-.04, 0., 0.], [-.06, 0., 0.], 12)
        self.assertGreaterEqual(min(r[0] for r in rows), -.04 - 1e-9)
        rows = [rows[-1]] + self.advance([.04, 0., 0.], [-.06, 0., 0.], 20)
        for previous, current in zip(rows, rows[1:]):
            self.assertLessEqual(abs(current[0] - previous[0]), .0075 + 1e-6)
        self.assertTrue(all(-.04 - 1e-9 <= r[0] <= .04 + 1e-9 for r in rows))

    def test_nonfinite_and_absolute_overspeed_feedback_stop(self):
        for value in (.11, -.11, math.nan, math.inf):
            with self.subTest(feedback=value):
                rows = self.advance([.04, 0., 0.], [value, 0., 0.], 8)
                self.assertEqual(rows[-1], [0., 0., 0.])

    def test_stale_feedback_stops(self):
        rows = self.advance([.04, 0., 0.], [0., 0., 0.], 8, stale=True)
        self.assertEqual(rows[-1], [0., 0., 0.])


if __name__ == '__main__':
    unittest.main(verbosity=2)
