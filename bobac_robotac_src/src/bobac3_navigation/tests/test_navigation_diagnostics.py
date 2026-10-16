"""Local relay and diagnostic checks; run with discovery-disabled Cyclone DDS."""
import importlib.util
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


relay_module = load('laser_qos_relay')
diagnostic_module = load('navigation_readiness')


class NavigationDiagnosticsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_relay_matches_both_writer_reliabilities_and_preserves_scan(self):
        for reliability in [ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE]:
            with self.subTest(reliability=reliability):
                node = relay_module.LaserQoSRelay()
                try:
                    self.assertTrue(node.get_parameter('use_sim_time').value)
                    received = []
                    publisher = node.create_publisher(LaserScan, '/laser_scan_fuse',
                                                     QoSProfile(depth=10, reliability=reliability))
                    subscription = node.create_subscription(LaserScan, '/laser_scan_fuse_best_effort',
                                                            received.append, qos_profile_sensor_data)
                    msg = LaserScan()
                    msg.header.frame_id = 'fuse_lidar_link'
                    msg.header.stamp.sec = 12
                    msg.header.stamp.nanosec = 34
                    msg.ranges = [0.5, 1.0, float('inf')]
                    deadline = time.monotonic() + 3
                    while not received and time.monotonic() < deadline:
                        publisher.publish(msg)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertTrue(received)
                    self.assertEqual(received[-1], msg)
                    node.destroy_subscription(subscription)
                finally:
                    node.destroy_node()

    def healthy_diagnostic(self):
        # Exercise diagnostic decisions without starting Nav2 or publishing chassis commands.
        node = Mock()
        node.scan_topic = '/laser_scan_fuse'
        node.counts = {'/clock': 2, '/odom': 2, node.scan_topic: 2}
        clock = Clock()
        clock.clock.sec = 12
        odom = Odometry()
        odom.header.stamp.sec = 12
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        scan = LaserScan()
        scan.header.stamp.sec = 12
        scan.header.frame_id = 'fuse_lidar_link'
        scan.angle_increment = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [1.0]
        node.messages = {'/clock': clock, '/odom': odom, node.scan_topic: scan}
        now = time.monotonic()
        node.received = dict.fromkeys(node.counts, now)
        node.clock_advanced = True
        node.tf.can_transform.return_value = True
        node.tf.lookup_transform.return_value = Mock(header=Mock(stamp=clock.clock))
        node.states = dict.fromkeys(diagnostic_module.LIFECYCLE_NODES, 'active')
        node.lifecycle_nodes = list(diagnostic_module.LIFECYCLE_NODES)
        node.sim_times = dict.fromkeys(diagnostic_module.LIFECYCLE_NODES, True)
        node.checked_at = {(name, kind): now for name in diagnostic_module.LIFECYCLE_NODES
                           for kind in ['state', 'sim_time']}
        node.action.server_is_ready.return_value = True
        node.get_publishers_info_by_topic.return_value = [Mock(
            topic_type='geometry_msgs/msg/Twist', node_name='velocity_smoother')]
        node.get_subscriptions_info_by_topic.return_value = [Mock(topic_type='geometry_msgs/msg/Twist')]
        return node

    def test_readiness_and_stale_lifecycle(self):
        node = self.healthy_diagnostic()
        self.assertEqual(diagnostic_module.NavigationReadiness.problems(node), [])
        node.checked_at[('bt_navigator', 'state')] -= 10
        self.assertTrue(any('Lifecycle /bt_navigator' in error
                            for error in diagnostic_module.NavigationReadiness.problems(node)))

    def test_optional_world_obstacle_server_must_also_be_active(self):
        node = self.healthy_diagnostic()
        node.lifecycle_nodes.append('world_obstacle_map_server')
        errors = diagnostic_module.NavigationReadiness.problems(node)
        self.assertTrue(any('Lifecycle /world_obstacle_map_server' in error for error in errors))

    def test_detects_sensor_frame_stamp_qos_and_command_interface_problems(self):
        node = self.healthy_diagnostic()
        scan = node.messages[node.scan_topic]
        scan.header.frame_id = 'wrong_lidar'
        scan.header.stamp.sec = 0
        scan.ranges = [float('inf')]
        node.get_publishers_info_by_topic.return_value = [Mock(
            topic_type='geometry_msgs/msg/TwistStamped', node_name='velocity_smoother')]
        errors = diagnostic_module.NavigationReadiness.problems(node)
        for expected in ['timestamp', 'LaserScan frame', 'no finite', 'Twist publisher']:
            self.assertTrue(any(expected in error for error in errors), errors)


if __name__ == '__main__':
    unittest.main(verbosity=2)
