#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserQoSRelay(Node):
    def __init__(self):
        super().__init__('laser_qos_relay',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])

        # BEST_EFFORT readers match both RELIABLE and BEST_EFFORT writers.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS for publishing (BEST_EFFORT to match Nav2 costmap)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Preserve the raw scan, header, timestamp and frame without conversion.
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_scan_fuse',
            self.laser_callback,
            sub_qos
        )

        # Publish to new topic with BEST_EFFORT QoS
        self.publisher = self.create_publisher(
            LaserScan,
            '/laser_scan_fuse_best_effort',
            pub_qos
        )

        self.get_logger().info('Laser QoS relay node started')
        self.get_logger().info('Subscribing to /laser_scan_fuse with BEST_EFFORT QoS')
        self.get_logger().info('Publishing to /laser_scan_fuse_best_effort with BEST_EFFORT QoS')

    def laser_callback(self, msg):
        # Simply relay the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserQoSRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
