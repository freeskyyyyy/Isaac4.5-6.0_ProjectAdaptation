#!/usr/bin/env python3
"""Gate an explicit grasp request on available navigation/planning/detection servers."""
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from eco65b_grasp_interfaces.action import DetectObject, GraspToPoint, GripperControl
from moveit_msgs.action import MoveGroup
from nav2_msgs.action import NavigateToPose


def main():
    rclpy.init()
    node = Node('course_action_readiness')
    clients = [(name, ActionClient(node, kind, name)) for name, kind in [
        ('/detect_object', DetectObject), ('/grasp_to_point', GraspToPoint),
        ('/gripper_control', GripperControl), ('/move_action', MoveGroup),
        ('/navigate_to_pose', NavigateToPose),
    ]]
    deadline = time.monotonic() + 60
    missing = []
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = [name for name, client in clients if not client.server_is_ready()]
            if not missing:
                print('Course action servers are ready; explicit task may start.')
                return
        raise SystemExit('Action servers not ready: ' + ', '.join(missing))
    finally:
        for _, client in clients:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
