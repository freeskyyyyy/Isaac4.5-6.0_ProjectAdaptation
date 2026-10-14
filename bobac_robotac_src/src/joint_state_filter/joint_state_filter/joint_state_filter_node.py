import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')

        self.declare_parameter('input_topic', '/arm/joint_states')
        self.declare_parameter('output_topic', '/arm/joint_state_deal')
        self.declare_parameter('joints', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        joints_param = self.get_parameter('joints').get_parameter_value().string_array_value
        self.joints = [str(x) for x in joints_param]
        self.joint_set = set(self.joints)

        self.pub = self.create_publisher(JointState, self.output_topic, 10)
        self.sub = self.create_subscription(JointState, self.input_topic, self._on_js, qos_profile_sensor_data)

        self.get_logger().info(f'Filtering {self.input_topic} -> {self.output_topic}, joints={self.joints}')

    def _on_js(self, msg: JointState):
        out = JointState()
        out.header = msg.header

        name = []
        position = []
        velocity = []
        effort = []

        has_vel = len(msg.velocity) == len(msg.name)
        has_eff = len(msg.effort) == len(msg.name)

        for i, n in enumerate(msg.name):
            if n not in self.joint_set:
                continue
            name.append(n)
            if i < len(msg.position):
                position.append(msg.position[i])
            if has_vel:
                velocity.append(msg.velocity[i])
            if has_eff:
                effort.append(msg.effort[i])

        out.name = name
        out.position = position
        if velocity:
            out.velocity = velocity
        if effort:
            out.effort = effort

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
