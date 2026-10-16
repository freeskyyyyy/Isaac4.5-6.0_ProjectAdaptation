"""Regression for invalid physics joint data reaching the public startup gate."""
import sys
import unittest
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'bobac_robotac_src/src/course_bringup'))
from course_bringup.readiness import Readiness


class ReadinessValues(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def check_arm(self, field, values):
        node = Readiness()
        try:
            msg = JointState(name=[f'joint_{i}' for i in range(1, 7)], position=[0.] * 6)
            setattr(msg, field, values)
            node.receive('/arm/joint_states', msg)
            return [e for e in node.problems() if 'JointState' in e]
        finally:
            node.destroy_node()

    def test_finite_positions_with_optional_arrays_empty_are_valid(self):
        self.assertEqual(self.check_arm('position', [0.] * 6), [])

    def test_nan_positions_are_rejected(self):
        self.assertTrue(any('non-finite' in e for e in self.check_arm('position', [float('nan')] * 6)))

    def test_infinite_positions_are_rejected(self):
        self.assertTrue(any('non-finite' in e for e in self.check_arm('position', [float('inf')] * 6)))

    def test_nan_velocity_is_rejected(self):
        self.assertTrue(any('non-finite' in e for e in self.check_arm('velocity', [float('nan')] * 6)))

    def test_nan_effort_is_rejected(self):
        self.assertTrue(any('non-finite' in e for e in self.check_arm('effort', [float('nan')] * 6)))


if __name__ == '__main__':
    unittest.main()
