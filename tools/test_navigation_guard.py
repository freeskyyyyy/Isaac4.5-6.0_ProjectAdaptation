import unittest
from navigation_guard import stopping_reserve, tracking_error, guard_reason


class NavigationGuardTest(unittest.TestCase):
    def test_reserve_grows_with_speed_and_feedback_age(self):
        self.assertGreater(stopping_reserve(.11, .15), stopping_reserve(.04, .05))

    def test_stop_before_clearance_limit(self):
        self.assertIsNotNone(guard_reason(.62, .01, .11, .15, .0))

    def test_tracking_stops_before_five_centimeters(self):
        self.assertIsNotNone(guard_reason(.9, .04, .08, .1, .04))

    def test_safe_stationary_sample(self):
        self.assertIsNone(guard_reason(.64, .005, .0, .05, .0))

    def test_projection_uses_segment(self):
        self.assertAlmostEqual(tracking_error([.5, .03], [[0, 0], [1, 0]]), .03)


if __name__ == '__main__':
    unittest.main()
