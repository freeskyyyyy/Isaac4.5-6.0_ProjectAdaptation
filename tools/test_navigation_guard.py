import unittest
import navigation_guard
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

    def test_command_guard_accepts_configured_limits(self):
        self.assertIsNone(navigation_guard.command_reason([.08, -.08, .15]))

    def test_command_guard_rejects_recorded_runaway(self):
        self.assertIsNotNone(navigation_guard.command_reason([.0818149431, .0007, .0006]))

    def test_command_guard_rejects_each_axis_and_nonfinite(self):
        for value in ([.081, 0., 0.], [0., -.081, 0.], [0., 0., .151],
                      [float("nan"), 0., 0.], [0., float("inf"), 0.]):
            with self.subTest(value=value):
                self.assertIsNotNone(navigation_guard.command_reason(value))

    def test_nonfinite_geometry_is_not_accepted(self):
        self.assertIsNotNone(guard_reason(float("nan"), .0, .0, .0, .0))

    def test_projection_uses_segment(self):
        self.assertAlmostEqual(tracking_error([.5, .03], [[0, 0], [1, 0]]), .03)

    def test_capture_wait_uses_both_simulation_freshness_and_wall_watchdog(self):
        wait_allowed = getattr(navigation_guard, 'capture_tf_wait_allowed', None)
        self.assertTrue(callable(wait_allowed), 'Capture-time wait must have explicit simulation and wall limits')
        # Recorded renderer stall: 0.62 wall seconds elapsed but only 0.05 sim seconds.
        self.assertTrue(wait_allowed(96.55, 96.60, .62))
        # Fast simulation, paused simulation, rewind and invalid observations all stop waiting.
        for args in [(96.55, 96.71, .1), (96.55, 96.55, 1.5),
                     (96.55, 96.50, .1), (96.55, float('nan'), .1),
                     (96.55, 96.56, float('inf'))]:
            with self.subTest(args=args):
                self.assertFalse(wait_allowed(*args))

    def test_recorded_replan_uses_accepted_path_and_same_path_growth(self):
        observe = getattr(navigation_guard, 'tracking_observation', None)
        self.assertTrue(callable(observe), 'Tracking must distinguish active and original paths')
        point = [8.223450660705566, 2.0352017879486084]
        previous = [8.224760055541992, 2.035240411758423]
        active = [[8.224508705927596, 2.0510695462646185], [8.216339182981958, 2.050713093179603]]
        initial = [[8.250000137835741, 2.050000187009573], [8.243750137742609, 2.0562501871027052]]
        values = observe(point, previous, .03333333399996263, active, initial)
        self.assertAlmostEqual(values['tracking_error_m'], .01580655486696813)
        self.assertAlmostEqual(values['initial_path_error_m'], .029237363642416338)
        self.assertEqual(values['outward_speed_m_s'], 0.)
        self.assertIsNone(guard_reason(1.216, values['tracking_error_m'], .0394, .0167,
                                       values['outward_speed_m_s']))

    def test_recorded_odom_render_stall_is_not_simulation_feedback_loss(self):
        reason = getattr(navigation_guard, 'odometry_feedback_reason', None)
        self.assertTrue(callable(reason), 'Odometry watchdog must use source time and both clocks')
        self.assertIsNone(reason(2308.416666666, 2308.416666666,
                                 2308.416666666, .53363464))
        # /odom can arrive one 60Hz SDK tick before the corresponding /clock.
        self.assertIsNone(reason(2308.433333333, 2308.416666666,
                                 2308.416666666, .01))

    def test_odom_loss_pause_rewind_and_invalid_stamps_still_stop(self):
        reason = getattr(navigation_guard, 'odometry_feedback_reason', None)
        self.assertTrue(callable(reason), 'Odometry watchdog must fail closed')
        for args in [(10., 10., 10.16, .1), (9.8, 10., 10., .01),
                     (10., 10., 10., 1.5), (10., 10., 9., .01),
                     (10.2, 10., 10., .01), (0., 10., 10., .01),
                     (10., 10., 10., -.01), (float('nan'), 10., 10., .01),
                     (10., 10., 10., float('inf'))]:
            with self.subTest(args=args):
                self.assertIsNotNone(reason(*args))

    def test_replan_jump_is_not_robot_drift_but_real_drift_still_stops(self):
        observe = getattr(navigation_guard, 'tracking_observation', None)
        self.assertTrue(callable(observe), 'Tracking must use one path for both positions')
        old = [[0., 0.], [1., 0.]]
        shifted = [[0., .03], [1., .03]]
        values = observe([.02, 0.], [.01, 0.], .1, shifted, old)
        self.assertAlmostEqual(values['tracking_error_m'], .03)
        self.assertEqual(values['outward_speed_m_s'], 0.)
        values = observe([.02, .015], [.01, .005], .1, old, old)
        self.assertAlmostEqual(values['outward_speed_m_s'], .1)
        self.assertIsNotNone(guard_reason(.9, values['tracking_error_m'], .04, .05,
                                          values['outward_speed_m_s']))


if __name__ == '__main__':
    unittest.main()
