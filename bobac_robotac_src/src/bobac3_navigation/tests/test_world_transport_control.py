"""World transport consistency; no ROS initialization or motion commands."""
import math
import sys
import numpy as np
from pathlib import Path
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class WorldTransportControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load((PACKAGE/'params/bobac3_navigation_world_params.yaml').read_text())
        cls.server = cls.params['controller_server']['ros__parameters']
        cls.mppi = cls.server['FollowPath']
        cls.smoother = cls.params['velocity_smoother']['ros__parameters']

    def test_local_and_global_envelopes_match(self):
        for name in ('local', 'global'):
            costmap = self.params[name+'_costmap'][name+'_costmap']['ros__parameters']
            self.assertNotIn('footprint', costmap)
            self.assertEqual(costmap['robot_radius'], .64)
            self.assertEqual(costmap['footprint_padding'], 0.)
            self.assertGreaterEqual(costmap['inflation_layer']['inflation_radius'], .85)

    def test_circular_cost_checks_every_prediction_step(self):
        critic = self.mppi['CostCritic']
        self.assertIn('CostCritic', self.mppi['critics'])
        self.assertNotIn('ObstaclesCritic', self.mppi['critics'])
        self.assertFalse(critic['consider_footprint'])
        self.assertEqual(critic['trajectory_point_step'], 1)
        self.assertGreaterEqual(critic['collision_cost'], 1e6)

    def test_velocity_and_acceleration_are_consistent(self):
        m, s = self.mppi, self.smoother
        self.assertEqual(s['feedback'], 'CLOSED_LOOP')
        self.assertTrue(s['scale_velocities'])
        self.assertFalse(m['open_loop'])
        self.assertEqual(s['max_velocity'], [m['vx_max'], m['vy_max'], m['wz_max']])
        self.assertEqual(s['min_velocity'], [m['vx_min'], -m['vy_max'], -m['wz_max']])
        self.assertEqual(s['max_accel'], [m['ax_max'], m['ay_max'], m['az_max']])
        self.assertEqual(s['max_decel'], [m['ax_min'], m['ay_min'], -m['az_max']])
        self.assertLessEqual(math.hypot(m['vx_max'], m['vy_max']), .15)
        self.assertLessEqual(m['wz_max'], .15)
        self.assertAlmostEqual(m['model_dt'], 1/self.server['controller_frequency'])
        self.assertEqual(s['smoothing_frequency'], self.server['controller_frequency'])
        # Idealized stop from max diagonal speed + command timeout, NOT measured physics.
        speed = math.hypot(m['vx_max'], m['vy_max'])
        decel = math.hypot(m['ax_min'], m['ay_min'])
        self.assertLess(speed*s['velocity_timeout'] + speed**2/(2*decel), .09)
        self.assertGreaterEqual(m['time_steps']*m['model_dt'], speed/decel+1.)

    def test_tracking_over_chasing_distant_goal(self):
        m = self.mppi
        self.assertGreaterEqual(m['PathAlignCritic']['cost_weight'], 20.)
        self.assertEqual(m['PathAlignCritic']['trajectory_point_step'], 1)
        self.assertLessEqual(m['PathAlignCritic']['offset_from_furthest'], 2)
        self.assertLessEqual(m['PathFollowCritic']['offset_from_furthest'], 2)
        self.assertLessEqual(m['GoalCritic']['threshold_to_consider'], .3)
        self.assertLessEqual(m['GoalCritic']['cost_weight'], m['PathAlignCritic']['cost_weight'])
        self.assertGreater(m['prune_distance'], math.hypot(m['vx_max'], m['vy_max'])*m['time_steps']*m['model_dt'])
        stopped = self.server['stopped_goal_checker']
        self.assertLessEqual(stopped['trans_stopped_velocity'], .02)
        self.assertLessEqual(stopped['rot_stopped_velocity'], .02)

    def test_supported_jazzy_smac_smoothing_bypass(self):
        planner = self.params['planner_server']['ros__parameters']['GridBased']
        self.assertNotIn('smooth_path', planner)
        self.assertEqual(planner['smoother']['max_iterations'], 0)
        self.assertFalse(planner['smoother']['do_refinement'])


class TransportTraceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(PACKAGE/'scripts'))
        from analyze_transport_trace import tracking_errors
        cls.errors = staticmethod(tracking_errors)

    def test_distance_to_segment_interior_and_endpoint(self):
        np.testing.assert_allclose(self.errors([[.5, .03], [1.1, 0]], [[0, 0], [1, 0]]), [.03, .1])

    def test_duplicate_reference_vertices(self):
        np.testing.assert_allclose(self.errors([[0, 0], [.5, 0]], [[0, 0], [0, 0], [1, 0]]), [0, 0])

    def test_reference_retains_measured_start_connector(self):
        from analyze_transport_trace import reference_points
        report = {'initial_localization': {'map_base_xy': [8, 1.082]},
                  'planner': {'points_xy': [[7.95, 1.05], [7.9, 1.1]]}}
        reference = reference_points(report)
        np.testing.assert_allclose(self.errors([[8, 1.082], [7.975, 1.066]], reference), [0, 0], atol=1e-12)

    def test_bad_trace_rejected(self):
        with self.assertRaises(ValueError):
            self.errors([[0, 0], [float('nan'), 1]], [[0, 0], [1, 0]])


if __name__ == '__main__':
    unittest.main(verbosity=2)
