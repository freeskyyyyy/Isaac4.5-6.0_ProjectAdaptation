"""Offline regressions: a plan success is not a clearance acceptance."""
import importlib.util
import json
import os
from pathlib import Path
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('clearance', PACKAGE/'scripts/check_path_clearance.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PathClearanceTest(unittest.TestCase):
    def test_saved_successful_plan_fails_transport_clearance(self):
        root = Path(os.environ.get('COURSE_PROJECT_ROOT', PACKAGE.parents[2]))
        data = json.loads((root/'reports/world-navigation-alignment-plan-3d.json').read_text())
        result = module.check(data['planner']['points_xy'], PACKAGE/'maps/world_scene.yaml')
        self.assertFalse(result['passed'])
        self.assertLess(result['minimum_center_clearance_lower_bound_m'], .19)
        self.assertLess(result['minimum_transport_margin_lower_bound_m'], 0)

    def test_reference_north_detour_has_reserve(self):
        result = module.check(module.REFERENCE_ROUTE, PACKAGE/'maps/world_scene.yaml')
        self.assertTrue(result['passed'], result)

    def test_segment_interior_is_checked(self):
        # Both endpoints are free, but their connecting segment crosses Wall02.
        result = module.check([[6.6, 1], [4.8, 1]], PACKAGE/'maps/world_scene.yaml')
        self.assertFalse(result['passed'])

    def test_unknown_and_outside_are_blocked(self):
        self.assertFalse(module.check([[-.8, -10.3], [-.7, -10.3]],
                                      PACKAGE/'maps/world_scene.yaml')['passed'])
        self.assertFalse(module.check([[11, 1], [12, 1]], PACKAGE/'maps/world_scene.yaml')['passed'])

    def test_invalid_input_is_rejected(self):
        for points in ([], [[1, 2]], [[float('nan'), 0], [1, 2]]):
            with self.assertRaises(ValueError):
                module.check(points, PACKAGE/'maps/world_scene.yaml')

    def test_global_planning_constraint_contains_transport_circle(self):
        params = yaml.safe_load((PACKAGE/'params/bobac3_navigation_world_params.yaml').read_text())
        costmap = params['global_costmap']['global_costmap']['ros__parameters']
        self.assertNotIn('footprint', costmap)
        self.assertGreaterEqual(costmap['robot_radius'], .64)
        self.assertEqual(costmap['footprint_padding'], 0.)
        self.assertGreaterEqual(costmap['inflation_layer']['inflation_radius'], .85)
        planner = params['planner_server']['ros__parameters']['GridBased']
        self.assertNotIn('smooth_path', planner)
        self.assertEqual(planner['smoother']['max_iterations'], 0)
        self.assertLessEqual(planner['tolerance'], .05)


if __name__ == '__main__':
    unittest.main(verbosity=2)
