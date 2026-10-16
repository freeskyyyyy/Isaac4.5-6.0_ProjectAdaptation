"""Offline timing diagnostics must separate emergency overrides and native clocks."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))

class ControlFeedbackTest(unittest.TestCase):
    def test_emergency_derivative_excluded_from_ordinary_metrics(self):
        from analyze_control_feedback import summarize
        rows = [dict(sim_seconds=t, wall_seconds=t*2, xy_wz=[v,0,0])
                for t,v in [(1.,.01),(1.05,.015),(1.1,0.)]]
        result=summarize(rows,'sim_seconds',1.075)
        self.assertAlmostEqual(result['ordinary']['maximum_absolute_sample_derivative_xy_wz'][0],.1)
        self.assertAlmostEqual(result['all']['maximum_absolute_sample_derivative_xy_wz'][0],.3)
        self.assertAlmostEqual(result['ordinary']['median_sim_dt_s'],.05)
        self.assertAlmostEqual(result['ordinary']['median_wall_dt_s'],.1)

    def test_nonmonotonic_intervals_not_differentiated(self):
        from analyze_control_feedback import summarize
        rows=[dict(sim_seconds=t,wall_seconds=t,xy_wz=[v,0,0])
              for t,v in [(1.,0),(1.,10),(1.1,10)]]
        result=summarize(rows,'sim_seconds',2.)['all']
        self.assertEqual(result['nonpositive_intervals'],1)
        self.assertEqual(result['maximum_absolute_sample_derivative_xy_wz'],[0.,0.,0.])

if __name__ == '__main__': unittest.main()
