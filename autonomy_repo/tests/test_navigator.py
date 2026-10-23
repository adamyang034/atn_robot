import sys
from pathlib import Path
import unittest

import numpy as np
import rclpy
from scipy.interpolate import splev

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from astar import AStar, DetOccupancyGrid2D
from navigator import Navigator
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState


class NavigationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Navigator()
        self.grid = StochOccupancyGrid2D(
            resolution=0.05, size_xy=np.array([100, 100]),
            origin_xy=np.array([-2., -2.]), window_size=9,
            probs=np.zeros(10000, dtype=int),
        )
        self.start = TurtleBotState(x=0., y=0., theta=0.)
        self.goal = TurtleBotState(x=1., y=0., theta=0.)

    def tearDown(self):
        self.node.destroy_node()

    def plan(self, goal=None, occupancy=None):
        return self.node.compute_trajectory_plan(
            self.start, goal if goal is not None else self.goal,
            occupancy if occupancy is not None else self.grid,
            resolution=0.1, horizon=4.,
        )

    def test_heading_wraps_across_pi(self):
        state = TurtleBotState(theta=np.pi - 0.1)
        goal = TurtleBotState(theta=-np.pi + 0.1)
        result = self.node.compute_heading_control(state, goal)
        self.assertAlmostEqual(result.omega, 0.4)
        self.assertEqual(result.v, 0.)

    def test_astar_routes_around_obstacle(self):
        occupancy = DetOccupancyGrid2D(4, 4, [((1., 0.), (2., 2.))])
        planner = AStar((0., 0.), (4., 4.), (0., 1.), (3., 1.), occupancy, 0.25)
        self.assertTrue(planner.solve())
        self.assertEqual(planner.path[0], planner.x_init)
        self.assertEqual(planner.path[-1], planner.x_goal)
        self.assertTrue(all(planner.is_free(p) for p in planner.path))
        self.assertTrue(any(p[1] > 2. for p in planner.path))

    def test_astar_reports_no_path(self):
        occupancy = DetOccupancyGrid2D(4, 4, [((1., 0.), (2., 4.))])
        planner = AStar((0., 0.), (4., 4.), (0., 1.), (3., 1.), occupancy, 0.25)
        self.assertFalse(planner.solve())
        self.assertIsNone(planner.path)

    def test_stochastic_map_and_constant_speed_splines(self):
        plan = self.plan()
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.duration, 1. / 0.15)
        self.assertTrue(all(self.grid.is_free(p) for p in plan.smoothed_path()))
        for t in (0., plan.duration / 2, plan.duration):
            self.assertAlmostEqual(float(splev(t, plan.path_x_spline)), 0.15 * t)
            self.assertAlmostEqual(float(splev(t, plan.path_y_spline)), 0.)
            self.assertAlmostEqual(float(splev(t, plan.path_x_spline, der=1)), 0.15)

    def test_short_and_blocked_paths_fail(self):
        self.assertIsNone(self.plan(TurtleBotState(x=0.2)))
        blocked = DetOccupancyGrid2D(4, 4, [((0.8, -1.), (1.2, 1.))])
        self.assertIsNone(self.plan(occupancy=blocked))

    def test_replanning_resets_controller_history(self):
        self.node.t_prev, self.node.v_prev, self.node.om_prev = 30., 0.2, 0.3
        self.assertIsNotNone(self.plan())
        self.assertEqual(self.node.t_prev, 0.)
        self.assertEqual(self.node.om_prev, 0.)
        self.assertAlmostEqual(self.node.v_prev, 0.15)

    def test_tracking_feedforward_and_limits(self):
        plan = self.plan()
        t = plan.duration / 2
        state = plan.desired_state(t)
        control = self.node.compute_trajectory_tracking_control(state, plan, t)
        self.assertAlmostEqual(control.v, 0.15)
        self.assertAlmostEqual(control.omega, 0.)
        displaced = TurtleBotState(x=-10., y=10., theta=0.)
        control = self.node.compute_trajectory_tracking_control(displaced, plan, t + 0.1)
        self.assertTrue(np.isfinite([control.v, control.omega]).all())
        self.assertLessEqual(abs(control.v), self.node.v_max)
        self.assertLessEqual(abs(control.omega), self.node.om_max)

    def test_tracking_reduces_position_error(self):
        plan = self.plan()
        state = TurtleBotState(x=-0.05, y=0.05, theta=0.)
        for t in np.arange(0., plan.duration, 0.05):
            control = self.node.compute_trajectory_tracking_control(state, plan, float(t))
            state.x += control.v * np.cos(state.theta) * 0.05
            state.y += control.v * np.sin(state.theta) * 0.05
            state.theta += control.omega * 0.05
        self.assertLess(np.hypot(state.x - 1., state.y), 0.03)


if __name__ == '__main__':
    unittest.main(verbosity=2)
