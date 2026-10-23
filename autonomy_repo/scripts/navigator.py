#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from scipy.interpolate import splev, splrep

from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotControl

from astar import AStar


class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__()
        self.heading_kp = 2.0
        self.kpx = self.kpy = 2.0
        self.kdx = self.kdy = 2.0
        self.v_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0
        self.v_prev_thresh = 0.0001

    def compute_heading_control(self, state, goal):
        control = TurtleBotControl()
        control.omega = float(self.heading_kp * wrap_angle(goal.theta - state.theta))
        return control

    def compute_trajectory_tracking_control(self, state, plan, t):
        t = min(max(t, 0.0), plan.duration)
        dt = max(t - self.t_prev, 0.0)

        x_d = splev(t, plan.path_x_spline)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        y_d = splev(t, plan.path_y_spline)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        v_for_heading = max(self.v_prev, self.v_prev_thresh)
        xd = v_for_heading * np.cos(state.theta)
        yd = v_for_heading * np.sin(state.theta)
        u1 = xdd_d + self.kpx * (x_d - state.x) + self.kdx * (xd_d - xd)
        u2 = ydd_d + self.kpy * (y_d - state.y) + self.kdy * (yd_d - yd)

        acceleration = np.cos(state.theta) * u1 + np.sin(state.theta) * u2
        omega = (-np.sin(state.theta) * u1 + np.cos(state.theta) * u2) / v_for_heading
        v = np.clip(v_for_heading + acceleration * dt, -self.v_max, self.v_max)
        omega = np.clip(omega, -self.om_max, self.om_max)

        self.t_prev = t
        self.v_prev = v
        self.om_prev = omega
        return TurtleBotControl(v=float(v), omega=float(omega))

    def compute_trajectory_plan(self, state, goal, occupancy, resolution, horizon):
        center_x = (state.x + goal.x) / 2
        center_y = (state.y + goal.y) / 2
        half = horizon / 2
        lower = (center_x - half, center_y - half)
        upper = (center_x + half, center_y + half)
        start = (state.x, state.y)
        finish = (goal.x, goal.y)

        astar = AStar(lower, upper, start, finish, occupancy, resolution)
        if not astar.solve() or len(astar.path) < 4:
            return None

        path = np.asarray(astar.path)
        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        ts = np.concatenate(([0], np.cumsum(segment_lengths) / 0.15))
        path_x_spline = splrep(ts, path[:, 0], s=0.05)
        path_y_spline = splrep(ts, path[:, 1], s=0.05)
        plan = TrajectoryPlan(path, path_x_spline, path_y_spline, ts[-1])

        if not all(occupancy.is_free(point) for point in plan.smoothed_path()):
            return None

        self.t_prev = 0.0
        self.v_prev = float(np.hypot(
            splev(0, path_x_spline, der=1),
            splev(0, path_y_spline, der=1),
        ))
        self.om_prev = 0.0
        return plan


if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
