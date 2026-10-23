# HW2 section prep: ROS2 navigation

`scripts/navigator.py` implements the three `BaseNavigator` methods required by
HW2: wrapped-angle heading control, differential-flatness trajectory tracking,
and A* planning followed by cubic spline smoothing. `astar.py` and `utils.py`
contain the planner and its supporting functions. New plans reset the tracking
controller history; failed plans and paths with fewer than four points return
`None`.

Clone the repository's `section5` branch into your course workspace. If you
already have the repository, fetch and switch to that branch instead of cloning
another copy. The package will be at `~/autonomy_ws/src/atn_robot/autonomy_repo`.

```bash
mkdir -p ~/autonomy_ws/src
cd ~/autonomy_ws/src
git clone --branch section5 https://github.com/nolanxdon/atn_robot.git
```

In a terminal with the course ROS2 environment loaded, build and launch it:

```bash
source /opt/ros/humble/setup.bash
source ~/tb_ws/install/local_setup.bash
cd ~/autonomy_ws
colcon build --symlink-install --packages-select autonomy_repo
source install/local_setup.bash
ros2 launch autonomy_repo navigator.launch.py
```

Start the course simulator separately as directed in section. Use RViz's
**2D Goal Pose** tool to select a target; the relay sends it to `/cmd_nav`.
The node publishes `/planned_path`, `/smoothed_path`, and `/nav_success`.
For a robot using wall-clock time, launch with `use_sim_time:=false` after
starting the robot's driver and mapping nodes as directed in section.

The local Lima VM loads the course packages from `~/tb_ws/install/local_setup.bash`
as shown above. On a course laptop, use the course's environment setup instead.

To rerun the navigation checks in the ROS2 environment:

```bash
cd ~/autonomy_ws
python3 -m unittest discover -s src/atn_robot/autonomy_repo/tests -v
```

Verified on October 23, 2026 in the local ROS2 Humble VM: `colcon build`, launch
argument loading, and eight checks covering heading wraparound, obstacle
routing, unreachable goals, stochastic occupancy grids and spline timing,
short/blocked path rejection, controller resets, tracking limits, and tracking
convergence. This verification did not use a physical robot.
