# Camera Navigation with ROS2 Nav2 — Simulation Tutorial

A beginner-friendly ROS2 package for learning autonomous navigation using a simulated depth camera (equivalent to Intel RealSense D455F) in Gazebo Classic.

## For Students

Read the step-by-step tutorial first:

👉 [`tutorial_sim_nav2.md`](tutorial_sim_nav2.md)

The tutorial walks you through every command from installation to sending autonomous navigation goals.

## Quick Reference — Command Sequence

### First time: install packages
```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup ros-humble-navigation2 \
  ros-humble-slam-toolbox ros-humble-depthimage-to-laserscan \
  ros-humble-teleop-twist-keyboard ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-tf2-tools \
  python3-colcon-common-extensions
```

### Build the workspace
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone <this-repo-url>
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### Phase A — Mapping (run once to create the map)
```bash
# T1 — Gazebo + robot + RViz2
ros2 launch my_robot_sim sim_launch.py

# T2 — SLAM
ros2 launch my_robot_sim slam_launch.py

# T3 — Keyboard drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# T4 — Save map when arena is fully explored
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/my_robot_sim/maps/arena_map
```

### Phase B — Autonomous Navigation
```bash
# T1 — Gazebo + robot + RViz2 (still running)
ros2 launch my_robot_sim sim_launch.py

# T2 — Nav2 with saved map
ros2 launch my_robot_sim nav2_launch.py

# In RViz2: click "2D Pose Estimate", then "Nav2 Goal"
# OR run the Python waypoint script:
python3 src/my_robot_sim/scripts/go_to_goal.py
```

## Package Structure

```
my_robot_sim/
├── urdf/
│   └── robot.urdf.xacro      # robot model + Gazebo depth camera plugin
├── worlds/
│   └── arena.world           # 6 m x 6 m arena with obstacles
├── config/
│   └── nav2_params.yaml      # Nav2 + SLAM Toolbox parameters
├── launch/
│   ├── sim_launch.py         # Gazebo + robot + depth-to-scan + RViz2
│   ├── slam_launch.py        # SLAM Toolbox (mapping phase)
│   └── nav2_launch.py        # Full Nav2 stack (navigation phase)
├── rviz/
│   └── nav2_sim.rviz         # pre-configured RViz2 layout
├── scripts/
│   └── go_to_goal.py         # Python script to send waypoints
└── maps/                     # generated map files go here (gitignored)
```

## Hardware (future)

When you are ready to use the real Intel RealSense D455F, see
[`tutorial_camera_nav2.md`](tutorial_camera_nav2.md) for the hardware-specific steps.
The Nav2 config and Python scripts carry over unchanged.

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic (ships with `ros-humble-gazebo-ros-pkgs`)
