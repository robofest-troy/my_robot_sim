# Camera Navigation in Simulation — ROS2 Nav2 + Gazebo Classic
### Step-by-Step Tutorial for High School Students (Simulation First)

**Goal:** Get autonomous navigation working entirely inside a simulator before touching any real hardware.  
**Software:** Ubuntu 22.04, ROS2 Humble, Gazebo Classic, Nav2, SLAM Toolbox  
**Simulated camera:** A depth camera plugin that behaves like the D455F

---

## Why Simulate First?

- You can crash the robot as many times as you want — nothing breaks
- No USB cables, no drivers, no hardware setup required
- You can pause, rewind, and restart the world instantly
- When the real D455F is connected later, almost nothing in your Nav2 config changes

---

## Big Picture

```
Gazebo (virtual world)
    │
    ├── Simulated depth camera  →  /camera/depth/image_raw
    │                               /camera/depth/points
    ├── Simulated wheel encoders →  /odom
    └── Accepts /cmd_vel         ←  Nav2 Controller

depthimage_to_laserscan → /scan

SLAM Toolbox  →  /map  (while mapping)
AMCL          →  /amcl_pose  (after map is saved)

Nav2 stack:
  Costmap → Planner → Controller → /cmd_vel → Gazebo wheels
```

---

## Step 1 — Install All Required Packages

Open a terminal and run these commands one block at a time.

### 1.1 Core simulation and Nav2 packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-rviz-plugins \
  ros-humble-slam-toolbox \
  ros-humble-depthimage-to-laserscan \
  ros-humble-teleop-twist-keyboard \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions
```

### 1.2 Verify Gazebo launches

```bash
source /opt/ros/humble/setup.bash
gazebo
```

An empty grey world should appear. Close it.

---

## Step 2 — Create a ROS2 Workspace and Package

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
ros2 pkg create --build-type ament_cmake my_robot_sim \
  --dependencies rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros
```

Inside `my_robot_sim`, create these folders:

```bash
cd ~/robot_ws/src/my_robot_sim
mkdir -p urdf launch worlds config maps rviz
```

---

## Step 3 — Build a Simple Robot Description (URDF)

The URDF is a text file that describes the robot's shape, joints, and sensors to both ROS2 and Gazebo.

Create `urdf/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- ── Base link ─────────────────────────────────── -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="blue"><color rgba="0.2 0.4 0.8 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.08" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- ── Camera link ───────────────────────────────── -->
  <!-- mounted 15 cm forward, 15 cm above base_link center -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry><box size="0.05 0.09 0.03"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.09 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- optical frame: Gazebo camera plugin uses Z-forward convention -->
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>
  <link name="camera_optical_link"/>

  <!-- ── Gazebo depth camera plugin ───────────────── -->
  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.5</horizontal_fov>  <!-- ~86 deg, close to D455F -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.2</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>~/image_raw:=/camera/color/image_raw</remapping>
          <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
          <remapping>~/points:=/camera/depth/points</remapping>
          <remapping>~/camera_info:=/camera/color/camera_info</remapping>
          <remapping>~/depth/camera_info:=/camera/depth/camera_info</remapping>
        </ros>
        <frame_name>camera_optical_link</frame_name>
        <min_depth>0.2</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ── Differential drive plugin ────────────────── -->
  <!-- Mecanum sim is complex; diff-drive teaches the same Nav2 concepts -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <remapping>cmd_vel:=/cmd_vel</remapping>
        <remapping>odom:=/odom</remapping>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>2.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- ── Wheels (needed by diff_drive plugin) ──────── -->
  <link name="left_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.04"/></geometry></visual>
    <collision><geometry><cylinder radius="0.05" length="0.04"/></geometry></collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.04"/></geometry></visual>
    <collision><geometry><cylinder radius="0.05" length="0.04"/></geometry></collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.003" iyy="0.003" izz="0.005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ── Caster (front, passive) ───────────────────── -->
  <link name="caster">
    <visual><geometry><sphere radius="0.025"/></geometry></visual>
    <collision><geometry><sphere radius="0.025"/></geometry></collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="0.15 0 -0.075"/>
  </joint>
  <gazebo reference="caster">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>

</robot>
```

> **Why differential drive instead of mecanum?**  
> The Gazebo diff-drive plugin is simpler and teaches the same Nav2 concepts.  
> When you move to the real mecanum robot, you only change the `/cmd_vel` subscriber in your CAN bridge — Nav2 stays identical.

---

## Step 4 — Create a Simple Gazebo World

Create `worlds/arena.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="arena">

    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <!-- outer walls -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <model name="wall_south">
      <static>true</static>
      <pose>0 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <model name="wall_east">
      <static>true</static>
      <pose>3 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <model name="wall_west">
      <static>true</static>
      <pose>-3 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <!-- a couple of interior obstacles -->
    <model name="box1">
      <static>true</static>
      <pose>1 1 0.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.4 0.4 0.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.4 0.4 0.5</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="box2">
      <static>true</static>
      <pose>-1 -1 0.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.4 0.4 0.5</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>0.4 0.4 0.5</size></box></geometry>
          <material><ambient>0.2 0.8 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## Step 5 — Create the Nav2 Parameters File

Create `config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    base_frame_id: base_footprint
    beam_skip_distance: 0.5
    global_frame_id: map
    laser_model_type: likelihood_field
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: odom
    scan_topic: /scan

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      min_vel_x: 0.0
      max_vel_x: 0.4
      min_vel_y: 0.0
      max_vel_y: 0.0
      min_speed_xy: 0.0
      max_speed_xy: 0.4
      min_vel_theta: -1.0
      max_vel_theta: 1.0
      min_speed_theta: 0.4
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.4
        cost_scaling_factor: 3.0

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      robot_base_frame: base_link
      global_frame: map
      update_frequency: 1.0
      publish_frequency: 1.0
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.4
        cost_scaling_factor: 3.0

planner_server:
  ros__parameters:
    use_sim_time: true
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: false

recoveries_server:
  ros__parameters:
    use_sim_time: true

slam_toolbox:
  ros__parameters:
    use_sim_time: true
    solver_plugin: solver_plugins::CeresSolver
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 8.0
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
```

---

## Step 6 — Create the RViz2 Config

Create `rviz/nav2_sim.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
  - Class: rviz_common/Views
Global Options:
  Fixed Frame: map
Displays:
  - Class: rviz_default_plugins/Map
    Name: Map
    Topic:
      Value: /map
  - Class: rviz_default_plugins/LaserScan
    Name: Scan
    Topic:
      Value: /scan
    Size (m): 0.04
    Color: 255; 0; 0
  - Class: rviz_default_plugins/PointCloud2
    Name: DepthCloud
    Topic:
      Value: /camera/depth/points
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
    Description Topic:
      Value: /robot_description
  - Class: nav2_rviz_plugins/Nav2Panel
    Name: Navigation
  - Class: rviz_default_plugins/TF
    Name: TF
    Enabled: false
```

---

## Step 7 — Create Launch Files

### 7.1 Gazebo + Robot launch (`launch/sim_launch.py`)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')

    # process the xacro file into plain URDF XML
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Gazebo with our world
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 os.path.join(pkg, 'worlds', 'arena.world'),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),

        # Robot state publisher — broadcasts TF frames
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'my_robot',
                '-x', '0', '-y', '0', '-z', '0.05',
            ],
            output='screen',
        ),

        # depth image → laser scan
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'use_sim_time': True,
                'scan_height': 10,
                'scan_time': 0.033,
                'range_min': 0.2,
                'range_max': 8.0,
                'output_frame': 'camera_optical_link',
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_sim.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
```

### 7.2 SLAM launch (`launch/slam_launch.py`)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(pkg, 'config', 'nav2_params.yaml'),
                {'use_sim_time': True},
            ],
            output='screen',
        ),
    ])
```

### 7.3 Nav2 (navigation) launch (`launch/nav2_launch.py`)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': os.path.join(pkg, 'maps', 'arena_map.yaml'),
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            }.items(),
        ),
    ])
```

---

## Step 8 — Build the Workspace

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

If you see any missing dependency errors, install them with `sudo apt install ros-humble-<package-name>`.

---

## Step 9 — Phase A: Mapping (Do This First)

Open **4 terminals**. Source ROS2 in every terminal:

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
```

**Terminal 1** — Start Gazebo + robot + depth-to-scan + RViz2:
```bash
ros2 launch my_robot_sim sim_launch.py
```
Wait until you see the Gazebo window open and the robot appear in RViz2.

**Terminal 2** — Start SLAM Toolbox:
```bash
ros2 launch my_robot_sim slam_launch.py
```
You should see `Map` topic appear in RViz2 and a grey grid start to fill in.

**Terminal 3** — Drive the robot with the keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i` (forward), `,` (backward), `j`/`l` (turn). Drive slowly around the entire arena so the map gets filled.

**Terminal 4** — Watch the scan topic to confirm depth-to-scan is working:
```bash
ros2 topic hz /scan
```
You should see ~30 Hz.

When the map looks complete (all walls visible, no big unknown patches):

**Save the map** (Terminal 4):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/my_robot_sim/maps/arena_map
```

This creates `arena_map.pgm` and `arena_map.yaml` in the maps folder.

**Stop SLAM Toolbox** (Ctrl+C in Terminal 2) — you will use Nav2 now instead.

---

## Step 10 — Phase B: Autonomous Navigation

**Terminal 1** — Gazebo + robot is still running. If not, relaunch:
```bash
ros2 launch my_robot_sim sim_launch.py
```

**Terminal 2** — Start Nav2 with the saved map:
```bash
ros2 launch my_robot_sim nav2_launch.py
```
Wait for the message `Navigation is active` in the terminal output.

**In RViz2:**

1. Click **"2D Pose Estimate"** in the Nav2 panel (or the toolbar)  
   → Click on the map where the robot currently is, drag in the direction it faces  
   → AMCL will scatter green particles and converge on the correct pose

2. Click **"Nav2 Goal"** (or "Navigation2 Goal" in the toolbar)  
   → Click anywhere on the free (white) space in the map  
   → Watch the robot plan a path (green line) and drive to the goal

---

## Step 11 — Send Goals from Python

Create `~/robot_ws/src/my_robot_sim/scripts/go_to_goal.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_to(self, x, y, yaw_deg=0.0):
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        half = math.radians(yaw_deg) / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal.pose = pose

        self.get_logger().info(f'Sending goal: x={x} y={y}')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Reached goal!')


def main():
    rclpy.init()
    nav = Navigator()

    # drive to a sequence of waypoints
    waypoints = [
        (1.5,  1.5,   0),
        (1.5, -1.5,  90),
        (-1.5, -1.5, 180),
        (-1.5,  1.5, 270),
        (0.0,  0.0,   0),   # return home
    ]
    for x, y, yaw in waypoints:
        nav.go_to(x, y, yaw)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it while Nav2 is active:
```bash
python3 ~/robot_ws/src/my_robot_sim/scripts/go_to_goal.py
```

The robot will visit all five waypoints in sequence.

---

## Step 12 — Inspect What's Happening (Diagnostic Commands)

These commands help you understand and debug the system. Run them any time in a spare terminal.

```bash
# See all active topics
ros2 topic list

# Check the depth image is publishing
ros2 topic hz /camera/depth/image_raw

# Check the scan is publishing
ros2 topic hz /scan

# See the TF tree (are all frames connected?)
ros2 run tf2_tools view_frames
# opens a PDF: frames.pdf — open it with: open frames.pdf

# Print the robot's current map position
ros2 topic echo /amcl_pose --once

# Print the current navigation action status
ros2 action list
ros2 action info /navigate_to_pose
```

---

## Step 13 — Moving to the Real D455F (What Changes)

When you are ready to use the real camera:

| What | In simulation | On real robot |
|---|---|---|
| Camera source | Gazebo plugin | `ros2 launch realsense2_camera rs_launch.py` |
| Depth topic | `/camera/depth/image_raw` | `/camera/camera/depth/image_rect_raw` |
| Camera info topic | `/camera/depth/camera_info` | `/camera/camera/depth/camera_info` |
| `use_sim_time` | `true` | `false` |
| `odom` source | Gazebo diff-drive plugin | Your CAN-based odometry publisher |
| `cmd_vel` sink | Gazebo diff-drive plugin | Your CAN motor controller |

Everything else — SLAM Toolbox, AMCL, Nav2 params, your Python waypoint script — stays the same.

---

## Practice Workflow (Do This Every Session)

```
1. Open 4 terminals, source ROS2 in each

--- MAPPING phase (first time only) ---
2. T1: ros2 launch my_robot_sim sim_launch.py
3. T2: ros2 launch my_robot_sim slam_launch.py
4. T3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
5. Drive around the whole arena slowly
6. T4: map_saver_cli  (save the map)
7. Ctrl+C in T2

--- NAVIGATION phase ---
8. T2: ros2 launch my_robot_sim nav2_launch.py
9. In RViz2: set 2D Pose Estimate
10. In RViz2: click Nav2 Goal  OR  run go_to_goal.py
```

---

## Troubleshooting Cheat Sheet

| Problem | What to check |
|---|---|
| Gazebo opens but robot doesn't appear | Wait 10–15 s; check `spawn_entity.py` output for errors |
| `/scan` not publishing | Check `depthimage_to_laserscan` node is running; verify depth topic name matches |
| Map is all grey / not building | SLAM Toolbox not started; `/scan` not publishing; TF frames broken |
| Robot drifts off the map | Odometry not publishing; check `/odom` topic is active |
| Nav2 says "waiting for costmap" | Map server not loaded; check `arena_map.yaml` path is correct |
| AMCL particles not converging | Set 2D Pose Estimate manually in RViz2; drive the robot a little |
| Goal immediately cancelled | Nav2 lifecycle not fully started; wait for `Navigation is active` message |
| Python script: goal rejected | Nav2 not running, or wrong `frame_id` (must be `'map'`) |

---

## Glossary

| Term | Plain English |
|---|---|
| URDF / xacro | A file that describes what the robot looks like and how its parts connect |
| Gazebo plugin | Code that simulates a sensor or motor inside Gazebo |
| TF frame | A named coordinate system attached to a robot part (e.g. `base_link`, `camera_link`) |
| SLAM | Build a map and know your position at the same time |
| AMCL | Figure out where you are inside an already-built map |
| Costmap | A grid used by Nav2 to know which areas are safe to drive through |
| Action | A ROS2 communication pattern for long tasks like "drive to goal" that you can cancel |
| `use_sim_time` | Tell ROS2 to use Gazebo's clock instead of the real wall clock |
