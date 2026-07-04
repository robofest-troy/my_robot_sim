# Camera Navigation with ROS2 Nav2 and Intel RealSense D455F
### A Step-by-Step Tutorial for High School Students

**Hardware:** Intel RealSense D455F depth camera  
**Software:** Ubuntu 22.04, ROS2 Humble, Nav2, Gazebo Classic  
**Robot:** Mecanum drive, 4 wheels, Jetson Orin Nano

---

## What You Will Build

By the end of this tutorial your robot will be able to:
1. See the world in 3D using the D455F depth camera
2. Build a map of a room or arena (SLAM)
3. Localize itself inside that map
4. Autonomously navigate from point A to point B while avoiding obstacles

---

## Big Picture: How Visual Navigation Works

```
Camera (D455F)
    │
    ▼
Depth Image / Point Cloud
    │
    ▼
SLAM (mapping + localization)   ◄──── odometry from wheels
    │
    ▼
Costmap (where is free space?)
    │
    ▼
Nav2 Planner (plan a path)
    │
    ▼
Nav2 Controller (follow the path)
    │
    ▼
Motor Commands via CAN
```

**Key concepts:**
- **Depth image** – every pixel has a distance value (like a ruler for each point in the scene)
- **Point cloud** – a 3D collection of all those measured points
- **SLAM** – Simultaneous Localization and Mapping: build a map while tracking where you are
- **Costmap** – a grid that marks cells as free, occupied, or unknown
- **Nav2** – ROS2's navigation stack: takes a goal pose, plans a path, and drives the robot

---

## Prerequisites

Before starting, you should be comfortable with:
- Basic Linux terminal commands (`cd`, `ls`, `mkdir`, `nano`/`code`)
- Running a ROS2 node and understanding topics/services
- Building a ROS2 workspace with `colcon build`

If any of these are unfamiliar, spend 30 minutes on the official ROS2 Humble beginner tutorials first.

---

## Step 1 — Install the RealSense ROS2 Driver

The D455F connects over USB 3. We need a ROS2 package that reads it.

### 1.1 Install the librealsense SDK

```bash
# Add Intel's APT key and repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 1.2 Verify the camera is detected

Plug in the D455F, then run:

```bash
realsense-viewer
```

You should see a color stream and a depth stream. Close the viewer when done.

### 1.3 Install the ROS2 RealSense wrapper

```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

### 1.4 Test the ROS2 driver

Open three terminals. In each one, source ROS2 first:

```bash
source /opt/ros/humble/setup.bash
```

**Terminal 1** — launch the camera node:
```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=640x480x30 \
  pointcloud.enable:=true
```

**Terminal 2** — list active topics:
```bash
ros2 topic list
```

You should see topics like:
```
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/depth/color/points
```

**Terminal 3** — visualize in RViz2:
```bash
rviz2
```
In RViz2, add a `PointCloud2` display and set the topic to `/camera/camera/depth/color/points`. You should see a live 3D point cloud of the room.

---

## Step 2 — Understand the Camera's Coordinate Frame

Before doing navigation you must understand how the camera is physically mounted and how ROS2 represents its position.

**The D455F publishes its own TF frame** (usually `camera_link`). Nav2 needs to know where the camera is relative to the robot's base (`base_link`).

Create a static transform publisher that says "the camera is mounted X meters in front of and Y meters above the robot center":

```bash
# Example: camera is 0.1 m forward, 0.2 m up, no rotation
ros2 run tf2_ros static_transform_publisher \
  0.1 0.0 0.2  0.0 0.0 0.0  base_link camera_link
```

> **Tip for students:** Use a ruler to measure where you physically bolted the camera. The numbers in the command must match reality.

---

## Step 3 — Install SLAM Toolbox

SLAM Toolbox is the recommended ROS2 mapping package. It uses a 2D laser-scan interface, so we first convert the 3D depth image into a virtual 2D scan.

### 3.1 Install packages

```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-depthimage-to-laserscan \
  ros-humble-nav2-bringup \
  ros-humble-navigation2
```

### 3.2 Convert depth image to laser scan

The `depthimage_to_laserscan` node takes a horizontal slice of the depth image and produces a `/scan` topic that SLAM Toolbox can use.

Create a launch file `depth_to_scan.launch.py` in your workspace:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[{
                'scan_height': 10,       # rows of depth image to use
                'scan_time': 0.033,
                'range_min': 0.2,
                'range_max': 6.0,
                'output_frame': 'camera_link',
            }],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
        ),
    ])
```

Build and source your workspace, then launch:

```bash
ros2 launch your_package depth_to_scan.launch.py
```

Verify the scan topic exists:
```bash
ros2 topic echo /scan --once
```

---

## Step 4 — Build a Map with SLAM Toolbox

### 4.1 Launch SLAM in mapping mode

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

### 4.2 Drive the robot to explore the area

Publish velocity commands to `/cmd_vel` (use a joystick, keyboard teleop, or your CAN bridge) and drive around the entire arena slowly:

```bash
# Install keyboard teleop if not already installed
sudo apt install -y ros-humble-teleop-twist-keyboard

# Run it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Tip:** Drive slowly (< 0.3 m/s). Fast motion causes blurry depth and degrades the map quality.

### 4.3 Watch the map build in RViz2

Open RViz2 and add:
- `Map` display → topic `/map`
- `LaserScan` display → topic `/scan`
- `RobotModel` display

You will see the map grow as you explore.

### 4.4 Save the map

Once the arena is fully mapped:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/your_user/maps/arena_map'}"
```

This saves two files: `arena_map.yaml` (metadata) and `arena_map.pgm` (image).

---

## Step 5 — Localize in the Saved Map (AMCL)

AMCL (Adaptive Monte Carlo Localization) uses the laser scan to figure out where the robot is inside the saved map.

### 5.1 Launch the map server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/your_user/maps/arena_map.yaml
```

### 5.2 Launch AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  --params-file /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### 5.3 Set the initial pose in RViz2

In RViz2, click **"2D Pose Estimate"** and click on the map where the robot currently is, dragging in the direction the robot faces. AMCL will scatter particles around that guess and converge on the true pose as the robot moves.

---

## Step 6 — Launch the Full Nav2 Stack

Instead of launching nodes one-by-one, use the Nav2 bringup launch file:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/your_user/maps/arena_map.yaml \
  use_sim_time:=false \
  params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

This starts:
- Map server
- AMCL
- Costmaps (global + local)
- Planner (NavFn)
- Controller (DWB)
- Behavior tree navigator

### 6.1 Send a navigation goal from RViz2

In RViz2 click **"Nav2 Goal"** and click a destination on the map. The robot will plan a path and drive to the goal autonomously.

---

## Step 7 — Tune for Your Robot (Mecanum Drive)

Mecanum wheels can move sideways (holonomic). Tell Nav2 about this so it does not waste time turning.

In your `nav2_params.yaml`, find the `controller_server` section and set:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  # enable strafe (sideways) motion
  min_vel_x: -0.3
  max_vel_x: 0.3
  min_vel_y: -0.3    # allow lateral motion
  max_vel_y: 0.3
  min_speed_xy: 0.0
  max_speed_xy: 0.5
```

Also make sure your robot's odometry publisher reports `x`, `y`, and `theta` correctly, as mecanum odometry requires all three.

---

## Step 8 — Send Goals Programmatically (Python)

For the competition you will want code to send goals automatically, not a human clicking RViz2.

Create a file `go_to_goal.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw_deg=0.0):
        import math
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # convert yaw (degrees) to quaternion
        half = math.radians(yaw_deg) / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal_msg.pose = pose

        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Goal reached!')


def main():
    rclpy.init()
    nav = Navigator()
    nav.send_goal(x=1.5, y=0.5, yaw_deg=90)   # change to your target coordinates
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 go_to_goal.py
```

---

## Step 9 — Practice Workflow (Do This Every Session)

```
1. Power on robot, plug in D455F
2. source /opt/ros/humble/setup.bash
3. Launch realsense2_camera node
4. Launch depth_to_scan node
5. If mapping:
     Launch slam_toolbox  →  drive around  →  save map
   If navigating with saved map:
     Launch nav2_bringup with your map
6. Open RViz2 to monitor
7. Set initial pose (2D Pose Estimate)
8. Send goals (RViz2 or Python script)
```

---

## Troubleshooting Cheat Sheet

| Problem | What to check |
|---|---|
| No camera topics | USB 3 port? `realsense-viewer` detects camera? |
| `/scan` empty or noisy | `scan_height` too small; robot too close to walls |
| Map drifts during SLAM | Drive slower; check wheel odometry is publishing |
| AMCL particles scattered everywhere | Set initial pose manually in RViz2 |
| Robot spins in place | Mecanum `min_vel_y`/`max_vel_y` not set; check odometry |
| Nav2 can't find a path | Increase `inflation_radius` in costmap params |
| TF errors | Static transform publisher not running; check `ros2 run tf2_tools view_frames` |

---

## Glossary

| Term | Plain English |
|---|---|
| TF / Transform | A record of where one part of the robot is relative to another |
| Odometry | Estimated position calculated from wheel encoder ticks |
| Costmap | A grid that Nav2 uses to decide which cells are safe to drive through |
| SLAM | Building a map and knowing your position at the same time |
| AMCL | A way to locate yourself inside an already-built map |
| Point cloud | A set of thousands of 3D (x,y,z) points measured by the depth camera |
| Action | A ROS2 communication pattern for long-running tasks (like "drive to goal") |

---

## What's Next

Once navigation works reliably:
- Use the `/camera/camera/color/image_raw` topic + OpenCV to detect colored objects or AprilTags for game-specific tasks
- Add a behavior tree (Nav2 BT) to sequence multiple goals automatically
- Integrate your CAN-based motor driver so Nav2's `/cmd_vel` commands reach the wheels

Good luck at the competition!
