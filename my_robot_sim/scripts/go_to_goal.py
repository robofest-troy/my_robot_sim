#!/usr/bin/env python3
"""
Send a sequence of navigation goals to Nav2.
Run this AFTER nav2_launch.py is active and the initial pose is set in RViz2.

Usage:
    python3 go_to_goal.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_to(self, x: float, y: float, yaw_deg: float = 0.0) -> bool:
        """Drive to (x, y) facing yaw_deg degrees. Returns True on success."""
        self.get_logger().info(f'Waiting for Nav2 action server...')
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

        self.get_logger().info(f'Sending goal  x={x:.2f}  y={y:.2f}  yaw={yaw_deg:.0f} deg')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by Nav2')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Goal REACHED')
        return True


def main():
    rclpy.init()
    nav = Navigator()

    # -----------------------------------------------------------
    # Edit this list to change the waypoints your robot visits.
    # Format: (x_meters, y_meters, yaw_degrees)
    # The arena is 6 m x 6 m; origin is the centre.
    # -----------------------------------------------------------
    waypoints = [
        ( 1.5,  1.5,   0.0),   # top-right corner area
        ( 1.5, -1.5,  90.0),   # bottom-right corner area
        (-1.5, -1.5, 180.0),   # bottom-left corner area
        (-1.5,  1.5, 270.0),   # top-left corner area
        ( 0.0,  0.0,   0.0),   # back to centre / home
    ]

    for x, y, yaw in waypoints:
        success = nav.go_to(x, y, yaw)
        if not success:
            nav.get_logger().error('Navigation failed — stopping.')
            break

    nav.get_logger().info('All waypoints done!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
