import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')

    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # Gazebo with the arena world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                os.path.join(pkg, 'worlds', 'arena.world'),
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen',
        ),

        # Robot state publisher — keeps TF frames up to date
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),


        # Publishes wheel joint states so robot_state_publisher can broadcast wheel TF                                                                                                                                                                                                                                            
        Node(                                                                                                                                                                                                                                                                                                                     
            package='joint_state_publisher',                                                                                                                                                                                                                                                                                      
            executable='joint_state_publisher',                                                                                                                                                                                                                                                                                   
            name='joint_state_publisher',                                                                                                                                                                                                                                                                                         
            parameters=[{'use_sim_time': True}],                                                                                                                                                                                                                                                                                  
       ),
        
        # Spawn the robot into Gazebo (delayed slightly so Gazebo is ready)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-topic', '/robot_description',
                        '-entity', 'my_robot',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.05',
                    ],
                ),
            ],
        ),

        # Convert depth image to 2D laser scan for SLAM / Nav2
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scan_height': 10,
                'scan_time': 0.033,
                'range_min': 0.2,
                'range_max': 8.0,
                'output_frame': 'camera_optical_link',
            }],
            remappings=[
                ('depth',            '/depth_camera/depth/image_raw'),
                ('depth_camera_info', '/depth_camera/depth/camera_info'),
                ('scan',             '/scan'),
            ],
        ),

        # RViz2 with pre-configured layout
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_sim.rviz')],
            parameters=[{'use_sim_time': True}],
        ),

    ])
