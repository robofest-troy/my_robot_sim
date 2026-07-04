import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg, 'maps', 'arena_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map':          map_file,
                'use_sim_time': 'true',
                'params_file':  os.path.join(pkg, 'config', 'nav2_params.yaml'),
            }.items(),
        ),
    ])
