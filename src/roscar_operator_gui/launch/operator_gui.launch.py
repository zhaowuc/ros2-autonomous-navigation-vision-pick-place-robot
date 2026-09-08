import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'points_file',
            default_value='~/.ros/roscar_data/calibration_points.yaml',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.expanduser('~/roscar_maps/active.yaml'),
        ),
        Node(
            package='roscar_operator_gui',
            executable='operator_gui',
            name='roscar_operator_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'points_file': LaunchConfiguration('points_file'),
                'map_yaml': LaunchConfiguration('map_yaml'),
            }],
        ),
    ])
