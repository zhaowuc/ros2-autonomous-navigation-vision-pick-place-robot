from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/scan'),
        DeclareLaunchArgument('output_topic', default_value='/scan_slam_filtered'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        Node(
            package='scan_front_filter',
            executable='fake_scan_publisher',
            name='fake_scan_publisher',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('input_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': False,
            }],
        ),
        Node(
            package='scan_front_filter',
            executable='scan_front_filter_node',
            name='scan_front_filter',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'lower_angle': -3.14159265,
                'upper_angle': 3.14159265,
                'use_sim_time': False,
            }],
        ),
    ])
