from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('align_scan_topic', default_value='/scan_align_filtered'),
        DeclareLaunchArgument('done_topic', default_value='/wall_align_done'),
        DeclareLaunchArgument('status_topic', default_value='/wall_align/status'),
        DeclareLaunchArgument('mode', default_value='front'),
        DeclareLaunchArgument('target_mode', default_value='parallel'),
        DeclareLaunchArgument('wall_search_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('timeout_sec', default_value='25.0'),
        Node(
            package='roscar_wall_align',
            executable='wall_align_node',
            name='wall_align_node',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'align_scan_topic': LaunchConfiguration('align_scan_topic'),
                'done_topic': LaunchConfiguration('done_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'mode': LaunchConfiguration('mode'),
                'target_mode': LaunchConfiguration('target_mode'),
                'wall_search_timeout_sec': ParameterValue(
                    LaunchConfiguration('wall_search_timeout_sec'),
                    value_type=float,
                ),
                'timeout_sec': ParameterValue(LaunchConfiguration('timeout_sec'), value_type=float),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
