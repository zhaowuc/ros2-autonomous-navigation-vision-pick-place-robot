from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('input_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('output_topic', default_value='/scan_slam_filtered'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'multi_echo_topic',
            default_value='/x10/multiecho_scan',
        ),
        DeclareLaunchArgument('prefer_multi_echo', default_value='true'),
        DeclareLaunchArgument('output_scan_bins', default_value='5400'),
        DeclareLaunchArgument('scan_max_stamp_skew_sec', default_value='1.0'),
        DeclareLaunchArgument('lower_angle', default_value='-3.14159265'),
        DeclareLaunchArgument('upper_angle', default_value='3.14159265'),
        DeclareLaunchArgument('range_min_limit', default_value='0.18'),
        DeclareLaunchArgument('range_max_limit', default_value='4.80'),
        DeclareLaunchArgument('median_window', default_value='0'),
        DeclareLaunchArgument('speckle_window', default_value='0'),
        DeclareLaunchArgument('speckle_max_delta', default_value='0.18'),
        DeclareLaunchArgument('speckle_min_neighbors', default_value='0'),
        DeclareLaunchArgument(
            'enable_motion_compensation',
            default_value='true',
        ),
        DeclareLaunchArgument('laser_offset_x', default_value='-0.235'),
        DeclareLaunchArgument('laser_offset_y', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_offset_yaw',
            default_value='0.092382808',
        ),
        DeclareLaunchArgument('far_guard_history_scans', default_value='50'),
        DeclareLaunchArgument('far_guard_max_age_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'far_guard_angular_window_deg',
            default_value='1.5',
        ),
        DeclareLaunchArgument(
            'far_guard_range_tolerance',
            default_value='0.20',
        ),
        DeclareLaunchArgument(
            'far_guard_jump_delta',
            default_value='0.25',
        ),
        DeclareLaunchArgument(
            'far_guard_min_confirmations',
            default_value='2',
        ),
        Node(
            package='scan_front_filter',
            executable='scan_front_filter_node',
            name='scan_front_filter',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'multi_echo_topic': LaunchConfiguration('multi_echo_topic'),
                'prefer_multi_echo': ParameterValue(
                    LaunchConfiguration('prefer_multi_echo'),
                    value_type=bool,
                ),
                'output_scan_bins': ParameterValue(
                    LaunchConfiguration('output_scan_bins'), value_type=int,
                ),
                'scan_max_stamp_skew_sec': ParameterValue(
                    LaunchConfiguration('scan_max_stamp_skew_sec'),
                    value_type=float,
                ),
                'lower_angle': LaunchConfiguration('lower_angle'),
                'upper_angle': LaunchConfiguration('upper_angle'),
                'range_min_limit': LaunchConfiguration('range_min_limit'),
                'range_max_limit': LaunchConfiguration('range_max_limit'),
                'median_window': ParameterValue(LaunchConfiguration('median_window'), value_type=int),
                'speckle_window': ParameterValue(LaunchConfiguration('speckle_window'), value_type=int),
                'speckle_max_delta': ParameterValue(
                    LaunchConfiguration('speckle_max_delta'),
                    value_type=float,
                ),
                'speckle_min_neighbors': ParameterValue(
                    LaunchConfiguration('speckle_min_neighbors'),
                    value_type=int,
                ),
                'enable_motion_compensation': ParameterValue(
                    LaunchConfiguration('enable_motion_compensation'),
                    value_type=bool,
                ),
                'laser_offset_x': ParameterValue(
                    LaunchConfiguration('laser_offset_x'),
                    value_type=float,
                ),
                'laser_offset_y': ParameterValue(
                    LaunchConfiguration('laser_offset_y'),
                    value_type=float,
                ),
                'laser_offset_yaw': ParameterValue(
                    LaunchConfiguration('laser_offset_yaw'),
                    value_type=float,
                ),
                'far_guard_history_scans': ParameterValue(
                    LaunchConfiguration('far_guard_history_scans'),
                    value_type=int,
                ),
                'far_guard_max_age_sec': ParameterValue(
                    LaunchConfiguration('far_guard_max_age_sec'),
                    value_type=float,
                ),
                'far_guard_angular_window_deg': ParameterValue(
                    LaunchConfiguration('far_guard_angular_window_deg'),
                    value_type=float,
                ),
                'far_guard_range_tolerance': ParameterValue(
                    LaunchConfiguration('far_guard_range_tolerance'),
                    value_type=float,
                ),
                'far_guard_jump_delta': ParameterValue(
                    LaunchConfiguration('far_guard_jump_delta'),
                    value_type=float,
                ),
                'far_guard_min_confirmations': ParameterValue(
                    LaunchConfiguration('far_guard_min_confirmations'),
                    value_type=int,
                ),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
