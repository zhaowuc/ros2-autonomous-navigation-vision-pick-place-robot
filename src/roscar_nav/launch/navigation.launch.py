from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    base_backend = LaunchConfiguration('base_backend')
    serial_device = LaunchConfiguration('serial_device')
    use_gui = LaunchConfiguration('use_gui')
    enable_auto_localize = LaunchConfiguration('enable_auto_localize')
    enable_lifecycle_fallback = LaunchConfiguration('enable_lifecycle_fallback')
    enable_localization_guard = LaunchConfiguration('enable_localization_guard')
    auto_return_home = LaunchConfiguration('auto_return_home')

    default_params = PathJoinSubstitution([
        FindPackageShare('roscar_nav'), 'config', 'nav2_params_roscar.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_real_hardware', default_value='false'),
        DeclareLaunchArgument('base_backend', default_value='sim'),
        DeclareLaunchArgument('serial_device', default_value='/dev/c50c_ros'),
        DeclareLaunchArgument('scan_range_max', default_value='4.8'),
        DeclareLaunchArgument('scan_prefer_multi_echo', default_value='true'),
        DeclareLaunchArgument('scan_output_bins', default_value='5400'),
        DeclareLaunchArgument('astra_obstacle_range', default_value='2.8'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('enable_auto_localize', default_value='false'),
        DeclareLaunchArgument('enable_lifecycle_fallback', default_value='false'),
        DeclareLaunchArgument('enable_localization_guard', default_value='true'),
        DeclareLaunchArgument('auto_return_home', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=str(Path.home() / 'roscar_maps/active.yaml')
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('roscar_bringup'), 'launch', 'robot_stack.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_real_hardware': use_real_hardware,
                'base_backend': base_backend,
                'serial_device': serial_device,
                'scan_range_max': LaunchConfiguration('scan_range_max'),
                'scan_prefer_multi_echo': LaunchConfiguration('scan_prefer_multi_echo'),
                'scan_output_bins': LaunchConfiguration('scan_output_bins'),
                'astra_obstacle_range': LaunchConfiguration('astra_obstacle_range'),
            }.items(),
        ),
        Node(
            package='roscar_command_arbiter', executable='command_arbiter',
            name='roscar_command_arbiter', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='roscar_navigation_mode_manager',
            executable='navigation_mode_manager',
            name='roscar_navigation_mode_manager', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'terminal_distance': 0.40}],
        ),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('roscar_nav'), 'launch', 'navigation_stack.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),
        Node(
            condition=IfCondition(enable_auto_localize),
            package='roscar_nav', executable='auto_localize',
            name='roscar_auto_localize', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_yaml': map_yaml,
                # Navigation publishes the real lidar filter on /scan_filtered;
                # /scan_slam_filtered only exists in mapping mode.
                'scan_topic': '/scan_filtered',
            }],
        ),
        Node(
            condition=IfCondition(enable_lifecycle_fallback),
            package='roscar_nav', executable='recover_localization_lifecycle',
            name='recover_localization_lifecycle', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(enable_localization_guard),
            package='roscar_nav', executable='localization_guard',
            name='roscar_localization_guard', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_yaml': map_yaml,
                'points_file': str(
                    Path.home() / '.ros/roscar_data/calibration_points.yaml'
                ),
                'home_point_name': '起点',
                'auto_return_home': ParameterValue(
                    auto_return_home, value_type=bool
                ),
            }],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='roscar_operator_gui', executable='operator_gui',
            name='roscar_operator_gui', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'map_yaml': map_yaml}],
        ),
    ])
