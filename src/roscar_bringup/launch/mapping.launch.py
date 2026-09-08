from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    base_backend = LaunchConfiguration('base_backend')
    serial_device = LaunchConfiguration('serial_device')
    control_params_file = LaunchConfiguration('control_params_file')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    default_params = PathJoinSubstitution([
        FindPackageShare('roscar_bringup'), 'config', 'mapping_control.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_real_hardware', default_value='false'),
        DeclareLaunchArgument('base_backend', default_value='sim'),
        DeclareLaunchArgument('serial_device', default_value='/dev/c50c_ros'),
        DeclareLaunchArgument('control_params_file', default_value=default_params),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('scan_range_max', default_value='10.0'),
        DeclareLaunchArgument('astra_obstacle_range', default_value='4.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('roscar_bringup'), 'launch', 'robot_stack.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_real_hardware': use_real_hardware,
                'base_backend': base_backend,
                'serial_device': serial_device,
                'filtered_scan_topic': '/scan_slam_filtered',
                'scan_range_max': LaunchConfiguration('scan_range_max'),
                # Deskew raw fine-bearing data before compacting to physical
                # beam bins. Sparse 5400-bin scans depress Karto match scores.
                'scan_prefer_multi_echo': 'false',
                'scan_output_bins': '540',
                'astra_obstacle_range': LaunchConfiguration('astra_obstacle_range'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('roscar_slam'), 'launch', 'slam.launch.py'
            ])),
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'max_laser_range': LaunchConfiguration('scan_range_max'),
            }.items(),
        ),
        # Mapping is manually driven by the native GUI.  Keep the same single
        # command authority used by navigation; only the planner/localization
        # portion of Nav2 is omitted while SLAM owns the map frame.
        Node(
            package='roscar_command_arbiter', executable='command_arbiter',
            name='roscar_command_arbiter', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[control_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', '/cmd_vel_selected'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
        ),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output='screen',
            parameters=[control_params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_mapping_control', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['velocity_smoother', 'collision_monitor'],
            }],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='roscar_operator_gui', executable='operator_gui',
            name='roscar_operator_gui', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2', executable='rviz2',
            name='mapping_rviz2', output='log',
            remappings=[('/scan_filtered', '/scan_slam_filtered')],
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('roscar_description'),
                'rviz',
                'mapping.rviz',
            ])],
        ),
    ])
