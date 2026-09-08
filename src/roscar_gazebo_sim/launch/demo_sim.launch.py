import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('roscar_gazebo_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    speed_profile = LaunchConfiguration('speed_profile')
    base_backend = LaunchConfiguration('base_backend')
    nav_params = PathJoinSubstitution([
        FindPackageShare('roscar_nav'), 'config', 'nav2_params_roscar.yaml'
    ])
    map_yaml = os.path.join(share, 'maps', 'corridor_demo.yaml')
    sim_points = os.path.expanduser('~/.ros/roscar_data/sim_points.yaml')
    description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('roscar_gazebo_sim'), 'urdf', 'roscar_sim.urdf.xacro'
        ]),
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
        ])),
        launch_arguments={
            'world': os.path.join(share, 'worlds', 'corridor_demo.world'),
            'gui': gazebo_gui,
            'verbose': 'false',
        }.items(),
    )
    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('roscar_nav'), 'launch', 'navigation_stack.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav_params,
            'autostart': 'true',
            # The simulation odometry is an exact kinematic observation.  Do
            # not inject fictitious wheel-slip noise into AMCL; real launches
            # retain their independently calibrated default motion model.
            'amcl_alpha': '0.0',
            'goal_xy_tolerance': '0.01',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('speed_profile', default_value='normal'),
        DeclareLaunchArgument('base_backend', default_value='sim'),
        DeclareLaunchArgument('ros_domain_id', default_value='77'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(share, 'models') + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', ''),
        ),
        gazebo,
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(description, value_type=str),
            }],
        ),
        Node(
            package='roscar_gazebo_sim', executable='scan_self_filter.py',
            name='roscar_sim_scan_filter', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'roscar', '-topic', 'robot_description',
                       '-x', '-3.0', '-y', '-2.0', '-z', '-0.1367'],
            output='screen',
        ),
        Node(
            package='roscar_command_arbiter', executable='command_arbiter',
            name='roscar_command_arbiter', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'speed_profile': speed_profile}],
        ),
        Node(
            package='roscar_base_interface', executable='base_interface_node',
            name='roscar_base_interface', output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('roscar_base_interface'),
                'config',
                'base_interface.yaml',
            ]), {
                'use_sim_time': use_sim_time,
                'base_backend': base_backend,
            }],
        ),
        Node(
            package='roscar_gazebo_sim', executable='sim_base_controller.py',
            name='roscar_sim_base', output='screen',
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
            parameters=[nav_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='roscar_gazebo_sim', executable='pedestrian_controller.py',
            name='pedestrian_controller', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'enabled': False}],
        ),
        nav_stack,
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('roscar_description'), 'rviz', 'mapping.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='roscar_operator_gui', executable='operator_gui',
            name='roscar_operator_gui', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'points_file': sim_points,
                'map_yaml': map_yaml,
                'odom_topic': '/wheel/odom_raw',
            }],
        ),
    ])
