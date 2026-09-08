from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    base_backend = LaunchConfiguration('base_backend')
    serial_device = LaunchConfiguration('serial_device')
    filtered_scan_topic = LaunchConfiguration('filtered_scan_topic')
    description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('roscar_description'),
            'urdf',
            'roscar_dofbot_temp.urdf.xacro',
        ]),
    ])
    real_hardware = GroupAction(
        condition=IfCondition(use_real_hardware),
        actions=[
            GroupAction(
                scoped=True,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('n300pro_imu_driver'),
                        'launch',
                        'n300pro_imu.launch.py',
                    ])),
                    launch_arguments={
                        'serial_port': '/dev/n300_imu',
                        'frame_id': 'n300_imu_link',
                    }.items(),
                )],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('roscar_bringup'), 'launch', 'n300_fusion.launch.py'
                ])),
            ),
            GroupAction(
                scoped=True,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('n10p_lidar_bringup'),
                        'launch',
                        'n10p.launch.py',
                    ])),
                    launch_arguments={
                        'serial_port': '/dev/n10plus_lidar',
                        'frame_id': 'laser_link',
                    }.items(),
                )],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('scan_front_filter'),
                    'launch',
                    'scan_front_filter.launch.py',
                ])),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'input_topic': '/scan',
                    'output_topic': filtered_scan_topic,
                    'range_max_limit': LaunchConfiguration('scan_range_max'),
                    'prefer_multi_echo': LaunchConfiguration('scan_prefer_multi_echo'),
                    'output_scan_bins': LaunchConfiguration('scan_output_bins'),
                }.items(),
            ),
            GroupAction(
                scoped=True,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('roscar_depth_camera'),
                        'launch',
                        'astra_pro_plus.launch.py',
                    ])),
                    launch_arguments={
                        'obstacle_max_range_m': LaunchConfiguration('astra_obstacle_range'),
                    }.items(),
                )],
            ),
            Node(
                package='roscar_arm_driver', executable='arm_driver_node',
                name='roscar_arm', output='screen',
                parameters=[{'serial_port': '/dev/roscar_arm'}],
            ),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_real_hardware', default_value='false'),
        DeclareLaunchArgument('base_backend', default_value='sim'),
        DeclareLaunchArgument('serial_device', default_value='/dev/c50c_ros'),
        DeclareLaunchArgument('scan_range_max', default_value='4.8'),
        DeclareLaunchArgument('scan_prefer_multi_echo', default_value='true'),
        DeclareLaunchArgument('scan_output_bins', default_value='5400'),
        DeclareLaunchArgument('astra_obstacle_range', default_value='2.8'),
        DeclareLaunchArgument(
            'filtered_scan_topic',
            default_value='/scan_filtered',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(description, value_type=str),
            }],
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
                'serial_device': serial_device,
            }],
        ),
        real_hardware,
    ])
