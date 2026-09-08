from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    IfElseSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def robot_state_publisher_node(use_sim_time):
    xacro_file = PathJoinSubstitution([
        FindPackageShare('roscar_description'),
        'urdf',
        'roscar_dofbot_temp.urdf.xacro',
    ])
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str,
            ),
            'use_sim_time': use_sim_time,
        }],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fused_odom = LaunchConfiguration('use_fused_odom')
    wheel_odom_topic = IfElseSubstitution(
        use_fused_odom,
        '/wheel/odom_raw',
        '/odom',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'c50c_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31014980-if00',
        ),
        DeclareLaunchArgument('c50c_baudrate', default_value='115200'),
        DeclareLaunchArgument('use_fused_odom', default_value='true'),
        DeclareLaunchArgument('legacy_imu_topic', default_value='/imu/legacy/data_raw'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/n300_imu',
        ),
        DeclareLaunchArgument('imu_baud_rate', default_value='115200'),
        DeclareLaunchArgument('imu_frame_id', default_value='n300_imu_link'),
        robot_state_publisher_node(use_sim_time),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('c50c_base_driver'),
                'launch',
                'c50c_base.launch.py',
            ])),
            launch_arguments={
                'port': LaunchConfiguration('c50c_port'),
                'baudrate': LaunchConfiguration('c50c_baudrate'),
                'imu_topic': LaunchConfiguration('legacy_imu_topic'),
                'odom_topic': wheel_odom_topic,
                'publish_odom_tf': NotSubstitution(use_fused_odom),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('n300pro_imu_driver'),
                'launch',
                'n300pro_imu.launch.py',
            ])),
            launch_arguments={
                'serial_port': LaunchConfiguration('imu_port'),
                'baud_rate': LaunchConfiguration('imu_baud_rate'),
                'frame_id': LaunchConfiguration('imu_frame_id'),
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_imu')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('roscar_bringup'),
                'launch',
                'n300_fusion.launch.py',
            ])),
            launch_arguments={
                'use_fused_odom': use_fused_odom,
                'wheel_odom_topic': wheel_odom_topic,
                'n300_imu_topic': '/imu/n300/data',
                'official_odom_topic': '/odom',
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
