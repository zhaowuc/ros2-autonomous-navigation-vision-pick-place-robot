from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31014980-if00',
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('max_vx', default_value='0.20'),
        DeclareLaunchArgument('max_vy', default_value='0.20'),
        DeclareLaunchArgument('max_wz', default_value='0.45'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.12'),
        DeclareLaunchArgument('mode_switch_stop_seconds', default_value='0.04'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('boot_hold_seconds', default_value='0.0'),
        DeclareLaunchArgument('odom_source', default_value='feedback'),
        DeclareLaunchArgument('odom_vx_scale', default_value='1.1311'),
        DeclareLaunchArgument('odom_vy_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_wz_scale', default_value='1.2572'),
        DeclareLaunchArgument('odom_wz_pos_scale', default_value='1.2572'),
        DeclareLaunchArgument('odom_wz_neg_scale', default_value='1.2572'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),
        Node(
            package='c50c_base_driver',
            executable='c50c_base_driver_node',
            name='c50c_base_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'publish_odom_tf': ParameterValue(
                    LaunchConfiguration('publish_odom_tf'),
                    value_type=bool,
                ),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'max_vx': ParameterValue(LaunchConfiguration('max_vx'), value_type=float),
                'max_vy': ParameterValue(LaunchConfiguration('max_vy'), value_type=float),
                'max_wz': ParameterValue(LaunchConfiguration('max_wz'), value_type=float),
                'cmd_timeout': ParameterValue(LaunchConfiguration('cmd_timeout'), value_type=float),
                'mode_switch_stop_seconds': ParameterValue(
                    LaunchConfiguration('mode_switch_stop_seconds'),
                    value_type=float,
                ),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'boot_hold_seconds': ParameterValue(
                    LaunchConfiguration('boot_hold_seconds'),
                    value_type=float,
                ),
                'odom_source': LaunchConfiguration('odom_source'),
                'odom_vx_scale': ParameterValue(
                    LaunchConfiguration('odom_vx_scale'),
                    value_type=float,
                ),
                'odom_vy_scale': ParameterValue(
                    LaunchConfiguration('odom_vy_scale'),
                    value_type=float,
                ),
                'odom_wz_scale': ParameterValue(
                    LaunchConfiguration('odom_wz_scale'),
                    value_type=float,
                ),
                'odom_wz_pos_scale': ParameterValue(
                    LaunchConfiguration('odom_wz_pos_scale'),
                    value_type=float,
                ),
                'odom_wz_neg_scale': ParameterValue(
                    LaunchConfiguration('odom_wz_neg_scale'),
                    value_type=float,
                ),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'),
                    value_type=float,
                ),
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/odom', LaunchConfiguration('odom_topic')),
            ],
        ),
    ])
