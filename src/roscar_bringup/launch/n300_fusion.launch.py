from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('roscar_bringup'),
        'config',
        'ekf_n300.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_fused_odom', default_value='true'),
        DeclareLaunchArgument('ekf_params_file', default_value=default_params),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom_raw'),
        DeclareLaunchArgument('n300_imu_topic', default_value='/imu/n300/data'),
        DeclareLaunchArgument(
            'official_odom_topic',
            default_value='/odom',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('ekf_params_file'),
                {
                    'publish_tf': ParameterValue(
                        LaunchConfiguration('use_fused_odom'),
                        value_type=bool,
                    ),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
            remappings=[
                ('/wheel/odom_raw', LaunchConfiguration('wheel_odom_topic')),
                ('/imu/n300/data', LaunchConfiguration('n300_imu_topic')),
                (
                    '/odometry/filtered',
                    LaunchConfiguration('official_odom_topic'),
                ),
            ],
            condition=IfCondition(LaunchConfiguration('use_fused_odom')),
        ),
    ])
