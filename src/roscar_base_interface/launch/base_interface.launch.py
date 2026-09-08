from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('roscar_base_interface')
    default_config = os.path.join(package_share, 'config', 'base_interface.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_backend',
            default_value='sim',
            description='Base backend: sim (default), mock, or serial',
        ),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=default_config,
            description='Unified base interface parameter file',
        ),
        DeclareLaunchArgument('serial_device', default_value='/dev/c50c_ros'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('command_tx_rate', default_value='50.0'),
        DeclareLaunchArgument('command_timeout', default_value='0.15'),
        DeclareLaunchArgument('feedback_expected_rate', default_value='50.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='roscar_base_interface',
            executable='base_interface_node',
            name='roscar_base_interface',
            output='screen',
            parameters=[
                LaunchConfiguration('base_params_file'),
                {
                    'base_backend': LaunchConfiguration('base_backend'),
                    'serial_device': LaunchConfiguration('serial_device'),
                    'baudrate': ParameterValue(
                        LaunchConfiguration('baudrate'),
                        value_type=int,
                    ),
                    'command_tx_rate': ParameterValue(
                        LaunchConfiguration('command_tx_rate'),
                        value_type=float,
                    ),
                    'command_timeout': ParameterValue(
                        LaunchConfiguration('command_timeout'),
                        value_type=float,
                    ),
                    'feedback_expected_rate': ParameterValue(
                        LaunchConfiguration('feedback_expected_rate'),
                        value_type=float,
                    ),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
