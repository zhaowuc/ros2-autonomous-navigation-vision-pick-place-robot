import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_exists(package_name):
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            check=True,
            text=True,
            capture_output=True,
        )
    except Exception:
        return False
    return package_name in set(result.stdout.splitlines())


def _detect_driver_package():
    if _package_exists('lslidar_driver'):
        return 'lslidar_driver'
    return ''


def _launch_driver(context):
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baudrate = LaunchConfiguration('baudrate').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    publish_multiecho = (
        LaunchConfiguration('publish_multiecho').perform(context).strip().lower()
        in ('1', 'true', 'yes', 'on')
    )
    driver_package = LaunchConfiguration('driver_package').perform(context).strip()
    driver_executable = LaunchConfiguration('driver_executable').perform(context).strip()

    if not driver_package:
        driver_package = _detect_driver_package()

    if driver_package == 'lslidar_driver':
        return [
            LogInfo(msg='N10P: found lslidar_driver, starting X10/N10Plus mode.'),
            Node(
                package=driver_package,
                executable=driver_executable or 'lslidar_driver_node',
                name='lslidar_driver_node',
                namespace='x10',
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[{
                    'lidar_type': 'X10',
                    'lidar_model': 'N10Plus',
                    'serial_port': serial_port,
                    'frame_id': frame_id,
                    'laserscan_topic': scan_topic,
                    'pointcloud_topic': 'lslidar_point_cloud',
                    'publish_scan': True,
                    'use_high_precision': True,
                    'min_range': 0.15,
                    'max_range': 25.0,
                    'use_time_service': False,
                    'use_first_point_time': False,
                    'publish_multiecholaserscan': publish_multiecho,
                    'enable_noise_filter': False,
                    'N10Plus_hz': 10,
                    'use_sim_time': False,
                }],
            ),
        ]

    if driver_package and driver_executable:
        return [
            LogInfo(msg=f'N10P: starting user supplied driver {driver_package}/{driver_executable}.'),
            Node(
                package=driver_package,
                executable=driver_executable,
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[{
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                    'frame_id': frame_id,
                    'scan_topic': scan_topic,
                    'use_sim_time': False,
                }],
            ),
        ]

    return [LogInfo(
        msg=(
            'N10P: no compatible ROS2 driver package found. '
            'Install the Leishen/LSLiDAR N10P ROS2 driver, then rerun '
            '`ros2 launch n10p_lidar_bringup n10p.launch.py`.'
        )
    )]


def generate_launch_description():
    detected_driver = _detect_driver_package()

    actions = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/n10plus_lidar',
        ),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('publish_multiecho', default_value='true'),
        DeclareLaunchArgument('driver_package', default_value=detected_driver),
        DeclareLaunchArgument('driver_executable', default_value='lslidar_driver_node'),
        OpaqueFunction(function=_launch_driver),
    ]

    return LaunchDescription(actions)
