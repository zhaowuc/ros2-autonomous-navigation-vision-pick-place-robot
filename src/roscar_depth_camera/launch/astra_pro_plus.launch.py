"""Launch the correctly identified Astra Pro Plus and its bounded pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VENDOR_ID = '2bc5'
DEPTH_PRODUCT_ID = '060f'


def _read_sysfs(path):
    return Path(path).read_text(encoding='ascii').strip()


def _astra_usb_uid():
    matches = []
    for device_path in sorted(Path('/sys/bus/usb/devices').glob('*')):
        vendor_file = device_path / 'idVendor'
        product_file = device_path / 'idProduct'
        if not vendor_file.is_file() or not product_file.is_file():
            continue
        try:
            vendor = _read_sysfs(vendor_file).lower()
            product = _read_sysfs(product_file).lower()
        except OSError:
            continue
        if vendor == VENDOR_ID and product == DEPTH_PRODUCT_ID:
            matches.append(device_path.name)
    if len(matches) != 1:
        raise RuntimeError(
            'Expected exactly one Astra Pro Plus depth device '
            f'{VENDOR_ID}:{DEPTH_PRODUCT_ID}, found {matches}'
        )
    # Orbbec SDK v1 list_devices_node reports this sysfs topology string as
    # the UID (for example 3-4.4.1). Unlike bus/devnum it survives replugging.
    return matches[0]


def _launch_setup(context):
    try:
        usb_uid = _astra_usb_uid()
    except RuntimeError as exc:
        # The depth camera is an auxiliary local-obstacle sensor.  A missing
        # or ambiguous USB device must not take down the base, lidar, web UI,
        # odometry, or navigation launch graph.
        return [LogInfo(msg=f'WARNING: Astra Pro Plus disabled: {exc}')]
    official_launch = str(
        Path(get_package_share_directory('orbbec_camera'))
        / 'launch'
        / 'astra_pro_plus.launch.py'
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(official_launch),
        launch_arguments={
            'camera_name': 'astra',
            'usb_port': usb_uid,
            'vendor_id': '0x2bc5',
            'product_id': '0x060f',
            'depth_registration': 'false',
            'enable_depth': 'true',
            'depth_width': '640',
            'depth_height': '480',
            # This Astra Pro Plus exposes 640x480 only at 30 fps.  The
            # application pipeline below drops alternate frames to 15 fps.
            'depth_fps': '30',
            'depth_format': 'Y11',
            'depth_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
            'enable_color': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'color_format': 'MJPG',
            'color_qos': 'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'enable_ir': 'false',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',
            'point_cloud_qos': 'SENSOR_DATA',
            'ordered_pc': 'false',
            'publish_tf': 'false',
            'enable_publish_extrinsic': 'false',
            'enable_soft_filter': 'true',
            'enable_heartbeat': 'false',
        }.items(),
    )
    pipeline = Node(
        package='roscar_depth_camera',
        executable='depth_pipeline',
        name='astra_depth_pipeline',
        output='screen',
        parameters=[{
            'web_fps': ParameterValue(LaunchConfiguration('web_fps'), value_type=float),
            'pointcloud_fps': ParameterValue(
                LaunchConfiguration('pointcloud_fps'), value_type=float
            ),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'), value_type=int
            ),
            'preview_width': ParameterValue(
                LaunchConfiguration('preview_width'), value_type=int
            ),
            'preview_height': ParameterValue(
                LaunchConfiguration('preview_height'), value_type=int
            ),
            'depth_display_near_m': ParameterValue(
                LaunchConfiguration('depth_display_near_m'), value_type=float
            ),
            'depth_display_far_m': ParameterValue(
                LaunchConfiguration('depth_display_far_m'), value_type=float
            ),
            'obstacle_min_range_m': ParameterValue(
                LaunchConfiguration('obstacle_min_range_m'), value_type=float
            ),
            'obstacle_max_range_m': ParameterValue(
                LaunchConfiguration('obstacle_max_range_m'), value_type=float
            ),
            'point_sample_stride': ParameterValue(
                LaunchConfiguration('point_sample_stride'), value_type=int
            ),
            'voxel_size_m': ParameterValue(
                LaunchConfiguration('voxel_size_m'), value_type=float
            ),
        }],
    )
    return [
        LogInfo(
            msg=(
                'Astra Pro Plus locked to USB '
                f'{usb_uid} '
                '(PID 060f; Gemini 0614 is excluded)'
            )
        ),
        GroupAction(scoped=True, actions=[camera]),
        pipeline,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('web_fps', default_value='5.0'),
        DeclareLaunchArgument('pointcloud_fps', default_value='10.0'),
        DeclareLaunchArgument('jpeg_quality', default_value='68'),
        DeclareLaunchArgument('preview_width', default_value='320'),
        DeclareLaunchArgument('preview_height', default_value='240'),
        DeclareLaunchArgument('depth_display_near_m', default_value='0.60'),
        DeclareLaunchArgument('depth_display_far_m', default_value='4.00'),
        DeclareLaunchArgument('obstacle_min_range_m', default_value='0.55'),
        DeclareLaunchArgument('obstacle_max_range_m', default_value='2.80'),
        DeclareLaunchArgument('point_sample_stride', default_value='4'),
        DeclareLaunchArgument('voxel_size_m', default_value='0.03'),
        OpaqueFunction(function=_launch_setup),
    ])
