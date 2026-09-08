"""Launch the exact Gemini device and the red-platform alignment node."""

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
DEPTH_PRODUCT_ID = '0614'


def _read_sysfs(path):
    return Path(path).read_text(encoding='ascii').strip()


def _gemini_usb_uid():
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
            'Expected exactly one Gemini depth device '
            f'{VENDOR_ID}:{DEPTH_PRODUCT_ID}, found {matches}'
        )
    return matches[0]


def _launch_setup(context):
    try:
        usb_uid = _gemini_usb_uid()
    except RuntimeError as exc:
        return [LogInfo(msg=f'ERROR: Gemini visual alignment disabled: {exc}')]
    official_launch = str(
        Path(get_package_share_directory('orbbec_camera'))
        / 'launch'
        / 'gemini_uw.launch.py'
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(official_launch),
        launch_arguments={
            'camera_name': 'gemini',
            'usb_port': usb_uid,
            'vendor_id': '0x2bc5',
            'product_id': '0x0614',
            'depth_registration': 'true',
            'align_mode': 'HW',
            'enable_depth': 'true',
            'depth_width': '640',
            'depth_height': '400',
            'depth_fps': '10',
            'depth_format': 'Y11',
            'depth_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
            'enable_color': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '10',
            'color_format': 'MJPG',
            'color_qos': 'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'enable_color_auto_exposure': 'true',
            'enable_color_auto_white_balance': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'publish_tf': 'false',
            'enable_publish_extrinsic': 'false',
            'enable_heartbeat': 'false',
        }.items(),
    )
    alignment = Node(
        package='roscar_depth_camera',
        executable='gemini_alignment',
        name='gemini_visual_alignment',
        output='screen',
        parameters=[{
            'motion_enabled': ParameterValue(
                LaunchConfiguration('motion_enabled'), value_type=bool
            ),
            'reference_center_x': ParameterValue(
                LaunchConfiguration('reference_center_x'), value_type=float
            ),
            'reference_center_y': ParameterValue(
                LaunchConfiguration('reference_center_y'), value_type=float
            ),
        }],
    )
    return [
        LogInfo(msg=f'Gemini locked to USB {usb_uid} (PID 0614)'),
        GroupAction(scoped=True, actions=[camera]),
        alignment,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('motion_enabled', default_value='true'),
        DeclareLaunchArgument('reference_center_x', default_value='0.5'),
        DeclareLaunchArgument('reference_center_y', default_value='0.5'),
        OpaqueFunction(function=_launch_setup),
    ])
