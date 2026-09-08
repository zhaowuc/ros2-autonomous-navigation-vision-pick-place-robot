from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("n300pro_imu_driver"), "config", "n300pro_imu.yaml"]
    )
    n300_params_file = LaunchConfiguration("n300_params_file")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    frame_id = LaunchConfiguration("frame_id")
    imu_topic = LaunchConfiguration("imu_topic")
    mag_topic = LaunchConfiguration("mag_topic")
    temperature_topic = LaunchConfiguration("temperature_topic")
    pressure_topic = LaunchConfiguration("pressure_topic")
    rpy_topic = LaunchConfiguration("rpy_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    expected_rate_hz = LaunchConfiguration("expected_rate_hz")
    minimum_rate_hz = LaunchConfiguration("minimum_rate_hz")
    stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("n300_params_file", default_value=default_params),
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/n300_imu"
                ),
            ),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="n300_imu_link"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/n300/data"),
            DeclareLaunchArgument("mag_topic", default_value="/imu/n300/mag"),
            DeclareLaunchArgument(
                "temperature_topic",
                default_value="/imu/n300/temperature",
            ),
            DeclareLaunchArgument(
                "pressure_topic",
                default_value="/imu/n300/pressure",
            ),
            DeclareLaunchArgument("rpy_topic", default_value="/imu/n300/rpy"),
            DeclareLaunchArgument(
                "diagnostics_topic",
                default_value="/diagnostics",
            ),
            DeclareLaunchArgument("expected_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("minimum_rate_hz", default_value="90.0"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.5"),
            Node(
                package="n300pro_imu_driver",
                executable="n300pro_imu_node",
                name="n300pro_imu",
                output="screen",
                parameters=[
                    n300_params_file,
                    {
                        "serial_port": serial_port,
                        "baud_rate": ParameterValue(baud_rate, value_type=int),
                        "frame_id": frame_id,
                        "imu_topic": imu_topic,
                        "mag_topic": mag_topic,
                        "temperature_topic": temperature_topic,
                        "pressure_topic": pressure_topic,
                        "rpy_topic": rpy_topic,
                        "diagnostics_topic": diagnostics_topic,
                        "expected_rate_hz": ParameterValue(
                            expected_rate_hz,
                            value_type=float,
                        ),
                        "minimum_rate_hz": ParameterValue(
                            minimum_rate_hz,
                            value_type=float,
                        ),
                        "stale_timeout_sec": ParameterValue(
                            stale_timeout_sec,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
