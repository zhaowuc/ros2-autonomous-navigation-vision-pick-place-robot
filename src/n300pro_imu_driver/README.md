# n300pro_imu_driver

Standalone ROS 2 Humble driver for the N300Pro IMU on car1. It decodes the
HiPNUC HI91 binary stream, validates every frame with CRC-16/CCITT, converts
device units to SI and publishes:

- `/imu/n300/data` (`sensor_msgs/Imu`)
- `/imu/n300/mag` (`sensor_msgs/MagneticField`)
- `/imu/n300/temperature` (`sensor_msgs/Temperature`)
- `/imu/n300/pressure` (`sensor_msgs/FluidPressure`)
- `/imu/n300/rpy` (`geometry_msgs/Vector3Stamped`, radians)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

`/imu/n300/rpy` is calculated from the normalized quaternion published in
`/imu/n300/data`, using standard ROS `roll, pitch, yaw` ordering. The HI91
native Euler payload has a different first-two-field ordering and is therefore
never published as ROS RPY. Its latest raw value is retained only in the
`raw_euler_native_deg` diagnostic field for protocol troubleshooting.

The standard sensor frame is `n300_imu_link`. Its measured installation
transform is `base_link -> n300_imu_link`, translation `(-0.170, 0, 0)` metres
and zero mounting rotation.

The configured hardware path is the stable USB by-id link:

```text
/dev/n300_imu
```

The detected protocol is 115200 8N1, HI91 tag `0x91`, 76-byte payload,
82-byte frame, nominal 100 Hz.

Run after building and sourcing the workspace:

```bash
ros2 launch n300pro_imu_driver n300pro_imu.launch.py
```

ROS message timestamps intentionally use the host ROS clock. The device's
`system_time` can change when its synchronization state changes and is tracked
only as a diagnostic counter.

The package is included by `roscar_bringup`. Publishing the N300Pro data does
not by itself make it part of wheel odometry, an EKF, or Nav2.

The node publishes the device axes and quaternion without an extra software
rotation. Before fusion, verify that the device is configured for ROS ENU and
that its calibrated body axes match the URDF `n300_imu_link` frame. Coordinate
corrections belong in the driver/device configuration and the measured static
TF, never in the web display.
