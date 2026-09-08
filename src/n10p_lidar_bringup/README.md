# N10P Lidar Bringup

This package does not pretend that an N10P driver is installed. Run:

```bash
ros2 run n10p_lidar_bringup check_n10p_driver.sh
```

or:

```bash
bash install/n10p_lidar_bringup/share/n10p_lidar_bringup/scripts/check_n10p_driver.sh
```

The launch file first checks installed ROS2 packages for likely N10P/LSLiDAR
drivers. If `lslidar_driver` is present, it starts that driver in X10/N10Plus mode.
Otherwise it prints a clear warning and exits without publishing fake `/scan`
data. For WHEELTEC N10P serial models, install the official Leishen/LSLiDAR ROS2
driver that supports N10/N10Plus, then set `serial_port` as needed.
