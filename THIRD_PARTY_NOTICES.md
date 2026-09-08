# Third-party notices

The repository contains upstream ROS packages alongside the project-specific packages. Those components remain under their own licenses:

- [LSLiDAR ROS 2 driver](https://github.com/Lslidar/Lslidar_ROS2_driver): Apache-2.0, as declared by its package metadata.
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox): LGPL, with its license retained in `src/slam_toolbox/LICENSE`.
- Karto SDK bundled by SLAM Toolbox: LGPLv3, as declared in `src/slam_toolbox/lib/karto_sdk/package.xml` and its retained license file.

Package-level `package.xml` files are authoritative for other bundled package licenses. The root MIT license applies to the original ROSCAR packages and repository documentation, not to third-party components under a different license.
