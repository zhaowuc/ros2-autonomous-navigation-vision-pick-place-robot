cmake_minimum_required(VERSION 3.8)
project(n10p_lidar_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS scripts/check_n10p_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
