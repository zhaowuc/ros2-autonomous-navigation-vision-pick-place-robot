cmake_minimum_required(VERSION 3.8)
project(roscar_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/clean_ros_runtime.sh
  scripts/start_mapping_clean.sh
  scripts/wait_for_localization_health.py
  scripts/wait_for_navigation_health.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/clean_ros_runtime.sh
  scripts/record_n300_navigation_bag.sh
  scripts/start_mapping_clean.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
