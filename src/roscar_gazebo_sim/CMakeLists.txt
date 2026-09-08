cmake_minimum_required(VERSION 3.8)
project(roscar_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds maps config models
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/pedestrian_controller.py
  scripts/run_sim_benchmark.py
  scripts/sim_base_controller.py
  scripts/scan_self_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
