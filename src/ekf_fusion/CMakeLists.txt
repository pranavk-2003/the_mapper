cmake_minimum_required(VERSION 3.5)
project(ekf_fusion)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
