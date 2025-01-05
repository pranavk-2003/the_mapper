cmake_minimum_required(VERSION 3.5)
project(ground_mapper)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)


# Add the executable for the mapping node
add_executable(mapping_node src/mapping_node.cpp)
add_executable(pointcloud_saver src/pointcloud_saver.cpp)
ament_target_dependencies(pointcloud_saver rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pointcloud_saver ${PCL_LIBRARIES})

install(TARGETS pointcloud_saver
  DESTINATION lib/${PROJECT_NAME})
# Link the dependencies
ament_target_dependencies(mapping_node rclcpp sensor_msgs nav_msgs tf2 tf2_ros)

# Install the executable so that ROS 2 can find it
install(TARGETS mapping_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# ament package
ament_package()
