cmake_minimum_required(VERSION 3.5)
project(lidar_to_image_depth)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)

# Add executables for both nodes
add_executable(lidar_to_image_projection src/lidar_to_image_projection.cpp)
add_executable(lidar_to_image_depth src/lidar_to_image_depth.cpp)

# Link necessary libraries for lidar_to_image_projection node
ament_target_dependencies(lidar_to_image_projection
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  image_geometry
  OpenCV
)

# Link necessary libraries for lidar_to_image_depth
ament_target_dependencies(lidar_to_image_depth
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  image_geometry
  cv_bridge
  OpenCV
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(lidar_to_image_projection
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_link_libraries(lidar_to_image_depth
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Install both executables
install(TARGETS lidar_to_image_projection lidar_to_image_depth
  DESTINATION lib/${PROJECT_NAME})

ament_package()
