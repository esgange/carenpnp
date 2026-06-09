cmake_minimum_required(VERSION 3.8)
project(obstacle_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(obstacle_perception_node src/obstacle_perception_node.cpp)
target_include_directories(obstacle_perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(obstacle_perception_node
  rclcpp
  sensor_msgs
  visualization_msgs
  image_geometry
  cv_bridge
  OpenCV
  message_filters
)

add_executable(obstacle_memory_node src/obstacle_memory_node.cpp)
target_include_directories(obstacle_memory_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(obstacle_memory_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS obstacle_perception_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS obstacle_memory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
