cmake_minimum_required(VERSION 3.8)
project(aruco_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetections.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  OpenCV
  cv_bridge
)
target_link_libraries(aruco_detector_node ${OpenCV_LIBS} yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aruco_detector_node "${cpp_typesupport_target}")

add_executable(perception_calibration src/perception_calibration.cpp)
ament_target_dependencies(perception_calibration
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(perception_calibration yaml-cpp)

install(TARGETS aruco_detector_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS perception_calibration
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
