cmake_minimum_required(VERSION 3.8)
project(tray_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dobot_msgs_v4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

add_executable(tray_teach_node src/tray_detector_node.cpp)
target_include_directories(tray_teach_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tray_teach_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(tray_teach_node yaml-cpp ${OpenCV_LIBS})

add_executable(tray_detect_node src/tray_detect_node.cpp)
target_include_directories(tray_detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tray_detect_node
  geometry_msgs
  rclcpp
  nav_msgs
  sensor_msgs
  std_srvs
  visualization_msgs
  dobot_msgs_v4
  cv_bridge
  tf2_ros
  OpenCV
)
target_link_libraries(tray_detect_node yaml-cpp ${OpenCV_LIBS})

install(TARGETS tray_teach_node tray_detect_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
