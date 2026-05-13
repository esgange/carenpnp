cmake_minimum_required(VERSION 3.8)
project(item_perception_yolo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dobot_msgs_v4 REQUIRED)
find_package(aruco_perception REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

add_executable(bin_teach src/bin_teach_node.cpp)
target_include_directories(bin_teach PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bin_teach
  aruco_perception
  dobot_msgs_v4
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(bin_teach Qt5::Widgets yaml-cpp)

install(TARGETS bin_teach
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/item_detect_yolo_node.py
  scripts/item_teach_yolo_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
