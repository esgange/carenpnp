cmake_minimum_required(VERSION 3.8)
project(item_perception)

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
find_package(aruco_perception REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
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

add_executable(item_teach src/item_teach_node.cpp)
target_include_directories(item_teach PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(item_teach
  geometry_msgs
  rclcpp
  sensor_msgs
  dobot_msgs_v4
  cv_bridge
  tf2_ros
  OpenCV
)
target_link_libraries(item_teach yaml-cpp ${OpenCV_LIBS})

add_executable(item_detect src/item_detect_node.cpp)
target_include_directories(item_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(item_detect
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
target_link_libraries(item_detect yaml-cpp ${OpenCV_LIBS})

install(TARGETS bin_teach item_teach item_detect
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
