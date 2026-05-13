cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

add_executable(eye_on_hand_calibrator src/eye_on_hand_calibrator.cpp)
ament_target_dependencies(eye_on_hand_calibrator
  rclcpp
  geometry_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  OpenCV
)

add_executable(calibration_perception src/calibration_perception.cpp)
ament_target_dependencies(calibration_perception
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(platform_teach src/platform_teach_node.cpp)
ament_target_dependencies(platform_teach
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(platform_teach Qt5::Widgets yaml-cpp)

install(TARGETS eye_on_hand_calibrator
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS calibration_perception platform_teach
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch calib
  DESTINATION share/${PROJECT_NAME})

ament_python_install_package(camera_calibration_gui
  PACKAGE_DIR python/camera_calibration_gui)

install(PROGRAMS scripts/camera_calibration_gui
  DESTINATION lib/${PROJECT_NAME})

ament_package()
