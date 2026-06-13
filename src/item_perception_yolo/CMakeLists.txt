cmake_minimum_required(VERSION 3.8)
project(item_perception_yolo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/item_detect_yolo_node.py
  scripts/item_teach_yolo_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
