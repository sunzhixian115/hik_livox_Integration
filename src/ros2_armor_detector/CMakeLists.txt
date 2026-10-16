cmake_minimum_required(VERSION 3.22)
project(ros2_armor_detector)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(NNDEPLOYMENT_ENABLE_TENSORRT OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../NNdeployment
  ${CMAKE_CURRENT_BINARY_DIR}/NNdeployment
)

add_executable(armor_detector_node src/armor_detector_node.cpp)
target_include_directories(armor_detector_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/core/algorithm/NNdeployment/interface
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(armor_detector_node
  NNdeployment_lib
  ${OpenCV_LIBS}
)
set_target_properties(armor_detector_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/NNdeployment/NNdeployment"
  INSTALL_RPATH "$ORIGIN/.."
)
ament_target_dependencies(armor_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
)

install(TARGETS armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS NNdeployment_lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
