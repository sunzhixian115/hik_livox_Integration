cmake_minimum_required(VERSION 3.22)
project(hik_livox_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dist_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stamper_cpp src/stamper.cpp)
ament_target_dependencies(stamper_cpp rclcpp livox_interfaces)
add_executable(projector_cpp src/projector.cpp)
ament_target_dependencies(projector_cpp rclcpp livox_interfaces vision_msgs sensor_msgs dist_msg cv_bridge)
target_link_libraries(projector_cpp ${OpenCV_LIBS})
target_include_directories(projector_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(projector_cpp PRIVATE cxx_std_17)
target_compile_options(projector_cpp PRIVATE -O3)
install(TARGETS stamper_cpp projector_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
