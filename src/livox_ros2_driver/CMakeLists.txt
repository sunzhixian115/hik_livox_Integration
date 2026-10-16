# Copyright(c) 2020 livoxtech limited.
 
cmake_minimum_required(VERSION 3.5)
project(livox_ros2_driver)
 
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()
 
# 启用政策以消除PCL_ROOT警告
cmake_policy(SET CMP0074 NEW)
 
# Printf version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER_VERSION}")
 
# 恢复ament_cmake_auto依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
 
# 关键修正：查找编译后的接口包名 livox_interfaces（而非源码包名）
find_package(livox_interfaces REQUIRED)
find_package(livox_sdk REQUIRED)

find_package(PCL REQUIRED)
 
# 检查apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)
 
# PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
# livox ros2 driver target
ament_auto_add_library(${PROJECT_NAME} SHARED
  livox_ros2_driver/lvx_file.cpp
  livox_ros2_driver/ldq.cpp
  livox_ros2_driver/lds.cpp
  livox_ros2_driver/lds_lvx.cpp
  livox_ros2_driver/lds_lidar.cpp
  livox_ros2_driver/lds_hub.cpp
  livox_ros2_driver/lddc.cpp
  livox_ros2_driver/livox_ros2_driver.cpp
  timesync/timesync.cpp
  timesync/user_uart/user_uart.cpp
  common/comm/comm_protocol.cpp
  common/comm/sdk_protocol.cpp
  common/comm/gps_protocol.cpp
)
 
target_compile_options(${PROJECT_NAME}
  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-missing-field-initializers -fPIC>
)
 
# include file directory
target_include_directories(${PROJECT_NAME} PUBLIC
  ${livox_sdk_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${livox_interfaces_INCLUDE_DIRS}  # 接口包头文件路径
  common
  common/rapidjson
  common/rapidxml
  common/comm
  timesync
  timesync/user_uart
  livox_ros2_driver
)
 
# 链接库时添加interfaces依赖
target_link_libraries(${PROJECT_NAME}
	livox::livox
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
	${APR_LIBRARIES}
	${livox_interfaces_LIBRARIES}  # 接口包库文件
	usb-1.0
)
 
# 声明依赖关系（确保编译顺序）
ament_target_dependencies(${PROJECT_NAME}
  livox_interfaces
)
 
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::LivoxDriver"
  EXECUTABLE ${PROJECT_NAME}_node
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
