cmake_minimum_required(VERSION 3.8)
project(ipmtool_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(invpersmap_pub src/IPMTool.cpp)

ament_target_dependencies(invpersmap_pub rclcpp OpenCV sensor_msgs cv_bridge)

install(TARGETS invpersmap_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
