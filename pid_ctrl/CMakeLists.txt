cmake_minimum_required(VERSION 3.8)
project(pid_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(softsys_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marvelmind_ros2_msgs REQUIRED)

add_executable(pid_ctrl_node src/pid_ctrl_node.cpp src/pid.cpp)

include_directories(include)

# target_include_directories(image_filter_node PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pid_ctrl_node rclcpp sensor_msgs std_msgs softsys_msgs geometry_msgs marvelmind_ros2_msgs)

install(TARGETS pid_ctrl_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}
)

ament_package()
