cmake_minimum_required(VERSION 3.8)
project(image_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(image_filter_node src/image_filter_node.cpp src/LF2.cpp src/LF_rebuild.cpp)

include_directories(include)

# target_include_directories(image_filter_node PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
target_compile_features(image_filter_node PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(image_filter_node rclcpp sensor_msgs cv_bridge std_msgs OpenCV geometry_msgs)
install(TARGETS image_filter_node
  DESTINATION lib/${PROJECT_NAME})

  # make sure we grab config folder
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()