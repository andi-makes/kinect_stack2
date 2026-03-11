cmake_minimum_required(VERSION 3.8)
project(kinect_stack2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)

pkg_check_modules(FREENECT REQUIRED libfreenect)

# Add freenect directory to include dirs
include_directories(
  include
  ${FREENECT_INCLUDE_DIRS}
)

# Declare our component
add_library(
  kinect_component
  SHARED
  src/kinect_component.cpp
)
# Add dependencies
ament_target_dependencies(kinect_component
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
  camera_info_manager
  image_transport
  rclcpp_components
)
target_link_libraries(kinect_component freenect rclcpp_components::component)
rclcpp_components_register_node(
  kinect_component
  PLUGIN "KinectNode"
  EXECUTABLE kinect_node
)
ament_export_targets(export_kinect_component)

# Install the component
install(TARGETS
  kinect_component
  EXPORT export_kinect_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install calibration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
