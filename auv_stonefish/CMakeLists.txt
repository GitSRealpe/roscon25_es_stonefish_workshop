cmake_minimum_required(VERSION 3.8)
project(auv_stonefish)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stonefish_ros2 REQUIRED)

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

# Install launch files.
install(DIRECTORY
  launch
  scenarios
  resources
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/publish_propeller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Declare the executable
# add_executable(auv_body_feedback scripts/auv_body_feedback.cpp)
# Specify the dependencies for the executable
# ament_target_dependencies(
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   stonefish_ros2
# )
# Install the executable
install(TARGETS 
  # auv_body_feedback
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
