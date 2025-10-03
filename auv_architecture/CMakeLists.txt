cmake_minimum_required(VERSION 3.8)
project(auv_architecture)

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

# Install launch files.
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Declare the executable
add_executable(auv_navigator_node scripts/auv_navigator_node.cpp)
# Specify the dependencies for the executable
ament_target_dependencies(auv_navigator_node
  rclcpp
  nav_msgs
  geometry_msgs
)
# Install the executable
install(TARGETS auv_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
