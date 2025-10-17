cmake_minimum_required(VERSION 3.8)
project(auv_architecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  geometry_msgs
  stonefish_ros2
  cv_bridge
  nav_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install launch files and other resources
install(DIRECTORY
  launch/
  scripts/
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Build the controller library
add_library(${PROJECT_NAME} SHARED
  src/EventRenderer.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Declare the auv_navigator_node executable
add_executable(auv_navigator_node scripts/auv_navigator_node.cpp)
ament_target_dependencies(auv_navigator_node rclcpp nav_msgs geometry_msgs)

# Build the renderer executable
add_executable(event_render_node scripts/event_render_node.cpp)
ament_target_dependencies(event_render_node PUBLIC rclcpp std_msgs stonefish_ros2 cv_bridge)
target_link_libraries(event_render_node PUBLIC
    ${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the test executable
install(TARGETS 
  auv_navigator_node
  event_render_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()