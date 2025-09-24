cmake_minimum_required(VERSION 3.8)
project(stonefish_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  stonefish_ros2
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install launch files and other resources
install(DIRECTORY
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
  src/ThrusterGroupHWInterface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link dependencies for the controller library
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export the controller as a plugin
pluginlib_export_plugin_description_file(hardware_interface stonefish_hw_interface_plugin.xml)

# Build the test executable
add_executable(test_hw scripts/testing_hardware.cpp)
ament_target_dependencies(test_hw rclcpp std_msgs hardware_interface)
target_link_libraries(test_hw ${PROJECT_NAME})
# Install the test executable
install(TARGETS test_hw
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

#
# Install the controller library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_stonefish_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets and dependencies
ament_export_targets(export_stonefish_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()