cmake_minimum_required(VERSION 3.8)
project(auv_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  Eigen3
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
  src/BodyPoseController.cpp
  src/BodyVelocityController.cpp
  src/BodyWrenchController.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# /////////////////////////////////////////////
generate_parameter_library(
  auv_pose_controller_params # cmake target name for the parameter library
  src/auv_pose_controller_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  auv_velocity_controller_params # cmake target name for the parameter library
  src/auv_velocity_controller_parameters.yaml # path to input yaml file
)
generate_parameter_library(
  auv_wrench_controller_params # cmake target name for the parameter library
  src/auv_wrench_controller_parameters.yaml # path to input yaml file
)

# //////////////////////////////////////////////////

# Link dependencies for the controller library
target_link_libraries(${PROJECT_NAME}
    auv_pose_controller_params
    auv_velocity_controller_params
    auv_wrench_controller_params
    )

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export the controller as a plugin
pluginlib_export_plugin_description_file(controller_interface auv_controllers_plugin.xml)

# Build the test executable
# add_executable(test_ctrl scripts/testing_controller.cpp)
# ament_target_dependencies(test_ctrl rclcpp std_msgs controller_interface)
# target_link_libraries(test_ctrl 
#     ${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the test executable
install(TARGETS 
  # test_ctrl
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the controller library
install(TARGETS ${PROJECT_NAME} 
  auv_pose_controller_params
  auv_velocity_controller_params 
  auv_wrench_controller_params
  EXPORT export_auv_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets and dependencies
ament_export_targets(export_auv_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()