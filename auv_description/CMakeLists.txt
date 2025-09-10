cmake_minimum_required(VERSION 3.5)
project(auv_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

# Declare ament package
ament_package()
