cmake_minimum_required(VERSION 3.5)

project(rqt_slides)

find_package(ament_cmake REQUIRED)

if(WIN32)
  message(STATUS "rqt_slides is not yet supported on Windows. Package will not be built.")
  ament_package()
  return()
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(rqt_slides_SRCS
  src/rqt_slides/image_view.cpp
  src/rqt_slides/ratio_layouted_frame.cpp
)

set(rqt_slides_HDRS
  include/rqt_slides/image_view.h
  include/rqt_slides/ratio_layouted_frame.hpp
)

set(rqt_slides_UIS
  src/rqt_slides/image_view.ui
)

qt5_wrap_cpp(rqt_slides_MOCS ${rqt_slides_HDRS})

qt5_wrap_ui(rqt_slides_UIS_H ${rqt_slides_UIS})

add_library(${PROJECT_NAME} SHARED
  ${rqt_slides_SRCS}
  ${rqt_slides_MOCS}
  ${rqt_slides_UIS_H}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rclcpp_TARGETS}
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  image_transport::image_transport
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  Qt5::Widgets
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  cv_bridge::cv_bridge)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

install(PROGRAMS scripts/rqt_slides
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export new-style CMake variables
ament_export_targets(${PROJECT_NAME})

ament_package()
