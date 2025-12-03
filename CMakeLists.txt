cmake_minimum_required(VERSION 3.5)
project(mppi_multi_robot_critic)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/trajectory_aware_critic.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_core
  nav2_mppi_controller
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  Eigen3
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_mppi_controller critics_plugins.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES critics_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()