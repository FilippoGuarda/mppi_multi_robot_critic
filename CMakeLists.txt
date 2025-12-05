cmake_minimum_required(VERSION 3.16)
project(mppi_multi_robot_critic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# ============ Multi-Agent Interaction Critic Library ============
add_library(mppi_multi_robot_critic SHARED
  src/multi_agent_critic.cpp
)

target_include_directories(mppi_multi_robot_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mppi_multi_robot_critic
  rclcpp
  nav2_mppi_controller
  nav2_costmap_2d
  nav2_core
  geometry_msgs
  nav2_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
)

# Export as plugin
pluginlib_export_plugin_description_file(nav2_mppi_controller plugin.xml)

# ============ Installation ============
install(TARGETS mppi_multi_robot_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ============ Ament Package ============
ament_package()