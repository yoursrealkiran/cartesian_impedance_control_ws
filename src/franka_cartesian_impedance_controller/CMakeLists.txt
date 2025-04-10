cmake_minimum_required(VERSION 3.14)
project(franka_cartesian_impedance_controller)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Controller library
add_library(franka_cartesian_impedance_controller SHARED
  src/cartesian_impedance_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  hardware_interface
  franka_semantic_components
  Eigen3
  moveit_core
  moveit_msgs
  geometry_msgs
)

# Export plugin description
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
