cmake_minimum_required(VERSION 3.8)
project(so3_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# SO3Control library (pure control math, no ROS dependency)
add_library(SO3Control src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(SO3Control rclcpp)

# Composable node shared library
add_library(so3_control_component SHARED
  src/so3_control_nodelet.cpp
)
target_include_directories(so3_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(so3_control_component
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(so3_control_component SO3Control)

# Register the component
rclcpp_components_register_nodes(so3_control_component "so3_control::SO3ControlComponent")

# Standalone node executable
add_executable(so3_control_node src/so3_control_node.cpp)
target_include_directories(so3_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(so3_control_node
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(so3_control_node so3_control_component SO3Control)

# Control example executable
add_executable(control_example src/control_example.cpp)
target_include_directories(control_example PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(control_example
  rclcpp
  quadrotor_msgs
)

# Install targets
install(TARGETS
  SO3Control
  so3_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  so3_control_node
  control_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
