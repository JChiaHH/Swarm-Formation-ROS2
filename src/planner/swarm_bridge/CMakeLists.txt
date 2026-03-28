cmake_minimum_required(VERSION 3.10)
project(swarm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(bridge_node
  src/bridge_node.cpp
)

target_include_directories(bridge_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  traj_utils
)

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
