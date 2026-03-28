cmake_minimum_required(VERSION 3.8)
project(traj_opt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(traj_utils REQUIRED)
find_package(path_searching REQUIRED)
find_package(swarm_graph REQUIRED)

add_library(traj_opt
  src/poly_traj_optimizer.cpp
)

target_include_directories(traj_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(traj_opt
  rclcpp
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
  path_searching
  swarm_graph
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS traj_opt
  EXPORT traj_optTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(traj_optTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp std_msgs visualization_msgs
  plan_env traj_utils path_searching swarm_graph
  Eigen3
)
ament_export_include_directories(include)

ament_package()
