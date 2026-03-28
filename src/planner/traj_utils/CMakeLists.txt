cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  "msg/Assignment.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(traj_utils_lib
  src/planning_visualization.cpp
)

target_include_directories(traj_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(traj_utils_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
)

# Get the typesupport target for this package's messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(traj_utils_lib "${cpp_typesupport_target}")

install(TARGETS traj_utils_lib
  EXPORT traj_utils_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(traj_utils_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  rosidl_default_runtime
)

ament_package()
