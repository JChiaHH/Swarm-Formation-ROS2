cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_executable(pcl_render_node
  src/pointcloud_render_node.cpp
)

target_include_directories(pcl_render_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(pcl_render_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  quadrotor_msgs
)

target_link_libraries(pcl_render_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcl_render_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_package()
