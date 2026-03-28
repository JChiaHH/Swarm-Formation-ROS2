cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ode>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(quadrotor_dynamics
  Eigen3
)

add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp
)
target_link_libraries(quadrotor_simulator_so3
  quadrotor_dynamics
)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  uav_utils
  Eigen3
)

install(TARGETS quadrotor_simulator_so3
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
