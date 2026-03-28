cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)

target_include_directories(poscmd_2_odom PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(poscmd_2_odom
  rclcpp
  nav_msgs
  quadrotor_msgs
)

install(TARGETS
  poscmd_2_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
