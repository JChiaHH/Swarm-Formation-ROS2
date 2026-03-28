cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(random_forest src/random_forest_sensing.cpp)

target_include_directories(random_forest PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(random_forest
  ${PCL_LIBRARIES}
)

install(TARGETS
  random_forest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
