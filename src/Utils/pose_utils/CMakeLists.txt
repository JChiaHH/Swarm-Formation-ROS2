cmake_minimum_required(VERSION 3.8)
project(pose_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(pose_utils
  src/pose_utils.cpp
)

target_link_libraries(pose_utils
  ${ARMADILLO_LIBRARIES}
)

ament_target_dependencies(pose_utils
  rclcpp
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS pose_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)
ament_export_dependencies(rclcpp)

ament_package()
