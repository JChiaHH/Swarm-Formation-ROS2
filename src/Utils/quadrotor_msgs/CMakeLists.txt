cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

# Build encode/decode libraries
add_library(decode_msgs src/decode_msgs.cpp)
target_include_directories(decode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(decode_msgs geometry_msgs)
rosidl_get_typesupport_target(decode_msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(decode_msgs ${decode_msgs_target})

add_library(encode_msgs src/encode_msgs.cpp)
target_include_directories(encode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(encode_msgs geometry_msgs)
rosidl_get_typesupport_target(encode_msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(encode_msgs ${encode_msgs_target})

# Install libraries
install(TARGETS decode_msgs encode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(decode_msgs encode_msgs)
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs nav_msgs)

ament_package()
