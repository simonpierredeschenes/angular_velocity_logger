cmake_minimum_required(VERSION 3.8)
project(angular_velocity_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(angular_velocity_logger_node src/angular_velocity_logger_node.cpp)
ament_target_dependencies(angular_velocity_logger_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS angular_velocity_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
