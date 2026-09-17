cmake_minimum_required(VERSION 3.8)
project(heading_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

add_executable(heading_controller_node
  src/rate_limiter.cpp
  src/heading_controller_node.cpp
  src/heading_controller_main.cpp
)
ament_target_dependencies(heading_controller_node
  rclcpp std_msgs geometry_msgs perception_msgs
)

target_include_directories(heading_controller_node PRIVATE include)

install(TARGETS
  heading_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
