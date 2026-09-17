cmake_minimum_required(VERSION 3.8)
project(fusion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

add_executable(fusion_node
  src/ekf.cpp
  src/fusion_node.cpp
  src/fusion_main.cpp
)
ament_target_dependencies(fusion_node
  rclcpp std_msgs geometry_msgs common_msgs perception_msgs
)

target_include_directories(fusion_node PRIVATE include)

install(TARGETS
  fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
