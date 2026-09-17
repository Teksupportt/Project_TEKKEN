cmake_minimum_required(VERSION 3.8)
project(common_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Matrix2.msg"
  "msg/Matrix4.msg"
)

ament_package()
