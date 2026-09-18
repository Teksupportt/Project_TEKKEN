cmake_minimum_required(VERSION 3.8)
project(combat_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
