cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# ONNX Runtime — no standard find_package script on most systems.
# Always installed via the Dockerfile download/extract step.
# Override with -DONNXRUNTIME_ROOT=... if it lives elsewhere.
set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "Path to ONNX Runtime install")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include"
  REQUIRED
)
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS "${ONNXRUNTIME_ROOT}/lib"
  REQUIRED
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# --- lidar_2d_tracker ---
add_executable(lidar_2d_tracker
  src/lidar_2d_tracker_node.cpp
  src/lidar_2d_tracker_main.cpp
)
ament_target_dependencies(lidar_2d_tracker
  rclcpp sensor_msgs common_msgs perception_msgs
)
target_include_directories(lidar_2d_tracker PRIVATE include)

# --- lidar_3d_tracker ---
add_executable(lidar_3d_tracker
  src/lidar_3d_tracker_node.cpp
  src/lidar_3d_tracker_main.cpp
)
ament_target_dependencies(lidar_3d_tracker
  rclcpp sensor_msgs common_msgs perception_msgs pcl_conversions
)
target_include_directories(lidar_3d_tracker PRIVATE include)
target_link_libraries(lidar_3d_tracker ${PCL_LIBRARIES})

# --- camera_cv_tracker ---
add_executable(camera_cv_tracker
  src/camera_cv_tracker_node.cpp
  src/camera_cv_tracker_main.cpp
)
ament_target_dependencies(camera_cv_tracker
  rclcpp sensor_msgs common_msgs perception_msgs cv_bridge
)
target_include_directories(camera_cv_tracker PRIVATE include)
target_link_libraries(camera_cv_tracker ${OpenCV_LIBRARIES})

# --- camera_ml_tracker ---
add_executable(camera_ml_tracker
  src/camera_ml_tracker_node.cpp
  src/camera_ml_tracker_main.cpp
)
ament_target_dependencies(camera_ml_tracker
  rclcpp sensor_msgs common_msgs perception_msgs cv_bridge
)
target_include_directories(camera_ml_tracker PRIVATE
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_link_libraries(camera_ml_tracker
  ${OpenCV_LIBRARIES}
  ${ONNXRUNTIME_LIB}
)

# --- install ---
install(TARGETS
  lidar_2d_tracker
  lidar_3d_tracker
  camera_cv_tracker
  camera_ml_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
