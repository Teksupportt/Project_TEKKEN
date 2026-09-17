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
# Set ONNXRUNTIME_ROOT to wherever it's installed (e.g. via Dockerfile
# download/extract step). Override with -DONNXRUNTIME_ROOT=... if needed.
set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "Path to ONNX Runtime install")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include"
)
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS "${ONNXRUNTIME_ROOT}/lib"
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(WARNING
    "ONNX Runtime not found at ${ONNXRUNTIME_ROOT} — camera_ml_tracker will not build. "
    "Set ONNXRUNTIME_ROOT to its install location.")
endif()

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
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
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
endif()

# --- install ---
set(PERCEPTION_TARGETS
  lidar_2d_tracker
  lidar_3d_tracker
  camera_cv_tracker
)
if(TARGET camera_ml_tracker)
  list(APPEND PERCEPTION_TARGETS camera_ml_tracker)
endif()

install(TARGETS
  ${PERCEPTION_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
