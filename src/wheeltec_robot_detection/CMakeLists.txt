cmake_minimum_required(VERSION 3.8)
project(wheeltec_robot_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(wheeltec_robot_msg REQUIRED)

# Find CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# Find TensorRT
set(TensorRT_ROOT "/usr/local/TensorRT" CACHE PATH "TensorRT installation directory")
find_path(TensorRT_INCLUDE_DIR NvInfer.h
  HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include)
find_library(TensorRT_LIBRARY nvinfer
  HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)

if(NOT TensorRT_INCLUDE_DIR OR NOT TensorRT_LIBRARY)
  message(WARNING "TensorRT not found. C++ detection node will not be built.")
  message(WARNING "Python detection node is still available.")
else()
  message(STATUS "TensorRT found: ${TensorRT_LIBRARY}")

  include_directories(
    include
    ${TensorRT_INCLUDE_DIR}
  )

  # C++ Detection Node Library
  add_library(detection_node_component SHARED
    src/detection_node.cpp
  )

  ament_target_dependencies(detection_node_component
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    cv_bridge
    OpenCV
    wheeltec_robot_msg
  )

  target_link_libraries(detection_node_component
    ${TensorRT_LIBRARY}
    ${CUDA_LIBRARIES}
    nvinfer
    nvonnxparser
  )

  rclcpp_components_register_node(detection_node_component
    PLUGIN "wheeltec_robot_detection::DetectionNode"
    EXECUTABLE detection_node_cpp
  )

  # Standalone executable
  add_executable(detection_node_main
    src/detection_node_main.cpp
  )

  target_link_libraries(detection_node_main
    detection_node_component
  )

  # Install C++ components
  install(TARGETS
    detection_node_component
    detection_node_main
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# Python components (always available)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS wheeltec_robot_detection/detection_node.py DESTINATION lib/${PROJECT_NAME} RENAME detection_node)
install(PROGRAMS wheeltec_robot_detection/tracker_node.py DESTINATION lib/${PROJECT_NAME} RENAME tracker_node)

install(PROGRAMS wheeltec_robot_detection/video_stream_node.py DESTINATION lib/${PROJECT_NAME} RENAME video_stream_node)
install(PROGRAMS wheeltec_robot_detection/websocket_stream_node.py DESTINATION lib/${PROJECT_NAME} RENAME websocket_stream_node)
install(PROGRAMS wheeltec_robot_detection/dynamic_obstacle_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME dynamic_obstacle_publisher)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install models directory
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(detection_node_component)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  wheeltec_robot_msg
)

ament_package()
