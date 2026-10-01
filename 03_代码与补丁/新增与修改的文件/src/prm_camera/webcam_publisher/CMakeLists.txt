cmake_minimum_required(VERSION 3.10)
project(webcam_publisher)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)



find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()


include_directories(/usr/include)
include_directories(include)

add_executable(VideoCaptureNode src/VideoCaptureNode.cpp)
ament_target_dependencies(VideoCaptureNode
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "camera_info_manager"
  "image_transport"
  "camera_calibration_parsers"
  "cv_bridge"
  "OpenCV"
)



install(
  TARGETS VideoCaptureNode #VideoCaptureImageNode
  DESTINATION lib/${PROJECT_NAME}
)

# Install only the directories that actually exist: `resources/` holds optional sample
# footage and may be absent in a trimmed checkout, which used to abort the CMake install.
set(_webcam_share_dirs launch config)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resources")
  list(APPEND _webcam_share_dirs resources)
endif()
install(
  DIRECTORY ${_webcam_share_dirs}
  DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs)
ament_package()
