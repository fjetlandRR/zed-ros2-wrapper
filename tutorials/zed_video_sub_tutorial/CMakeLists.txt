cmake_minimum_required(VERSION 3.5)
project(zed_video_sub_tutorial_ros2)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# includes
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

## Build
add_executable(zed_video_sub src/zed_video_sub_tutorial.cpp)
ament_target_dependencies(zed_video_sub
  rclcpp
  sensor_msgs
)

# Install executable
install(
    TARGETS zed_video_sub
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
